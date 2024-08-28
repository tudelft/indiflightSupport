# PyNDIflight simulation definition for N randomized throw-to-hover experiments.
#
# Copyright 2024 Till Blaha (Delft University of Technology)
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <https://www.gnu.org/licenses/>.


import numpy as np
from tqdm import tqdm
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter, ArgumentTypeError
import threading
import os
import sys

from PyNDIflight.crafts import MultiRotor, Rotor, IMU
from PyNDIflight.interfaces import IndiflightSITLWrapper, visApp
from PyNDIflight.sim import Sim

import pickle

np.random.seed(42)
from numpy.random import random, randint, permutation

def runOneSim(mc, imu, args):
    # disable all console output from the cdll
    devnull = os.open(os.devnull, os.O_WRONLY)
    orig_stdout = sys.stdout.fileno()
    orig_stderr = sys.stderr.fileno()
    saved_stdout = os.dup(orig_stdout)
    saved_stderr = os.dup(orig_stderr)

    if not args.verbose:
        os.dup2(devnull, orig_stdout)
        os.dup2(devnull, orig_stderr)
        os.close(devnull)

    # load, configure and initialize indiflight.
    sil = IndiflightSITLWrapper(mc, imu, args.sil, N=len(mc.rotors))
    sil.mockup.load_profile( args.sil_profile_txt ) if args.sil_profile_txt else None
    sil.mockup.setLogging( args.sil_log )

    # set setpoints, flight modes and switch positions
    sil.mockup.sendPositionSetpoint( [0., 0., -1.5], 0. )
    sil.mockup.enableFlightMode(flightModeFlags.ANGLE_MODE
                                | flightModeFlags.POSITION_MODE
                                | flightModeFlags.LEARNER_MODE)

    sil.mockup.enableRxBox(boxId.BOXTHROWTOARM
                           | boxId.BOXARM)

    sim = Sim(mc, imu, sil=sil)

    if not args.verbose:
        # always print tqdm bar
        os.dup2(saved_stdout, orig_stdout)
        os.dup2(saved_stderr, orig_stderr)
        os.close(saved_stdout)
        os.close(saved_stderr)

    # run loop
    dt = 0.0005 # 2kHz
    T = 8. # seconds
    itermax = int(T / dt)
    dt_rt = None if args.no_real_time else dt
    for _ in tqdm(range(itermax), target_looptime=dt_rt, leave=False): 
        # leave=False makes the nested progress bar look proper
        sim.tick(dt)


if __name__=="__main__":
    parser = ArgumentParser(formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument("--sil", type=str, default=None, help="Load INDIflight interface as shared library.")
    parser.add_argument("--sil-profile-txt", required=False, type=str, metavar="PROFILE.txt", help="Import these profile settings into the SIL")
    parser.add_argument("--sil-log", required=False, action="store_true", help="Write Indiflight logs into ./logs")
    parser.add_argument("--no-vis", required=False, action="store_true", help="Do not launch visualization webserver")
    parser.add_argument("--no-real-time", required=False, action="store_true", help="Run as fast as possible")
    parser.add_argument("--num", required=False, type=int, default=20, help="Number of randomized runs")
    parser.add_argument("--verbose", required=False, action="store_true", help="Print debugging output from INDIflight")
    args = parser.parse_args()

    if not os.path.isfile(args.sil):
        raise FileNotFoundError("Library not found. Likely you didnt compile it yet. In external/indiflight, run `make TARGET=MOCKUP so`")

    # import python indiflight bindings
    sys.path.append(os.path.join((os.path.dirname(args.sil)), "../../src/utils"))
    from indiflight_mockup_interface import flightModeFlags, boxId

    if not args.no_vis:
        print("\n\n##########################################\n")
        print(f"Welcome to the sim -- Starting visualization at http://localhost:5000")
        visThread = threading.Thread(target=visApp.run, daemon=True, kwargs={'host':'0.0.0.0'})
        visThread.start( )

    os.makedirs('./logs', exist_ok=True)

    N = args.num
    uavs = []
    for j in tqdm(range(N)):
        ableToHover = False
        while not ableToHover:
            n_rotors = 4
            throw_height = 4.
            throw_rotation = 10.*random(3) - 5.
            throw_direction = 4.*random(2) - 2.

            offset = np.pi/3. * random(n_rotors)
            phi = random(n_rotors) * np.pi/3  +  np.arange(0, 2.*np.pi, np.pi/2.)
            R = random(n_rotors) * 0.1 + 0.05
            X = R * np.stack(( np.cos(phi), np.sin(phi), np.zeros(n_rotors) ))
            X[2, :] = -0.02

            kappas = random(n_rotors) * 0.5 + 0.25
            taus = random(n_rotors) * 0.02 + 0.015
            cms = random(n_rotors) * 0.01 + 0.01

            dirs = ["lh", "rh", "lh", "rh"] if randint(0,2) else ["rh", "lh", "rh", "lh"]

            mc = MultiRotor()
            uavs.append(mc)
            mc.setInertia(m=.45, I=np.diag([0.75e-3, 0.75e-3, 0.75e-3]))
            perm = permutation(range(n_rotors))
            for i in perm:
                mc.addRotor(Rotor(r=X[:, i], Tmax=5., kESC=kappas[i], tau=taus[i], Izz=5e-7, cm=cms[i], dir=dirs[i]))

            G1, _, _ = mc.calculateG1G2()
            Qr, Rr = np.linalg.qr(G1[3:, :].T, 'complete')
            Nr = Qr[:, -1] # rotational nullspace
            Nrmin = min(Nr)
            Nrmax = max(Nr)
            if Nrmax <= 0:
                Nr = -Nr

            if min(Nr) < 0:
                print(f"generated craft has no rotational nullspace within actuator limits and is thus unable to hover. Regenerating...")
                continue
            elif max(Nr) == 0. or (G1[2, :] @ (Nr / max(Nr)) > -1.2*9.81): # require at least 1.2 thrust to weight
                print(f"generated craft does not have enough thrust-to-weight to hover without rotation. Regenerating...")
                continue

            ableToHover = True

        # initial conditions
        mc.setPose(x=[0., 0., -0.1], q=[1., 0., 0., 0.])
        mc.setTwist(v=[0., 0., 0.], w=[0., 0., 0.])

        # throw config
        mc.throw(height=throw_height, acc=40., wB=throw_rotation, vHorz=throw_direction, at_time=0.5)

        with open(f'./logs/copter_{j:05}.pickle', 'wb') as file:
            G1, G2, Gs = mc.calculateG1G2()
            pickle.dump({
                'G1': G1,
                'G2': G2,
                'Gs': Gs,
                'kappas': kappas[perm],
                'taus': taus[perm],
                'cms': cms[perm],
                'wmax': np.array([r.wmax for r in mc.rotors])
            }, file, protocol=pickle.HIGHEST_PROTOCOL)

        imu = IMU(mc, r=[0., 0., 0.], qBody=[0., 0., 0., 1.], accStd=0.8, gyroStd=0.08)

        runOneSim(mc, imu, args)
