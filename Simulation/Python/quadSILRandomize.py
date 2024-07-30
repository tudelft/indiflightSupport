#!/usr/bin/env python3

import numpy as np
from tqdm import tqdm
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter, ArgumentTypeError
import threading
import os
import sys

from crafts import MultiRotor, Rotor, IMU
from interfaces import IndiflightSITLWrapper, visApp
from sim import Sim

import pickle

N = 100
np.random.seed(42)
from numpy.random import random

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
    sil = IndiflightSITLWrapper(mc, imu, args.library, N=len(mc.rotors))
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
    parser.add_argument("library", type=str, default=None, help="Load INDIflight interface as shared library.")
    parser.add_argument("--sil-profile-txt", required=False, type=str, metavar="PROFILE.txt", help="Import these profile settings into the SIL")
    parser.add_argument("--sil-log", required=False, action="store_true", help="Write Indiflight logs into ./logs")
    parser.add_argument("--no-vis", required=False, action="store_true", help="Do not launch visualization webserver")
    parser.add_argument("--no-real-time", required=False, action="store_true", help="Run as fast as possible")
    parser.add_argument("--verbose", required=False, action="store_true", help="Print debugging output from INDIflight")
    args = parser.parse_args()

    if not os.path.isfile(args.library):
        raise FileNotFoundError("Library not found. Likely you didnt compile it yet. In external/indiflight, run `make TARGET=MOCKUP so`")

    # import python indiflight bindings
    sys.path.append(os.path.join((os.path.dirname(args.library)), "../../src/utils"))
    from indiflight_mockup_interface import flightModeFlags, boxId

    if not args.no_vis:
        print("\n\n##########################################\n")
        print(f"Welcome to the sim -- Starting visualization at http://localhost:5000")
        visThread = threading.Thread(target=visApp.run, daemon=True, kwargs={'host':'0.0.0.0'})
        visThread.start( )

    try:
        os.mkdir('./logs')
    except OSError:
        pass

    uavs = []
    for i in tqdm(range(N)):
        mass = .2 + 0.5 * random()
        I = 1e-4 + 2e-3 * random(3)
        n_rotors = 4
        throw_height = 3. + 2.*random()
        throw_rotation = 10.*random(3) - 5.
        throw_direction = 4.*random(2) - 2.

        mc = MultiRotor()
        uavs.append(mc)
        mc.setInertia(m=mass, I=np.diag(I))
        mc.addRotor(Rotor(r=[-0.05, +0.0635, 0.0], Tmax=5., kESC=0.5, tau=0.02, Izz=5e-7, dir='lh')) # RR
        mc.addRotor(Rotor(r=[+0.05, +0.0635, 0.0], Tmax=5., kESC=0.5, tau=0.02, Izz=5e-7, dir='rh')) # FR
        mc.addRotor(Rotor(r=[-0.05, -0.0635, 0.0], Tmax=5., kESC=0.5, tau=0.02, Izz=5e-7, dir='rh')) # RL
        mc.addRotor(Rotor(r=[+0.05, -0.0635, 0.0], Tmax=5., kESC=0.5, tau=0.02, Izz=5e-7, dir='lh')) # FL

        # initial conditions
        mc.setPose(x=[0., 0., -0.1], q=[1., 0., 0., 0.])
        mc.setTwist(v=[0., 0., 0.], w=[0., 0., 0.])

        # throw config
        mc.throw(height=throw_height, acc=40., wB=throw_rotation, vHorz=throw_direction, at_time=0.5)

        with open(f'./logs/copter_{i:05}.pickle', 'wb') as file:
            pickle.dump(mc, file, protocol=pickle.HIGHEST_PROTOCOL)

        imu = IMU(mc, r=[0., 0., 0.], qBody=[0., 0., 0., 1.], accStd=0.8, gyroStd=0.08)

        runOneSim(mc, imu, args)
