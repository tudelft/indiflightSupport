# PyNDIflight simulation definition for a multirotor.
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
from PyNDIflight.interfaces import Mocap, IndiflightHIL, IndiflightSITLWrapper, visApp, visData
from PyNDIflight.sim import Sim

SUPPORTED_BAUDS = [
#57600, 115200, # unlikely to work 
500000, 576000, # maybe works
921600, # shown to work
#1000000, 1500000, 2000000, 3000000, # theoretically possible, untested
]

if __name__=="__main__":
    parser = ArgumentParser(formatter_class=ArgumentDefaultsHelpFormatter)
    # required arguments for docker to work
    parser.add_argument("--sil", required=False, type=str, metavar="LIBRARY", default=None, help="Load INDIflight interface as shared library.")
    parser.add_argument("--sil-profile-txt", required=False, type=str, metavar="PROFILE.txt", help="Import these profile settings into the SIL")
    # further arguments
    parser.add_argument("--sil-log", required=False, action="store_true", help="Write Indiflight logs into ./logs")
    parser.add_argument("--hil", required=False, type=str, metavar="DEVICE", help="Use INDIflight hardware interface. Use either --hil or --sil")
    parser.add_argument("--hil-baud", required=False, choices=SUPPORTED_BAUDS, type=int, default=921600, help="HIL baudrate ")
    parser.add_argument("--mocap", required=False, nargs=2, metavar=("IP", "PORT"), help="Stream mocap UDP packets to this IP/hostname and port")
    parser.add_argument("--no-vis", required=False, action="store_true", help="Do not launch visualization webserver")
    parser.add_argument("--no-real-time", required=False, action="store_true", help="Run as fast as possible")
    parser.add_argument("--catapult", required=False, action="store_true", help="Use catapult (sil-only)")
    parser.add_argument("--throw", required=False, action="store_true", help="Use throwing")
    parser.add_argument("--learn", required=False, action="store_true", help="Learn after throw/catapult (sil-only)")
    args = parser.parse_args()

    if args.sil and args.hil is not None:
        raise ArgumentTypeError("Cannot have both --hil and --sil")

    if args.hil and args.no_real_time:
        raise ArgumentTypeError("--hil must not be used with --no-real-time")

    if not args.sil and args.catapult:
        raise ArgumentTypeError("--catapult only works for SIL")

    if not args.sil and args.learn:
        raise ArgumentTypeError("--learn only works for SIL")

    if args.catapult and args.throw:
        raise ArgumentTypeError("--catapult and --throw cannot be selected at the same time")

    if args.learn and not args.catapult and not args.throw:
        raise ArgumentTypeError("--learn requires either --catapult or --throw")

    if args.sil:
        if not os.path.isfile(args.sil):
            raise FileNotFoundError("Library not found. Likely you didnt compile it yet. In external/indiflight, run `make TARGET=MOCKUP so`")

        import os
        sys.path.append(os.path.join((os.path.dirname(args.sil)),
                                     "../../src/utils"))
        from indiflight_mockup_interface import flightModeFlags, boxId

    if args.mocap is not None:
        args.mocap_host = args.mocap[0]
        try:
            args.mocap_port = int(args.mocap[1])
        except ValueError:
            raise ArgumentTypeError ("PORT must be integer")


    #%% Generate craft
    mc = MultiRotor()
    # NOTE: number of rotors should match the settings learner_num_act in IndiflightProfile.txt
    # NOTE: max 6 actuators
    mc.setInertia(m=0.41, I=0.75*np.diag([0.75e-3, 0.8e-3, 0.9e-3]))
    mc.addRotor(Rotor(r=[-0.05, +0.05, -0.05], Tmax=4.5, dir='lh', axis=[0.,  0., -1.]))
    mc.addRotor(Rotor(r=[+0.05, +0.05, -0.05], Tmax=4.5, dir='rh', axis=[0.,  0., -1.]))
    mc.addRotor(Rotor(r=[-0.05, -0.05, +0.05], Tmax=4.5, dir='lh', axis=[0., -1.,  0.]))
    mc.addRotor(Rotor(r=[+0.05, -0.05, +0.05], Tmax=4.5, dir='rh', axis=[0., -1.,  0.]))
    mc.addRotor(Rotor(r=[-0.05, -0.05, -0.05], Tmax=4.5, dir='rh', axis=[0., -1., -1.]))
    mc.addRotor(Rotor(r=[+0.05, -0.05, -0.05], Tmax=4.5, dir='lh', axis=[0., -1., -1.]))

    # check if even able to hover
    hoverable = True
    G1, _, _ = mc.calculateG1G2()
    Qr, Rr = np.linalg.qr(G1[3:, :].T, 'complete')
    if (len(mc.rotors) < 4) or (np.abs(np.diag(Rr)) < 1e-3).any():
        print(f"\nWARNING: generated craft has no control over some rotation axis or axes.")
        hoverable = False
    else:
        Nr = Qr[:, 3:] # rotational nullspace
        A = Nr.T @ G1[:3, :].T @ G1[:3, :] @ Nr
        v, V = np.linalg.eig(A)
        # calculate most effeicient hover allocation with 1.1 thrust to weight margin
        ustar = ( 1.1 * 9.81 / np.sqrt(max(v)) ) * (Nr @ V[:, np.argmax(v)])
        if not ( ((ustar >= 0.) & (ustar <= 1.)).all() or ((ustar >= 1.) & (ustar <= 0.)).all()):
            print(f"\nWARNING: generated craft does not have enough thrust-to-weight to hover without rotation. Double check rotation directions")
            hoverable = False

    if not hoverable:
        input("Press Enter to continue anyway")


    #%% craft interfaces
    #imu = IMU(mc, r=[0., 0., 0.], qBody=[0., 0., 0., 1.], accStd=0., gyroStd=0.)
    imu = IMU(mc, r=[-0.01, -0.012, 0.008], qBody=[0., 0., 0., 1.], accStd=0., gyroStd=0.)
    #imu = IMU(mc, r=[-0.01, -0.012, 0.008], qBody=[0., 0., 0., 1.], accStd=0.8, gyroStd=0.08)

    mocap = Mocap(mc, args.mocap_host, args.mocap_port) if args.mocap else None
    hil = IndiflightHIL(mc, imu, device=args.hil, baud=args.hil_baud) if args.hil else None
    sil = IndiflightSITLWrapper(mc, imu, args.sil, N=len(mc.rotors)) if args.sil else None


    #%% indiflight configuration, if software in the loop
    if sil is not None:
        sil.mockup.load_profile( args.sil_profile_txt ) if args.sil_profile_txt else None
        sil.mockup.setLogging( args.sil_log )
        if args.sil_log:
            os.makedirs('./logs', exist_ok=True)

        sil.sendMocap()
        sil.mockup.sendPositionSetpoint( [0., 0., -1.5], 0. )
        sil.mockup.enableFlightMode(flightModeFlags.ANGLE_MODE | flightModeFlags.POSITION_MODE)

        if args.learn:
            sil.mockup.enableFlightMode(flightModeFlags.LEARNER_MODE)

        if args.catapult:
            sil.mockup.enableFlightMode(flightModeFlags.CATAPULT_MODE)
        elif args.throw:
            sil.mockup.enableRxBox(boxId.BOXTHROWTOARM)
            sil.mockup.enableRxBox(boxId.BOXARM)


    #%% initial conditions
    mc.setPose(x=[0., 0., -0.1], q=[1., 0., 0., 0.])
    mc.setTwist(v=[0., 0., 0.], w=[0., 0., 0.])

    sim = Sim(mc, imu, mocap, hil, sil)

    if not args.no_vis:
        print("\n\n##########################################\n")
        print(f"Welcome to the sim -- Starting visualization at http://localhost:5000")
        visThread = threading.Thread(target=visApp.run, daemon=True, kwargs={'host':'0.0.0.0'})
        visThread.start( )

    if args.throw:
        mc.throw(height=4.,
                 wB=[2., -4., 3.], # approx body rotation in rad/s
                 vHorz=[1., -2.], # final speed in x-y-plane in m/s
                 at_time=1.)


    #%% run loop
    dt = 0.0005 # 2kHz
    T = 1000. # seconds
    dt_rt = None if args.no_real_time else dt
    start_trajectory = False
    heading = False
    for i in tqdm(range(int(T / dt)), target_looptime=dt_rt):
        if not args.throw and sim.t > 1.:
            sil.mockup.arm() if sil else None

        if not start_trajectory and sim.t > 10. and sil is not None:
            # start trajectory tracking at 8*0.5 = 4m/s target speed
            sil.mockup.sendKeyboard('1')
            if sim.t > 15.:
                for _ in range(8):
                    sil.mockup.sendKeyboard('3')
                start_trajectory = True

        if not heading and sim.t > 20. and sil is not None:
            heading = True
            sil.mockup.sendKeyboard('h')

        sim.tick(dt)
