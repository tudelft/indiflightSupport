#!/usr/bin/env python3

import numpy as np
from tqdm import tqdm
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter, ArgumentTypeError
import threading
import os
import sys

from crafts import MultiRotor, Rotor, IMU
from interfaces import Mocap, IndiflightHIL, IndiflightSITLWrapper, visApp, visData

class Sim():
    def __init__(self, uav, imu=None, mocap=None, hil=None, sil=None):
        self.uav = uav
        self.imu = imu
        self.mocap = mocap
        self.hil = hil
        self.sil = sil
        self.t = 0.
        self.i = 0

    def tick(self, dt):
        if not (self.i % 2):
            self.hil.receive() if self.hil else None

        self.sil.receive() if self.sil else None

        self.uav.step(dt)

        if self.sil:
            self.sil.sendImuAndMotor()
            self.sil.tick(dt) # ticks the indiflight controller

        if not (self.i % 2): # 1kHz
            self.imu.update() if self.imu else None
            self.hil.send() if self.hil else None
        if not (self.i % 20): # 100 Hz
            visData.update(self.uav)
        if not (self.i % 100): # 20 Hz
            self.mocap.update() if self.mocap else None
            self.sil.sendMocap() if self.sil else None

        self.t += dt
        self.i += 1

SUPPORTED_BAUDS = [
#57600, 115200, # unlikely to work 
500000, 576000, # maybe works
921600, # shown to work
#1000000, 1500000, 2000000, 3000000, # theoretically possible, untested
]

if __name__=="__main__":
    parser = ArgumentParser(formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument("--sil-mockup", required=False, type=str, metavar="LIBRARY", default=None, help="Load INDIflight interface as shared library.")
    parser.add_argument("--sil-profile-txt", required=False, type=str, metavar="PROFILE.txt", help="Import these profile settings into the SIL")
    parser.add_argument("--sil-log", required=False, action="store_true", help="Write Indiflight logs into ./logs")
    parser.add_argument("--hil", required=False, type=str, metavar="DEVICE", help="Use INDIflight hardware interface. Use either --hil or --sil-mockup")
    parser.add_argument("--hil-baud", required=False, choices=SUPPORTED_BAUDS, type=int, default=921600, help="HIL baudrate ")
    parser.add_argument("--mocap", required=False, nargs=2, metavar=("IP", "PORT"), help="Stream mocap UDP packets to this IP/hostname and port")
    parser.add_argument("--no-vis", required=False, action="store_true", help="Do not launch visualization webserver")
    parser.add_argument("--no-real-time", required=False, action="store_true", help="Run as fast as possible")
    parser.add_argument("--catapult", required=False, action="store_true", help="Use catapult (sil-only)")
    parser.add_argument("--throw", required=False, action="store_true", help="Use throwing")
    parser.add_argument("--learn", required=False, action="store_true", help="Learn after throw/catapult (sil-only)")
    args = parser.parse_args()

    if args.sil_mockup and args.hil is not None:
        raise ArgumentTypeError("Cannot have both --hil and --sil-mockup")

    if args.hil and args.no_real_time:
        raise ArgumentTypeError("--hil must not be used with --no-real-time")

    if not args.sil_mockup and args.catapult:
        raise ArgumentTypeError("--catapult only works for SIL")

    if not args.sil_mockup and args.learn:
        raise ArgumentTypeError("--learn only works for SIL")

    if args.catapult and args.throw:
        raise ArgumentTypeError("--catapult and --throw cannot be selected at the same time")

    if args.learn and not args.catapult and not args.throw:
        raise ArgumentTypeError("--learn requires either --catapult or --throw")

    if args.sil_mockup:
        if not os.path.isfile(args.sil_mockup):
            raise FileNotFoundError("Library not found. Likely you didnt compile it yet. In external/indiflight, run `make TARGET=MOCKUP so`")

        import os
        sys.path.append(os.path.join((os.path.dirname(args.sil_mockup)),
                                     "../../src/utils"))
        from indiflight_mockup_interface import flightModeFlags, boxId

    if args.mocap is not None:
        args.mocap_host = args.mocap[0]
        try:
            args.mocap_port = int(args.mocap[1])
        except ValueError:
            raise ArgumentTypeError ("PORT must be integer")

    mc = MultiRotor()
    mc.setInertia(m=0.45, I=np.diag([0.75e-3, 0.8e-3, 0.9e-3]))
    mc.addRotor(Rotor(r=[-0.05, +0.0635, 0.0], Tmax=5., kESC=0.5, tau=0.02, Izz=5e-7, dir='lh')) # RR
    mc.addRotor(Rotor(r=[+0.05, +0.0635, 0.0], Tmax=5., kESC=0.5, tau=0.02, Izz=5e-7, dir='rh')) # FR
    mc.addRotor(Rotor(r=[-0.05, -0.0635, 0.0], Tmax=5., kESC=0.5, tau=0.02, Izz=5e-7, dir='rh')) # RL
    mc.addRotor(Rotor(r=[+0.05, -0.0635, 0.0], Tmax=5., kESC=0.5, tau=0.02, Izz=5e-7, dir='lh')) # FL

    #imu = IMU(mc, r=[-0.01, -0.012, 0.008], qBody=[0., 0., 0., 1.], accStd=0., gyroStd=0.)

    #imu = IMU(mc, r=[-0.01, -0.012, 0.008], qBody=[0., 0., 0., 1.], accStd=0., gyroStd=0.)
    #imu = IMU(mc, r=[0., 0., 0.], qBody=[0., 0., 0., 1.], accStd=0.4, gyroStd=0.04)
    #imu = IMU(mc, r=[-0.01, -0.012, 0.008], qBody=[0., 0., 0., 1.], accStd=0., gyroStd=0.)
    imu = IMU(mc, r=[-0.01, -0.012, 0.008], qBody=[0., 0., 0., 1.], accStd=0.8, gyroStd=0.08)
    #imu = IMU(mc, r=[-0.014, -0.008, 0.01], qBody=[0., 0., 0., 1.], accStd=0.8, gyroStd=0.08)
    #imu = IMU(mc, r=[-0.03, -0.02, 0.045], qBody=[0., 0., 0., 1.], accStd=0.8, gyroStd=0.08)

    mocap = Mocap(mc, args.mocap_host, args.mocap_port) if args.mocap else None
    hil = IndiflightHIL(mc, imu, device=args.hil, baud=args.hil_baud) if args.hil else None
    sil = IndiflightSITLWrapper(mc, imu, args.sil_mockup, N=len(mc.rotors)) if args.sil_mockup else None
    if sil is not None:
        sil.mockup.load_profile( args.sil_profile_txt )
        sil.mockup.setLogging( args.sil_log )
        sil.mockup.sendPositionSetpoint( [0., 0., -1.5], 0. )
        sil.mockup.enableFlightMode(flightModeFlags.ANGLE_MODE | flightModeFlags.POSITION_MODE)

        if args.learn:
            sil.mockup.enableFlightMode(flightModeFlags.LEARNER_MODE)

        if args.catapult:
            sil.mockup.enableFlightMode(flightModeFlags.CATAPULT_MODE)
        elif args.throw:
            sil.mockup.enableRxBox(boxId.BOXTHROWTOARM)

    # initial conditions
    mc.setPose(x=[0., 0., -0.1], q=[1., 0., 0., 0.])
    mc.setTwist(v=[0., 0., 0.], w=[0., 0., 0.])

    sim = Sim(mc, imu, mocap, hil, sil)

    if not args.no_vis:
        from netifaces import interfaces, ifaddresses, AF_INET
        for ifaceName in interfaces():
            addresses = [i['addr'] for i in ifaddresses(ifaceName).setdefault(AF_INET, [{'addr':'No IP addr'}] )]

        print(f"Welcome to sim.py -- Starting visualization at http://{addresses[-1]}:5000\n")

        visThread = threading.Thread(target=visApp.run, daemon=True, kwargs={'host':'0.0.0.0'})
        visThread.start( )

    # run loop
    dt = 0.0005 # 1kHz
    dt_rt = None if args.no_real_time else 1*dt
    thrown = False
    for i in tqdm(range(int(1e8)), target_looptime=dt_rt):
        if not thrown and args.throw and sim.t > 0.5:
            mc.throw(wB=[2., -4., 3.], # approx body rotation in rad/s
                     vHorz=[1., -2.], # final speed in x-y-plane in m/s
                     )
            thrown = True

        if not args.throw and sim.t > 0.5:
            sil.mockup.arm() if sil else None

        sim.tick(dt)
