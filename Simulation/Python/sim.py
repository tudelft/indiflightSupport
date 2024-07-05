#!/usr/bin/env python3

import numpy as np
from tqdm import tqdm
from time import sleep
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
import threading

from crafts import MultiRotor, Rotor, IMU
from interfaces import Mocap, IndiflightHIL, visApp, visData

def runSim(args):
    mc = MultiRotor()
    mc.setInertia(m=0.45, I=np.diag([0.75e-3, 0.8e-3, 0.9e-3]))
    mc.addRotor(Rotor(r=[-0.05, +0.0635, 0.0], Tmax=5., kESC=0.5, tau=0.02, Izz=5e-7, dir='lh')) # RR
    mc.addRotor(Rotor(r=[+0.05, +0.0635, 0.0], Tmax=5., kESC=0.5, tau=0.02, Izz=5e-7, dir='rh')) # FR
    mc.addRotor(Rotor(r=[-0.05, -0.0635, 0.0], Tmax=5., kESC=0.5, tau=0.02, Izz=5e-7, dir='rh')) # RL
    mc.addRotor(Rotor(r=[+0.05, -0.0635, 0.0], Tmax=5., kESC=0.5, tau=0.02, Izz=5e-7, dir='lh')) # FL

    #imu = IMU(mc, r=[-0.01, -0.012, 0.008], qBody=[0., 0., 0., 1.], accStd=0., gyroStd=0.)
    #imu = IMU(mc, r=[0., 0., 0.], qBody=[0., 0., 0., 1.], accStd=0.4, gyroStd=0.04)
    #imu = IMU(mc, r=[-0.01, -0.012, 0.008], qBody=[0., 0., 0., 1.], accStd=0., gyroStd=0.)
    imu = IMU(mc, r=[-0.01, -0.012, 0.008], qBody=[0., 0., 0., 1.], accStd=0.8, gyroStd=0.08)
    #imu = IMU(mc, r=[-0.014, -0.008, 0.01], qBody=[0., 0., 0., 1.], accStd=0.8, gyroStd=0.08)
    #imu = IMU(mc, r=[-0.03, -0.02, 0.045], qBody=[0., 0., 0., 1.], accStd=0.8, gyroStd=0.08)

    mocap = Mocap(mc, args.mocap_host, args.mocap_port) if args.mocap else None
    hil = IndiflightHIL(mc, imu, device=args.hil, baud=args.hil_baud) if args.hil else None

    # initial conditions
    mc.setPose(x=[0., 0., -0.1], q=[1., 0., 0., 0.])
    mc.setTwist(v=[1., 1., -5.], w=[2., 1., -0.1])

    # run loop
    dt = 0.0005 # 2kHz
    for i in tqdm(range(int(1e8)), target_looptime=dt):
    #for i in tqdm(range(int(1e8))):
        if not (i % 2):
            if hil is not None:
                hil.receive()

        mc.step(dt)

        if not (i % 2): # 1kHz
            imu.update()
            if hil is not None:
                hil.send()
        if not (i % 20): # 100 Hz
            visData.update(mc)
        if not (i % 100): # 20 Hz
            if mocap is not None:
                mocap.update()

SUPPORTED_BAUDS = [
#57600, 115200, # unlikely to work 
500000, 576000, # maybe works
921600, # shown to work
#1000000, 1500000, 2000000, 3000000, # theoretically possible, untested
]

if __name__=="__main__":
    parser = ArgumentParser(formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument("--hil", required=False, type=str, metavar="DEVICE", help="INDIflight hardware interface")
    parser.add_argument("--hil-baud", required=False, choices=SUPPORTED_BAUDS, type=int, default=921600, help="HIL baudrate ")
    parser.add_argument("--mocap", required=False, nargs=2, metavar=("IP", "PORT"), help="Stream mocap UDP packets to this IP/hostname and port")
    args = parser.parse_args()

    if args.mocap is not None:
        args.mocap_host = args.mocap[0]
        try:
            args.mocap_port = int(args.mocap[1])
        except ValueError:
            print("PORT must be integer")

    simThread = threading.Thread(target=runSim, daemon=True, args=[args])
    simThread.start( )
    visApp.run( )
