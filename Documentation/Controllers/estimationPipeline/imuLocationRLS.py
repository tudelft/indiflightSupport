#!/usr/bin/env python3
from argparse import ArgumentParser
import logging
import numpy as np

from logTools import IndiflightLog, RLS, Signal

if __name__=="__main__":
    logging.basicConfig(
        format='%(asctime)s -- %(name)s %(levelname)s: %(message)s',
        level=logging.INFO,
        )

    parser = ArgumentParser()
    parser.add_argument("datafile", help="single indiflight bfl logfile")
    parser.add_argument("--range","-r", required=False, nargs=2, default=(2215, 2670), type=float, help="time range to consider in ms since start of datafile")
    args = parser.parse_args()

    # fitting parameters
    fc = 200. # Hz. tau = 1/(2*pi*fc) if first order
    order = 2 # 1 --> simple first order. 2 and up --> butterworth
    gamma = 1e4
    forgetting = 1. # todo: dependent on sampling rate?
    forgetting = 1.

    # load unfiltered data into numpy
    log = IndiflightLog(args.datafile, args.range)
    log.resetTime()

    gyro = Signal(log.data["timeS"], log.data[[f"gyroADCafterRpm[{i}]" for i in range(3)]] )
    acc  = Signal(log.data["timeS"], log.data[[f"accUnfiltered[{i}]" for i in range(3)]] )

    gyroFilt = gyro.filter('lowpass', order, fc)
    accFilt  = acc.filter('lowpass', order, fc)
    gyroFilt = gyro
    accFilt  = acc

    est = RLS(6, 3, gamma=gamma, forgetting=forgetting)
    for i in range(len(log.data)):
        wx, wy, wz    = gyroFilt.y[i]
        dwx, dwy, dwz = gyroFilt.dot().y[i]
        A = np.array([
            [-(wy*wy + wz*wz),    wx*wy - dwz   ,    wx*wz + dwy   , 10., 0., 0.],
            [  wx*wy + dwz   ,  -(wx*wx + wz*wz),    wy*wz - dwx   , 0. , 10., 0.],
            [  wx*wz - dwy   ,    wy*wz + dwx   ,  -(wx*wx + wy*wy), 0. , 0., 10.],
            ])
        # equally weighing all 3 axes
        est.newSample(A, accFilt.y[i])

    est.setName("IMU offset estimator")
    est.setParameterNames(["$d_x$", "$d_y$", "$d_z$", "$f_{0,x}$", "$f_{0,y}$", "$f_{0,z}$"])
    # est.setRegressorNames( NOT SET )
    est.setOutputNames(["$f_x$", "$f_y$", "$f_z$"])

    f = est.plotParameters(
        parGroups=[[0], [1], [2], [3,4,5,]],
        yGroups=[[0], [1], [2]],
        timeMs=log.data['timeMs'],
        sharey=True,
    )
    f.show()
