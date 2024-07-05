#!/usr/bin/env python3
from argparse import ArgumentParser
import logging
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import matplotlib as mpl

from logTools import IndiflightLog, RLS, Signal
# ipython -i estimationPipeline/newImuLS.py -- /mnt/data/WorkData/BlackboxLogs/2024-06-10/IMU_throws/LOG00403_pitchRollYaw.BFL -r 4600 4900 -r 8300 8500 -r 11300 11400

if __name__=="__main__":
    logging.basicConfig(
        format='%(asctime)s -- %(name)s %(levelname)s: %(message)s',
        level=logging.INFO,
        )

    parser = ArgumentParser()
    parser.add_argument("datafile", help="single indiflight bfl logfile")
    parser.add_argument("--range","-r", required=False, nargs=2, action='append', type=float, metavar=('START', 'END'), help="time range to consider in ms since start of datafile")
    args = parser.parse_args()

    ranges = [tuple(r) for r in args.range] if args.range else []


    #%% symbolic math
    from sympy import Matrix, symbols
    wx, wy, wz = symbols("wx wy wz")
    dwx, dwy, dwz = symbols("dwx dwy dwz")
    ax, ay, az = symbols("ax ay az")
    rx, ry, rz = symbols("rx ry rz")
    w = Matrix([[wx,wy,wz]])
    dw = Matrix([[dwx,dwy,dwz]])
    a = Matrix([[ax,ay,az]])
    r = Matrix([[rx,ry,rz]])

    print("\n\n")
    Asym = a + dw.cross(r) + w.cross( w.cross(r) )
    print("aMeasured = ", a, "+",  Asym.jacobian(r), "*", r)
    print("\n\n")

    #%% data filtting
    # fitting parameters
    fc = 50. # Hz. tau = 1/(2*pi*fc) if first order
    order = 2 # 1 --> simple first order. 2 and up --> butterworth

    gyroFilt = np.empty((0, 3))
    dgyroFilt = np.empty((0, 3))
    accFilt = np.empty((0, 3))
    logFull = None

    Arows = []
    yrows = []
    xhat = []

    for j, r in enumerate(ranges):
        if logFull is None:
            logFull = log = IndiflightLog(args.datafile, r)
            log.resetTime()
        else:
            log = IndiflightLog(args.datafile, r)
            log.resetTime()
            log.data['timeS'] += logFull.data['timeS'].iloc[-1] + 0.002
            log.data['timeMs'] += logFull.data['timeMs'].iloc[-1] + 2
            log.data['timeUs'] += logFull.data['timeUs'].iloc[-1] + 2000
            logFull.data = pd.concat( (logFull.data, log.data), ignore_index=True )

        gyro = Signal(log.data["timeS"], log.data[[f"gyroADCafterRpm[{i}]" for i in range(3)]] )
        acc  = Signal(log.data["timeS"], log.data[[f"accADCafterRpm[{i}]" for i in range(3)]] )

        gyroFiltSingle = gyro.filter('lowpass', order, fc).y
        dgyroFiltSingle = gyro.filter('lowpass', order, fc).dot().y
        accFiltSingle = acc.filter('lowpass', order, fc).y

        gyroFilt = np.concatenate( (gyroFilt, gyroFiltSingle) )
        dgyroFilt = np.concatenate( (dgyroFilt, dgyroFiltSingle) )
        accFilt = np.concatenate( (accFilt, accFiltSingle) )

        if j == 1:
            A = np.empty((gyroFiltSingle.shape[0], 3, 3))
            for i, (w, dw, a) in enumerate(zip(gyroFiltSingle, dgyroFiltSingle, accFiltSingle)):
                wx, wy, wz = w
                dwx, dwy, dwz = dw
                A[i] = np.array([
                    [-(wy*wy + wz*wz),    wx*wy - dwz   ,    wx*wz + dwy   ],
                    [  wx*wy + dwz   ,  -(wx*wx + wz*wz),    wy*wz - dwx   ],
                    [  wx*wz - dwy   ,    wy*wz + dwx   ,  -(wx*wx + wy*wy)],
                    ])

            # stack regressors and observations
            Arows.append( A.reshape(-1, 3) )
            yrows.append( accFiltSingle.reshape(-1) )

            # solve
            xh, residuals, rank, s = np.linalg.lstsq(Arows[-1], yrows[-1], rcond=None)
            xhat.append(xh)

        A = np.empty((gyroFilt.shape[0], 3, 3))
        for i, (w, dw, a) in enumerate(zip(gyroFilt, dgyroFilt, accFilt)):
            wx, wy, wz = w
            dwx, dwy, dwz = dw
            A[i] = np.array([
                [-(wy*wy + wz*wz),    wx*wy - dwz   ,    wx*wz + dwy   ],
                [  wx*wy + dwz   ,  -(wx*wx + wz*wz),    wy*wz - dwx   ],
                [  wx*wz - dwy   ,    wy*wz + dwx   ,  -(wx*wx + wy*wy)],
                ])

        # stack regressors and observations
        Arows.append( A.reshape(-1, 3) )
        yrows.append( accFilt.reshape(-1) )

        # solve
        xh, residuals, rank, s = np.linalg.lstsq(Arows[-1], yrows[-1], rcond=None)
        xhat.append(xh)

    logFull.resetTime()

    # statitics
    # https://learnche.org/pid/least-squares-modelling/multiple-linear-regression
    from scipy.stats import chi2
    from scipy.stats import t

    DOF = 3
    q = 0.95
    crit = chi2.ppf(q, DOF)
    #crit = t.ppf(q, DOF)

    # Plot the ellipsoid --> chatGPT
    import cycler
    #plt.rcParams['axes.prop_cycle'] = cycler.cycler(
    #    color=[
    #        '#333333',
    #        '#777777',
    #        '#cccccc',
    #        ],
    #    )
    #mpl.rcParams['hatch.linewidth'] = 0.1

    fig = plt.figure(figsize=(6,5))
    fig.subplots_adjust(left=0., bottom=0., right=0.92, top=1.)

    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X [mm]')
    ax.set_ylabel('Y [mm]')
    ax.set_zlabel('Z [mm]')
    xmax = -np.inf; xmin = np.inf;
    ymax = -np.inf; ymin = np.inf;
    zmax = -np.inf; zmin = np.inf;
    handles = []
    for k, (xh, ys, A) in enumerate(zip(xhat, yrows, Arows)):
        N = int(len(ys)/3)
        residuals = ys - A @ xh
        SE2 = (residuals @ residuals) / (N - DOF)
        Sigma = np.linalg.inv(A.T @ A) * SE2

        # unit ball --> chatGPT
        num_points = 50
        u = np.linspace(0, 2 * np.pi, num_points)
        v = np.linspace(0, np.pi, num_points)
        x = 1e3*np.outer(np.cos(u), np.sin(v))
        y = 1e3*np.outer(np.sin(u), np.sin(v))
        z = 1e3*np.outer(np.ones(np.size(u)), np.cos(v))

        eigvals, eigvecs = np.linalg.eigh(Sigma)
        radii = np.sqrt(eigvals * crit)
        # strech unit ball --> chatGPT
        for i in range(len(x)):
            for j in range(len(x)):
                [x[i, j], y[i, j], z[i, j]] = (radii*eigvecs) @ np.array([x[i, j], y[i, j], z[i, j]])
        x += 1e3*xh[0]
        y += 1e3*xh[1]
        z += 1e3*xh[2]
        xmax = max(xmax, x.max()); xmin = min(xmin, x.min())
        ymax = max(ymax, y.max()); ymin = min(ymin, y.min())
        zmax = max(zmax, z.max()); zmin = min(zmin, z.min())

        handle = ax.plot_surface(x, y, z, alpha=0.6, edgecolor='blue', linewidth=0.0, label=f'Throw {"+".join([str(x+1) for x in range(k+1)])}')
        handles.append(handle)


    # Plot the estimated parameters
    #ax.scatter(1e3*xhat[-1][0], 1e3*xhat[-1][1], 1e3*xhat[-1][2], color='r', s=100)

    max_range = np.array([xmax-xmin, ymax - ymin, zmax - zmin]).max() / 2.0
    mean_x = (xmax + xmin) / 2.0
    mean_y = (ymax + ymin) / 2.0
    mean_z = (zmax + zmin) / 2.0

    ax.set_xlim(mean_x - max_range, mean_x + max_range)
    ax.set_ylim(mean_y - max_range, mean_y + max_range)
    ax.set_zlim(mean_z - 0.2*max_range, mean_z + 0.2*max_range)
    #ax.auto_scale_xyz([0, 500], [0, 500], [0, 0.15])
    ax.set_box_aspect(aspect=(1, 1, 0.2))

    ax.legend([handles[0], handles[1], handles[2], handles[3] ], [
        "Throw $1$",
        "Throw $2$",
        "Throw $1 \cup 2$",
        "Throw $1 \cup 2 \cup 3$",
    ],
        fontsize=12)
    ax.view_init(elev=21, azim=-18, roll=0)

    fig.savefig("95pEllipsoidsIMU.pdf", format='pdf')

    #plt.show()

    print()
    print(f"x [mm]: {(xhat[-1]*1e3).round(2)}")

    f, axs = plt.subplots(4, 1, figsize=(8, 7), sharex=True)
    f.subplots_adjust(left=0.098, bottom=0.074, right=0.96, top=0.974)
    timeMs = logFull.data['timeMs']

    axs[0].plot( timeMs, gyroFilt )
    axs[0].set_ylabel("Gyro [rad/s]")
    axs[0].legend(['x','y','z'],fontsize=12)

    axs[1].plot( timeMs, dgyroFilt )
    axs[1].set_ylabel("Gyro Derivative [rad/s/s]")
    #axs[1].legend(fontsize=10)

    axs[2].plot( timeMs, accFilt )
    axs[2].set_ylabel("Spec force [N/kg]")
    axs[2].set_ylim(bottom=-4.0, top=+4.0)
    #axs[1].legend(fontsize=10)

    axs[3].plot( timeMs, accFilt - Arows[-1].reshape(-1, 3, 3) @ xhat[-1] )
    axs[3].set_ylim(bottom=-0.4, top=+0.4)
    axs[3].set_ylabel("Corrected spec force [N/kg]")
    axs[3].set_xlabel("Time [ms]")
    #axs[3].legend(fontsize=10)

    f.savefig("accCorrection.pdf", format='pdf')


