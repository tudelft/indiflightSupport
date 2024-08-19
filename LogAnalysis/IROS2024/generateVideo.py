#!/usr/bin/env python3

import sys
from os import path, makedirs
absPath = path.dirname(__file__)
sys.path.append(path.join(absPath, '..'))
sys.path.append(path.join(absPath, '..', '..', 'Simulation'))

outputPath = path.join(absPath, "figures")
makedirs(outputPath, exist_ok=True)

from indiflightLogTools import IndiflightLog, Signal, imuOffsetCorrection

import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib import animation
from cycler import cycler

plt.rcParams.update({
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "legend.loc": 'upper right',
    "legend.fontsize": 9,
    "legend.columnspacing": 2.0,
    'figure.subplot.bottom': 0.075,
    'figure.subplot.left': 0.075,
    'figure.subplot.right': 0.95,
    'figure.subplot.top': 0.95,
    'figure.subplot.hspace': 0.450,
    'figure.subplot.wspace': 0.400,
    'figure.titlesize': 'large',
    'lines.linewidth': 1,
    "axes.formatter.limits": [-2, 4]
})

plt.rcParams['axes.prop_cycle'] = cycler('linestyle', ['-', '--', ':', '-.'])

identTime = (1.285, 1.285+0.475)
recoveryTime = (1.285+0.475, 1.285+2.000)
videos = [
    {'time': identTime, 'filename': "identData.mp4", 'slowmo': 30},
    {'time': recoveryTime, 'filename': "recoveryData.mp4", 'slowmo': 5},
]

for video in videos:
    log = IndiflightLog(path.join(absPath, "IROS2024_VideoShootData", "LOG00342.BFL"))
    crop, timeS = log.crop(video['time'][0], video['time'][1])
    timeMs = timeS * 1e3

    order = 2
    fc = 20 # Hz
    r = np.array([-0.01, -0.01, 0.015])

    omegaRaw = Signal(timeS, crop[[f'omegaUnfiltered[{i}]' for i in range(4)]])
    gyroRaw = Signal(timeS, crop[[f'gyroADCafterRpm[{i}]' for i in range(3)]])
    spfRaw = Signal(timeS, crop[[f'accUnfiltered[{i}]' for i in range(3)]])
    spfRawCor = Signal(timeS, imuOffsetCorrection(spfRaw.y.copy(), gyroRaw.y, gyroRaw.dot().y, r))

    fig = plt.figure(figsize=(11, 14))
    gs = fig.add_gridspec(10, 3)

    axFx = []
    g = 0
    while g < 10:
        axFx.append(fig.add_subplot(gs[g, 1]))
        g += 1 if g != 5 else 2

    axG1 = fig.add_subplot(gs[:5, 2])
    axG2 = fig.add_subplot(gs[7:, 2])
    axd = fig.add_subplot(gs[0:1, 0])
    axOmega = fig.add_subplot(gs[1:3, 0])
    axSpGyro = fig.add_subplot(gs[3:5, 0])
    axGyro = fig.add_subplot(gs[5:7, 0])
    axStep = fig.add_subplot(gs[8:, 0])

    tSteps = np.linspace(0, 0.2, 101)
    def stepResp(t, tau):
        return 1 - np.exp(-t / tau) if tau != 0 else np.zeros_like(t)

    axisSymbols = ['x', 'y', 'z', 'p', 'q', 'r']
    axisNames = ['Surge', 'Sway', 'Hieve', 'Roll', 'Pitch', 'Yaw']

    gyroLines = []
    gyroSpLines = []
    fxLines = [[] for _ in range(9)]
    for axis in range(0, 9):
        l = axFx[axis].plot([0], [[0 for _ in range(4)]])
        fxLines[axis].extend(l)
        if axis < 3:
            axFx[axis].set_ylim(bottom=-2e-6, top=2e-6)
            l, = axGyro.plot([0], [0])
            axGyro.set_ylabel("Gyro rate $\Omega$ [$\circ$/s]")
            axGyro.set_ylim(bottom=-1600,top=1600)
            axGyro.set_xlim(left=timeMs[0], right=timeMs[-1])
            axGyro.legend(['Roll', 'Pitch', 'Yaw'], ncols=3, loc="upper left")
            axGyro.set_xlabel("Time [ms]")
            gyroLines.append(l)
            l, = axSpGyro.plot([0], [0])
            axSpGyro.set_ylabel("Rate Setpoint $\Omega_r$ [$\circ$/s]")
            axSpGyro.set_ylim(bottom=-1600,top=1600)
            axSpGyro.set_xlim(left=timeMs[0], right=timeMs[-1])
            #axSpGyro.legend(['Roll', 'Pitch', 'Yaw'], ncols=3, loc="upper left")
            #axSpGyro.set_xlabel("Time [ms]")
            gyroSpLines.append(l)
        elif axis < 6:
            axFx[axis].set_ylim(bottom=-5e-5, top=5e-5)
        else:
            axFx[axis].set_ylim(bottom=-2.5e-3, top=2.5e-3)

        if axis < 6:
            axFx[axis].set_ylabel(axisNames[axis])
        else:
            axFx[axis].set_ylabel(axisNames[axis-3])

        if axis == 0:
            axFx[axis].set_title("$\omega^2$ Effectiveness Values")
        if axis == 5:
            axFx[axis].legend([f"Motor {i}" for i in range(1,5)], bbox_to_anchor=(0.9, -.25), ncol = 2)
        if axis == 6:
            axFx[axis].set_title("$\dot\omega$ Effectiveness Values")

        if (axis != 5) and (axis != 8):
            axFx[axis].set_xticklabels([])

        if axis == 8:
            axFx[axis].set_xlabel("Time [ms]")
        axFx[axis].set_xlim(left=timeMs[0], right=timeMs[-1])


    stepLines = []
    exLines = []
    omegaLines = []
    for motor in range(4):
        l, = axStep.plot(tSteps*1e3, stepResp(tSteps, 0.02))
        axStep.set_xlabel("Time [ms]")
        axStep.set_ylabel("Motor RPM")
        axStep.legend([f"Motor {i+1}" for i in range(4)], loc='lower right')
        axStep.set_title("Step response model and max speed")
        axStep.set_ylim(bottom=0, top=60000.)
        stepLines.append(l)

        l, = axd.plot([0], [0])
        axd.set_ylim(bottom=0, top=100)
        axd.set_xlim(left=timeMs[0], right=timeMs[-1])
        axd.set_ylabel("Motor Input [$\%$]")
        #axd.legend([f"Motor {i}" for i in range(1,5)], ncols=2)
        axd.set_xticklabels([])
        exLines.append(l)

        l, = axOmega.plot([0], [0])
        axOmega.set_ylim(bottom=0, top=60000)
        axOmega.set_xlim(left=timeMs[0], right=timeMs[-1])
        axOmega.set_ylabel("Motor RPM")
        axOmega.set_xticklabels([])
        axOmega.legend([f"Motor {i}" for i in range(1,5)], ncols=2)
        omegaLines.append(l)

    allLines = []
    [allLines.extend(l) for l in fxLines]
    allLines.extend(gyroLines)
    allLines.extend(stepLines)
    allLines.extend(exLines)
    allLines.extend(omegaLines)

    G1im = axG1.imshow(np.zeros((6, 4)))
    G2im = axG2.imshow(np.zeros((3, 4)))
    axG1.set_xticks(range(4), labels=[f"Motor {i}" for i in range(1,5)])
    axG1.set_yticks(range(6), labels=axisNames)
    axG1.set_title("$\omega^2$ Effectiveness Matrix")
    axG2.set_xticks(range(4), labels=[f"Motor {i}" for i in range(1,5)])
    axG2.set_title("$\dot\omega$ Effectiveness Matrix")
    axG2.set_yticks(range(3), labels=axisNames[3:])
    allIms = [G1im, G2im]

    def init():
        for line in allLines:
            line.set_data([], [])

        return allLines+allIms

    def func(frame):
        allLines = []

        for axis, fx in enumerate(fxLines):
            for motor, line in enumerate(fx):
                x = list(line.get_xdata())
                y = list(line.get_ydata())
                x.append(frame['timeMs'])
                if axis < 6:
                    y.append(frame['G1'][axis, motor])
                else:
                    y.append(frame['G2'][axis-6, motor])
                line.set_data(x, y)
                allLines.append(line)

        for axis, line in enumerate(gyroLines):
            x = list(line.get_xdata())
            y = list(line.get_ydata())
            x.append(frame['timeMs'])
            y.append(frame['gyro'][axis] * 180/np.pi)
            line.set_data(x, y)
            allLines.append(line)

        for axis, line in enumerate(gyroSpLines):
            x = list(line.get_xdata())
            y = list(line.get_ydata())
            x.append(frame['timeMs'])
            y.append(frame['gyroSp'][axis] * 180/np.pi)
            line.set_data(x, y)
            allLines.append(line)

        for motor, line in enumerate(exLines):
            x = list(line.get_xdata())
            y = list(line.get_ydata())
            x.append(frame['timeMs'])
            y.append(frame['d'][motor]*100)
            line.set_data(x, y)
            allLines.append(line)

        for motor, line in enumerate(omegaLines):
            x = list(line.get_xdata())
            y = list(line.get_ydata())
            x.append(frame['timeMs'])
            y.append(frame['omega'][motor]*60/np.pi/2.)
            line.set_data(x, y)
            allLines.append(line)

        for step, tau, wmax, widle in zip(stepLines, frame['tau'], frame['wmax'], frame['widle']):
            step.set_data(tSteps*1e3, 60./np.pi/2*(widle + wmax * stepResp(tSteps, tau)))
            allLines.append(step)

        for im in allIms:
            im.remove()

        cmapname = 'coolwarm'
        limF = 1e-6
        limR = 5e-5
        G1clip = frame['G1']
        G1clip[:3, :] = np.clip(G1clip[:3, :] / limF, -1, 1)
        G1clip[3:, :] = np.clip(G1clip[3:, :] / limR, -1, 1)
        limRd = 2e-3
        G2clip = frame['G2']
        G2clip[:3, :] = np.clip(G2clip[:3, :] / limRd, -1, 1)

        allIms[0] = axG1.imshow(G1clip, cmap=mpl.colormaps[cmapname], vmin=-1, vmax=1)
        allIms[1] = axG2.imshow(G2clip, cmap=mpl.colormaps[cmapname], vmin=-1, vmax=1)

        return allLines+allIms

    from tqdm import tqdm

    def data_gen():
        for index, row in tqdm(crop.iterrows(), 
                               total=len(crop), 
                               desc=f"rendering {video['filename']}"):
            wmax = []
            widle = []
            ks = []
            tau = []
            G1 = np.zeros((6,4))
            G2 = np.zeros((3,4))
            d = []
            omega = []

            for axi, ax in enumerate(axisSymbols):
                theta = row[[f'fx_{ax}_rls_x[{i}]' for i in range(4)]].to_numpy(dtype=float)
                G1[axi, :] = theta

            for axi, ax in enumerate(axisSymbols[3:]):
                theta = row[[f'fx_{ax}_rls_x[{i}]' for i in range(4,8)]].to_numpy(dtype=float)
                G2[axi, :] = theta

            gyro = row[[f'gyroADCafterRpm[{i}]' for i in range(3)]].to_numpy(dtype=float)
            gyroSp = row[[f'gyroSp[{i}]' for i in range(3)]].to_numpy(dtype=float)

            for motor in range(4):
                theta = row[[f'motor_{motor}_rls_x[{i}]' for i in range(4)]].to_numpy(dtype=float)
                wm = theta[0] + theta[1]
                k = theta[0] / wm if wm != 0 else 0
                ks.append(k)

                wmax.append( wm )
                widle.append( theta[2] )
                tau.append( theta[3] )
                d.append( row[f'motor[{motor}]'] )
                omega.append( row[f'omegaUnfiltered[{motor}]'] )

            frame = { 'wmax': wmax, 'widle': widle, 'tau': tau,
                      'd': d, 'omega': omega,
                      'gyro': gyro, 'gyroSp': gyroSp,
                      'timeMs': row['timeMs'], 'G1': G1, 'G2': G2 }
            yield frame

    ani = animation.FuncAnimation(fig, func, data_gen, init_func=init, 
                                  interval=2, blit=True, repeat=False,
                                  save_count=len(crop))
    FFwriter = animation.FFMpegWriter(fps=500/video['slowmo'], bitrate=10000)
    ani.save(path.join(outputPath, video['filename']), writer=FFwriter)
