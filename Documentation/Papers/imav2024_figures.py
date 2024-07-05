#!/usr/bin/env python3

from logTools import IndiflightLog, Signal, imuOffsetCorrection
from glob import glob
from os import path
import pandas as pd
import numpy as np
import pickle

import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

plt.rcParams.update({
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "legend.loc": 'upper right',
    "legend.fontsize": 9,
    "legend.columnspacing": 1.5,
    'figure.subplot.bottom': 0.083,
    'figure.subplot.left': 0.076,
    'figure.subplot.right': 0.979,
    'figure.subplot.top': 0.96,
    'figure.subplot.wspace': 0.4,
    'figure.subplot.hspace': 0.3,
    'figure.titlesize': 'large',
    'lines.linewidth': 1,
    "axes.formatter.limits": [-2, 3]
})

from cycler import cycler
backupCycle = plt.rcParams['axes.prop_cycle']
plt.rcParams['axes.prop_cycle'] = cycler('linestyle', ['-', '--', ':', '-.'])

log = IndiflightLog("/mnt/data/WorkData/BlackboxLogs/2024-06-12/LOG00472_video.BFL", (1368, 1820))
log.resetTime()
crop = log.data
timeMs = log.data['timeMs']


import quaternion
qTrue = np.quaternion(0.837, -0.491, 0.242, 0.).normalized()
r = 1e-3 * np.array([-10., -12., 8.]) # onbaord precision is 1mm
rq = np.quaternion(0, r[0], r[1], r[2])

rIMU = (qTrue * rq * qTrue.inverse()).vec


omegaRaw = Signal(crop['timeS'], crop[[f'omegaUnfiltered[{i}]' for i in range(4)]])
gyroRaw = Signal(crop['timeS'], crop[[f'gyroADCafterRpm[{i}]' for i in range(3)]])
spfRaw = Signal(crop['timeS'], crop[[f'accADCafterRpm[{i}]' for i in range(3)]])
#spfRawFilt = spfRaw.filter('lowpass', 2, 20.)
spfRawFilt = Signal(crop['timeS'], crop[[f'accSmooth[{i}]' for i in range(3)]])
spfRawCor = Signal(crop['timeS'], imuOffsetCorrection(spfRaw.y.copy(), gyroRaw.y, gyroRaw.dot().y, rIMU))
spfRawCorFilt = spfRawCor.filter('lowpass', 2, 20.)

f = plt.figure(figsize=(8.0, 7.0))
gs = GridSpec(4, 2, figure=f)
axs = []
axs.append(f.add_subplot(gs[:2, 0]))
axs.append(f.add_subplot(gs[2:, 0]))
axs.append(f.add_subplot(gs[0, 1]))
axs.append(f.add_subplot(gs[1, 1]))
axs.append(f.add_subplot(gs[2, 1]))
axs.append(f.add_subplot(gs[3, 1]))
axs02 = axs[0].twinx()
axs02.grid(False)
for motor in range(4):
    axs[0].plot(timeMs,
        crop[f'motor[{motor}]'],
        linewidth=1.5,
        label=f'Motor {motor+1}')
    axs02.plot(timeMs,
               omegaRaw.y[:, motor],
               linewidth=0.75)

axs[0].set_ylabel("Input $u$ [-]")
axs02.set_ylabel("Speed $\omega$ [rad/s]")
axs02.set_ylim(bottom=0., top=5e3)
axs[0].set_ylim(bottom=0., top=1.)
axs[0].legend(ncol=2, loc="upper right")

#axs[1].plot(timeMs, spfRawFilt.y, linestyle="--")
AXES = ['x', 'y', 'z']
axs[1].plot(timeMs, spfRawCor.y)#, linestyle="--")
#axs[1].plot(timeMs, spfRawCorFilt.y)#, linestyle="-") # aliasing!
#axs[1].plot(timeMs, spfRawFilt.y)#, linestyle="--")
axs[1].set_ylabel("Specific Force $f$")
#axs[1].legend(['x', 'y', 'z', 'x Filt', 'y Filt', 'z Filt'], ncol=2, loc="upper left")
axs[1].legend(['x', 'y', 'z'], ncol=3, loc="upper left")
axs[1].set_xlabel("Time [ms]")


for plotid, axis in enumerate(AXES):
    axs[2+plotid].plot(timeMs, 
                       crop[[f'fx_{axis}_rls_x[{motor}]' for motor in range(4)]])
    axs[2+plotid].set_ylim(bottom=-2e-6, top=2e-6)
    axs[2+plotid].set_ylabel(f"$G_{{1,{axis}}}$")
#axs[2].legend([f"Motor {i}" for i in range(1,5)], ncol=2, loc="upper left")

axs[5].plot(timeMs,
            crop[[f'hoverAttitude[{i}]' for i in range(4)]],
            linewidth=1.5)
axs[5].plot(timeMs,
            (np.ones_like(timeMs) * np.array([qTrue.w, qTrue.x, qTrue.y, qTrue.z])[:, np.newaxis]).T,
            linewidth=1.)
axs[5].set_ylabel("Thrust Frame $q^T_U$")
axs[5].set_ylim(bottom=-1.1, top=+1.1)
axs[5].legend([
    "$q_w$",
    "$q_x$",
    "$q_y$",
    "$q_z$"
    ], ncol=4, loc="lower left")


axs[-1].set_xlabel("Time [ms]")

f.savefig("hoverAttitude.pdf", format="pdf")
#plt.show()


#%% execution time plot

plt.rcParams.update({
    "legend.fontsize": 10,
    'figure.subplot.bottom': 0.20,
    'figure.subplot.left': 0.12,
    'figure.subplot.right': 0.97,
    'figure.subplot.top': 0.96,
    'figure.subplot.hspace': 0.3,
    'figure.subplot.wspace': 0.35,
    'figure.titlesize': 'large',
    'lines.linewidth': 1,
    "axes.formatter.limits": [-2, 4]
})

log = IndiflightLog("/mnt/data/WorkData/BlackboxLogs/2024-06-12/LOG00472_video.BFL", (1100, 1900))
log.resetTime()
crop = log.data
timeMs = log.data['timeMs'] + 1100 - 1368

plt.rcParams['axes.prop_cycle'] = backupCycle

fex, ax = plt.subplots(1,1, sharex=True, figsize=(6, 2.7))
timings = crop[[f"learnerTimings[{i}]" for i in range(7)]].to_numpy()
timings[:, 1:] = np.diff(timings, axis=1)
relevantTimings = [0, 2, 3, 5, 6]
legendNames = [
    "Filters",
    #"IMU location",
    "Effectiveness",
    "Motors",
    #"Gain Tuning",
    "Updating Values",
    "Thrust Axis"
]


handles = ax.stackplot( 
    timeMs,
    timings[:, relevantTimings].T,
    )

#hatches = ['//', '\\\\', '||', '--', '++', 'xx', 'oo', 'OO', '..', '**']
hatches = ['//', '\\\\', '..', '', 'oo', 'xx', '--', 'OO', '++', '||']
colors = [
    '#333333',
    '#666666',
    '#999999',
    '#cccccc',
    '#ffffff',
    ]
for i, handle in enumerate(handles):
    handle.set_hatch(hatches[i])
    handle.set_facecolor(colors[i])
    handle.set_edgecolor("black")
    handle.set_linewidth(0.5)
ax.set_ylabel( "Execution Time [$\mu$s]" )
ax.set_xlabel( "Time [ms]" )
ax.legend( handles[::-1], legendNames[::-1], loc="upper left")

fex.savefig("learnerTimings.pdf", format='pdf')
fex.savefig("learnerTimings.png", format='png', dpi=300)
fex.savefig("learnerTimings.eps", format='eps')

