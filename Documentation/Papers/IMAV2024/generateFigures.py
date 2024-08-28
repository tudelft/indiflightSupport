# Generate all figures used for our IMAV2024 contribution
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
from pyquaternion import Quaternion

#%% plotting setup

import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from cycler import cycler

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

backupCycle = plt.rcParams['axes.prop_cycle']
plt.rcParams['axes.prop_cycle'] = cycler('linestyle', ['-', '--', ':', '-.'])


#%% paths setup
import sys
from os import path, makedirs
absPath = path.dirname(__file__)
repoRootPath = path.join(absPath, '..', '..', '..')
sys.path.append(repoRootPath)

outputPath = path.join(absPath, "figures")
makedirs(outputPath, exist_ok=True)


from argparse import ArgumentParser
parser = ArgumentParser()
parser.add_argument("dataset", type=str, default=None, help="Path to the throw-to-hover dataset")
args = parser.parse_args()


#%% import data

from LogAnalysis.indiflightLogTools import IndiflightLog, Signal, imuOffsetCorrection
log = IndiflightLog(path.join(args.dataset, "IMAV2024_ExperimentData", "LOG00472_throwAndFly.BFL"), (1368, 1820))
throwsFile = path.join(args.dataset, "IMAV2024_ExperimentData", "LOG00403_pitchRollYaw_throwAndCatch.BFL")


#%% analyse data
log.resetTime()
crop = log.data
timeMs = log.data['timeMs']

qTrue = Quaternion(scalar=0.837, vector=[-0.491, 0.242, 0.]).normalised
r = 1e-3 * np.array([-10., -12., 8.]) # onbaord precision is 1mm
rq = Quaternion(scalar=0., vector=r)

rIMU = (qTrue * rq * qTrue.inverse).vector


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

f.savefig(path.join(outputPath, "hoverAttitude.pdf"), format="pdf")
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

#log = IndiflightLog(path.join(absPath, "IMAV2024_ExperimentData", "LOG00472.BFL"), (1368, 1820))
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

fex.savefig(path.join(outputPath, "learnerTimings.pdf"), format='pdf')
fex.savefig(path.join(outputPath, "learnerTimings.png"), format='png', dpi=300)
fex.savefig(path.join(outputPath, "learnerTimings.eps"), format='eps')


#%% Find IMU distance and generate Ellipsoid plots

import pandas as pd
from scipy.stats import chi2

ranges = [(4600, 4900), (8300, 8500), (11300, 11400)]

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
        logFull = log = IndiflightLog(throwsFile, r)
        log.resetTime()
    else:
        log = IndiflightLog(throwsFile, r)
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

DOF = 3
q = 0.95
crit = chi2.ppf(q, DOF)

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

fig.savefig(path.join(outputPath, "95pEllipsoidsIMU.pdf"), format='pdf')

#plt.show()

print()
print(f"x [mm]: {(xhat[-1]*1e3).round(2)}")




#%% acc correction plots 

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

f.savefig(path.join(outputPath, "accCorrection.pdf"), format='pdf')


