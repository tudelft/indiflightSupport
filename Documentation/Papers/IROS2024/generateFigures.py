# Generate all figures used for our IROS2024 contribution
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

from tqdm import tqdm
import pandas as pd
import numpy as np
from glob import glob
import pickle

#%% path setup
import sys
from os import path, makedirs
absPath = path.dirname(__file__)
repoRootPath = path.join(absPath, '..', '..', '..')
sys.path.append(repoRootPath)
sys.path.append(path.join(repoRootPath, 'Simulation'))

outputPath = path.join(absPath, "figures")
makedirs(outputPath, exist_ok=True)

from argparse import ArgumentParser
parser = ArgumentParser()
parser.add_argument("dataset", type=str, default=None, help="Path to the throw-to-hover dataset")
args = parser.parse_args()

experimentDataPath = path.join(args.dataset, "IROS2024_ExperimentData")
videoDataPath      = path.join(args.dataset, "IROS2024_VideoShootData")
simulationDataPath = path.join(args.dataset, "IROS2024_SimulationData")


#%% start

from LogAnalysis.indiflightLogTools import IndiflightLog, imuOffsetCorrection, Signal

# G1/G2
rowNames = ['x', 'y', 'z', 'p', 'q', 'r']

def analyseLogs(logPath, calcInitConds = False):
    searchPath = path.join(logPath, "*.BFL")
    files = sorted(glob(searchPath)) # sorting?!
    if len(files) == 0:
        raise ValueError(f"Path {searchPath} contains no .BFL files")

    #%% go through logs and get G1 and G2 parameters
    logs = []
    pars = []
    for runIdx, file in enumerate(tqdm(files, desc=f"importing logs from {logPath}")):
        log = IndiflightLog(file)
        logs.append(log)
        lastRow = log.data.iloc[-1] 

        parameterRow = {'run': runIdx}

        for i, r in enumerate(rowNames):
            for col in range(4):
                parameterRow[f'G1_{r}_{col}'] = lastRow[f'fx_{r}_rls_x[{col}]']

            if i >= 3:
                omegaDotOffset = 8 if log.num_learner_vars > 8 else 4
                for col in range(omegaDotOffset, omegaDotOffset + (log.num_learner_vars >> 1)):
                    parameterRow[f'G2_{r}_{col-omegaDotOffset}'] = lastRow[f'fx_{r}_rls_x[{col}]']

        # motors
        for motor in range(4):
            a, b, w0, tau = lastRow[[f'motor_{motor}_rls_x[{i}]' for i in range(4)]]
            a = a if a > 0 else 0xFFFF + a # integer overflow in the data... not nice
            #b = b if b > 0 else 0xFFFF + b # integer overflow in the data... not nice
            wm = a+b
            lam = a / wm
            parameterRow[f'motor_{motor}_wm'] = wm
            parameterRow[f'motor_{motor}_k'] = lam
            parameterRow[f'motor_{motor}_w0'] = w0
            parameterRow[f'motor_{motor}_tau'] = tau

        if calcInitConds:
            # initial rotation rate, after 1400 ms
            initCondIdx = (log.data['timeMs'] - 1400).abs().idxmin()
            parameterRow['p0'] = log.data['gyroADC[0]'].loc[initCondIdx] # in rad/s
            parameterRow['q0'] = log.data['gyroADC[1]'].loc[initCondIdx]
            parameterRow['r0'] = log.data['gyroADC[2]'].loc[initCondIdx]

            # get minimum altitude from this point until the manual takeover
            takeoverIdx = log.flags[log.flags['disable'].apply(lambda x: 9 in x)].index[0]
            parameterRow['minH'] = -log.data['extPos[2]'].loc[initCondIdx:takeoverIdx].max()

        pars.append(parameterRow.copy())

    #%% get mean and std as latex table

    df = pd.DataFrame(pars)
    df.set_index('run', inplace=True)

    return df


df = analyseLogs(experimentDataPath, calcInitConds=True)

mean = df.mean()
std = df.std()

G1df = pd.DataFrame([], index=[f"$B_{{1,{r}}}\cdot k \cdot 10^6$" for r in rowNames])
G2df = pd.DataFrame([], index=[f"$B_{{2,{r}}}\cdot 10^3$" for r in rowNames[3:]])
motordf = pd.DataFrame([], index=['$\omega_{\\text{max}}$', '$\kappa$', '$\omega_{\\text{idle}}$', '$\\tau$ [ms]'])

# from genGMc.py
G1_fromData = np.array([
       [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 0.00000000e+00],
       [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 0.00000000e+00],
       [-6.21495327e-07, -6.21495327e-07, -6.21495327e-07, -6.21495327e-07],
       [-2.34401887e-05, -2.34401887e-05,  2.34401887e-05, 2.34401887e-05],
       [-1.57181818e-05,  1.57181818e-05, -1.57181818e-05, 1.57181818e-05],
       [-2.98876404e-06,  2.98876404e-06,  2.98876404e-06, -2.98876404e-06]])

G2_fromData = np.array([
       [ 0.        , -0.        , -0.        ,  0.        ],
       [ 0.        , -0.        , -0.        ,  0.        ],
       [-0.00101124,  0.00101124,  0.00101124, -0.00101124]])

omegaMax = 4113.063728303113 # from prop bench test
k = 0.46 # from prop bench test
omega0 = 449.725817910626 # from prop bench test
tau = 0.02 # from prop bench test

motordf['Benchtest'] = [omegaMax, k, omega0, tau]
for motor in range(4):
    G1df[f'True motor {motor}'] = G1_fromData[:, motor]
    G1df[f'Mean motor {motor}'] = mean.filter(regex=f'^G1_[xyzpqr]_{motor}').to_list()
    G1df[f'Std motor {motor}'] = std.filter(regex=f'^G1_[xyzpqr]_{motor}').to_list()

    G2df[f'True motor {motor}'] = G2_fromData[:, motor]
    G2df[f'Mean motor {motor}'] = mean.filter(regex=f'^G2_[pqr]_{motor}').to_list()
    G2df[f'Std motor {motor}'] = std.filter(regex=f'^G2_[pqr]_{motor}').to_list()

    motordf[f'Mean motor {motor}'] = mean.filter(regex=f'^motor_{motor}').to_list()
    motordf[f'Std motor {motor}'] = std.filter(regex=f'^motor_{motor}').to_list()

with open(path.join(outputPath, "flightExperiment.txt"), "w") as f:
    f.write("Flight experiments: \n\n")
    f.write((1e6                            *G1df).to_latex(float_format="%.3f") + "\n")
    f.write((1e3                            *G2df).to_latex(float_format="%.3f") + "\n")
    f.write((np.array([[1.,1.,1.,1000.]]).T *motordf).to_latex(float_format="%.3f") + "\n")

#%% Initial condition scatter

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 
plt.close('all')

plt.rcParams.update({
#    "text.usetex": True,  # NB: original plots were generated with latex, but for portability, this is turned off here
#    "font.family": "Helvetica",
    "font.family": "sans-serif",
    "font.size": 12,
    "axes.grid": True,
    "axes.grid.which": 'both',
    "grid.linestyle": '--',
    "grid.alpha": 0.7,
    "axes.labelsize": 10,
    "axes.titlesize": 16,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "legend.loc": 'upper right',
    "legend.fontsize": 9,
    "legend.columnspacing": 2.0,
    'figure.subplot.bottom': 0.19,
    'figure.subplot.left': 0.07,
    'figure.subplot.right': 0.96,
    'figure.subplot.top': 0.92,
    'figure.subplot.hspace': 0.3,
    'figure.subplot.wspace': 0.4,
    'figure.titlesize': 'large',
    'lines.linewidth': 1,
})

from cycler import cycler
plt.rcParams['axes.prop_cycle'] = cycler('linestyle', ['-', '--', ':', '-.'])

p0, q0, r0 = df[['p0', 'q0', 'r0']].to_numpy(dtype=float).T * 180./np.pi
minH = df['minH']
fig = plt.figure(figsize=(6.7, 5))
ax = fig.add_subplot(111, projection='3d')
#ax.stem(p0, q0, r0, 2, 2, r0+800, color='black', shade=False)
_, stemlines,_ = ax.stem(p0, q0, r0, basefmt=' ', linefmt='grey', markerfmt=' ', bottom=-700)
stemlines.set_linestyle('--')
scatter = ax.scatter(p0, q0, r0, c=minH, cmap='viridis', alpha=1.)
colorbar = fig.colorbar(scatter, pad=0.1)
colorbar.set_label('Minimum Altitude during Recovery [m]')
ax.set_xlabel("Roll [deg/s]")
ax.set_ylabel("Pitch [deg/s]")
ax.set_zlabel("Yaw [deg/s]")
ax.set_zlim(bottom=-700)
ax.view_init(elev=17, azim=-137)
#plt.title("Initial rotation before excitation")

fig.savefig(path.join(outputPath, 'InitialRotation.pdf'), format='pdf')
#fig.show()


#%% Simulation results

# craft data
searchPath = path.join(simulationDataPath, "*.pickle")
pkls = sorted(glob(searchPath))

# simulation logs
dfSim = analyseLogs(simulationDataPath, calcInitConds=False)

trueParameters = []
for runIdx, pkl in enumerate(tqdm(pkls, desc=f"importing crafts from {searchPath}")):
    with open(pkl, 'rb') as f:
        data = pickle.load(f)

    parameterRow = {'run': runIdx}
    for i, r in enumerate(rowNames):
        for motor in range(4):
            parameterRow[f'G1_{r}_{motor}'] = data['G1'][i, motor] / data['wmax'][motor]**2
        if (i >= 3):
            for motor in range(4):
                parameterRow[f'G2_{r}_{motor}'] = data['G2'][i, motor]

    # motors
    for motor in range(4):
        parameterRow[f'motor_{motor}_wm'] = data['wmax'][motor] 
        parameterRow[f'motor_{motor}_k'] = data['kappas'][motor]
        parameterRow[f'motor_{motor}_w0'] = 0.
        parameterRow[f'motor_{motor}_tau'] = data['taus'][motor]

    trueParameters.append(parameterRow)

dfSimTrue = pd.DataFrame(trueParameters)
dfSimTrue.set_index('run', inplace=True)

dfError = (dfSimTrue - dfSim)

dfOverview = pd.DataFrame({'nominal': dfSimTrue.abs().mean()}).T
dfOverview.loc['errorRMS'] = (dfError**2).mean()**0.5

G1df = pd.DataFrame([], index=[f"$G_{{1,{r}}}$" for r in rowNames])
G2df = pd.DataFrame([], index=[f"$G_{{2,{r}}}$" for r in rowNames[3:]])
motordf = pd.DataFrame([], index=['$\omega_{\\text{max}}$', '$\kappa$', '$\omega_{\\text{idle}}$', '$\\tau$ [ms]'])

G1df['nominal'] = dfOverview.filter(regex=f'^G1_[xyzpqr]_{0}').T['nominal'].to_numpy()
G2df['nominal'] = dfOverview.filter(regex=f'^G2_[pqr]_{0}').T['nominal'].to_numpy()
motordf['nominal'] = dfOverview.filter(regex=f'^motor_{0}').T['nominal'].to_numpy()

for motor in range(4):
    G1df[f'errorRMS {motor}'] = dfOverview.filter(regex=f'^G1_[xyzpqr]_{motor}').T['errorRMS'].to_numpy()
    G2df[f'errorRMS {motor}'] = dfOverview.filter(regex=f'^G2_[pqr]_{motor}').T['errorRMS'].to_numpy()
    motordf[f'errorRMS {motor}'] = dfOverview.filter(regex=f'^motor_{motor}').T['errorRMS'].to_numpy()

with open(path.join(outputPath, "simulationExperiment.txt"), "w") as f:
    f.write("Simulation experiments: \n\n")
    f.write((1e6                            *G1df).to_latex(float_format="%.3f") + "\n")
    f.write((1e3                            *G2df).to_latex(float_format="%.3f") + "\n")
    f.write((np.array([[1.,1.,1.,1000.]]).T *motordf).to_latex(float_format="%.3f") + "\n")

#%% Time plots excitation

plt.rcParams.update({
#    "text.usetex": True,  # NB: original plots were generated with latex, but for portability, this is turned off here
#    "font.family": "Helvetica",
    "font.family": "sans-serif",
    "font.size": 12,
    "axes.grid": True,
    "axes.grid.which": 'both',
    "grid.linestyle": '--',
    "grid.alpha": 0.7,
    "axes.labelsize": 10,
    "axes.titlesize": 16,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "legend.loc": 'upper right',
    "legend.fontsize": 9,
    "legend.columnspacing": 1.0,
    'figure.subplot.bottom': 0.15,
    'figure.subplot.left': 0.13,
    'figure.subplot.right': 0.96,
    'figure.subplot.top': 0.94,
    'figure.subplot.hspace': 0.15,
    'figure.subplot.wspace': 0.4,
    'figure.titlesize': 'large',
    'lines.linewidth': 1,
})

log = IndiflightLog(path.join(experimentDataPath, "LOG00271.BFL"))
log.data['gyroADC[1]'] *= -1. # data file was with old FLU frame
log.data['gyroADC[2]'] *= -1. # data file was with old FLU frame
log.data['gyroADCafterRpm[1]'] *= -1. # data file was with old FLU frame
log.data['gyroADCafterRpm[2]'] *= -1.
log.data['accUnfiltered[1]'] *= -1. # data file was with old FLU frame
log.data['accUnfiltered[2]'] *= -1.

timeMs = log.data['timeMs'] - 1435
boolarr = (timeMs > 0) & (timeMs < 457)
timeMs = timeMs[boolarr]
crop = log.data[boolarr]

f, axs = plt.subplots(2, 1, figsize=(5.0, 3.2), sharex='all')
#f, axs = plt.subplots(1, 3, figsize=(9, 2.2), sharex='all')
for i in range(4):
    axs[0].plot(timeMs,
                crop[f'motor[{i}]'],
                label=f'Motor {i+1}')
    #axs[0].set_xlabel("Time [ms]")
    axs[0].set_ylabel("Motor input $\delta$ [-]")

    #axs[1].plot(timeMs,
    #            crop[f'omegaUnfiltered[{i}]'],
    #            label=f'Motor {i}')
    #axs[1].set_xlabel("Time [ms]")
    #axs[1].set_ylabel("Motor Rotation Rate $\omega$ [rad/s]")

axs[1].plot(timeMs,
            crop[[f'gyroADCafterRpm[{i}]' for i in range(3)]])
axs[1].set_xlabel("Time [ms]")
axs[1].set_ylabel("Body Rate $\Omega$ [rad/s]")
axs[1].legend(["Roll", "Pitch", "Yaw"], loc='upper left', ncol=3)

axs[0].set_ylim(top=1.)
#axs[1].set_ylim(top=4000.)
axs[0].legend( ncol=4 )
#axs[1].legend( ncol=2 )

f.savefig(path.join(outputPath, 'Excitation.pdf'), format='pdf')

#%% regressors

plt.rcParams.update({
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "legend.loc": 'upper right',
    "legend.fontsize": 9,
    "legend.columnspacing": 2.0,
    'figure.subplot.bottom': 0.18,
    'figure.subplot.left': 0.1,
    'figure.subplot.right': 0.95,
    'figure.subplot.top': 0.85,
    'figure.subplot.hspace': 0.3,
    'figure.subplot.wspace': 0.35,
    'figure.titlesize': 'large',
    'lines.linewidth': 1,
    "axes.formatter.limits": [-2, 3]
})

timeMs = log.data['timeMs'] - 1435
boolarr = (timeMs > 0) & (timeMs < 500)
timeMs = timeMs[boolarr]
crop = log.data[boolarr]

order = 2
fc = 20 # Hz
r = np.array([-0.01, -0.01, 0.015])

omegaRaw = Signal(crop['timeS'], crop[[f'omegaUnfiltered[{i}]' for i in range(4)]])
gyroRaw = Signal(crop['timeS'], crop[[f'gyroADCafterRpm[{i}]' for i in range(3)]])
spfRaw = Signal(crop['timeS'], crop[[f'accUnfiltered[{i}]' for i in range(3)]])
spfRawCor = Signal(crop['timeS'], imuOffsetCorrection(spfRaw.y.copy(), gyroRaw.y, gyroRaw.dot().y, r))

omegaFilt = omegaRaw.filter('lowpass', order, fc)
gyroFilt = gyroRaw.filter('lowpass', order, fc)
spfFiltCor = spfRawCor.filter('lowpass', order, fc)

regSpf = 2. * omegaFilt.y * omegaFilt.diff().y
regRot = np.concatenate((regSpf, omegaFilt.dot().diff().y), axis=1)

for axi, ax in enumerate(['x', 'y', 'z']):
    frls, axs = plt.subplots(1, 2, figsize=(9, 3), sharex=True)

    theta = crop[[f'fx_{ax}_rls_x[{i}]' for i in range(4)]].to_numpy()
    reproduction = np.array([t.T @ r for t, r in zip(theta, regSpf)])

    axs[0].plot(timeMs, spfRawCor.diff().y[:, axi], alpha=0.5, lw=0.5, ls='--', label=f"Unfiltered Output")
    axs[0].plot(timeMs, spfFiltCor.diff().y[:, axi], lw=1.0, ls='-', label=f"Filtered Output")
    axs[0].plot(timeMs, reproduction, lw=1.5, ls='-.', label=f"Online reproduction")
    axs[0].set_xlabel("Time [ms]")
    axs[0].set_ylabel("Specific Force Delta [N/kg]")
    axs[0].set_ylim(bottom=-7, top=7)
    axs[0].legend(loc="lower left")

    axs[1].plot(timeMs, theta)
    axs[1].set_ylim(bottom=-0.3e-5, top=0.3e-5)
    axs[1].set_xlabel("Time [ms]")
    axs[1].set_ylabel("Motor effectiveness estimates [$\\frac{N/kg}{(rad/s)^2}$]")
    axs[1].legend([f'Motor {i}' for i in range(4)])

    frls.suptitle(f"Online Estimation for {ax}-Axis Force Effectiveness")

    frls.savefig(path.join(outputPath, f"Fx_estimation_{ax}.pdf"), format='pdf')

axis_names = ['Roll', 'Pitch', 'Yaw']
for axi, ax in enumerate(['p', 'q', 'r']):
    frls, axs = plt.subplots(1, 3, figsize=(9, 3), sharex=True)

    theta = crop[[f'fx_{ax}_rls_x[{i}]' for i in range(8)]].to_numpy()
    reproduction = np.array([t.T @ r for t, r in zip(theta, regRot)])

    axs[0].plot(timeMs, gyroRaw.dot().diff().y[:, axi], alpha=0.5, lw=0.5, ls='--', label=f"Unfiltered Output")
    axs[0].plot(timeMs, gyroFilt.dot().diff().y[:, axi], lw=1.0, ls='-', label=f"Filtered Output")
    axs[0].plot(timeMs, reproduction, lw=1.5, ls='-.', label=f"Online reproduction")
    axs[0].set_xlabel("Time [ms]")
    axs[0].set_ylabel("Rotation Acceleration Delta [$rad/s^2$]")
    axs[0].set_ylim(bottom=-30, top=30)
    axs[0].legend(loc="lower left")

    axs[1].plot(timeMs, theta[:, :4])
    if ax == 'r':
        axs[1].set_ylim(bottom=-0.5e-4, top=0.5e-4)
    else:
        axs[1].set_ylim(bottom=-1e-4, top=1e-4)
    axs[1].set_xlabel("Time [ms]")
    axs[1].set_ylabel("Effectiveness $B_1 k$ [$\\frac{Nm/(kg\cdot m^2)}{(rad/s)^2}$]")
    axs[1].legend([f'Motor {i+1}' for i in range(4)], loc='upper left', ncols=2)

    axs[2].plot(timeMs, theta[:, 4:])
    axs[2].set_ylim(bottom=-6e-3, top=6e-3)
    axs[2].set_xlabel("Time [ms]")
    axs[2].set_ylabel("Effectiveness $B_2$ [$\\frac{Nm/(kg\cdot m^2)}{(rad/s^2)}$]")
    axs[2].legend([f'Motor {i+1}' for i in range(4)], loc='upper right', ncols = 2)

    frls.suptitle(f"Online Estimation for {axis_names[axi]} Effectiveness")

    frls.savefig(path.join(outputPath, f"Fx_estimation_{ax}.pdf"), format='pdf')

plt.rcParams.update({
    'figure.subplot.bottom': 0.15,
    'figure.subplot.left': 0.1,
    'figure.subplot.right': 0.97,
    'figure.subplot.top': 0.95,
    'figure.subplot.hspace': 0.15,
    'figure.subplot.wspace': 0.35,
    "axes.formatter.limits": [-2, 4]
})

# recovery
timeMs = log.data['timeMs'] - 1435
boolarr = (timeMs > 474) & (timeMs < 2000)
timeMsRec = timeMs[boolarr]
rec = log.data[boolarr]

frec, axs = plt.subplots(2, 1, figsize=(5, 3.5), sharex=True)
axs[0].plot(timeMsRec, rec['gyroSp[0]'], "-", label="Roll rate reference")
axs[0].plot(timeMsRec, rec['gyroADCafterRpm[0]'], ls="none", marker='x', markevery=25, markersize=4, label="Roll rate")
axs[0].plot(timeMsRec, rec['gyroSp[1]'], "--", label="Pitch rate reference")
axs[0].plot(timeMsRec, rec['gyroADCafterRpm[1]'], ls="none", marker='s', markevery=25, markersize=4, label="Pitch rate", markerfacecolor="none")
#alpha = Signal(timeMsRec*1e-3, rec[[f'gyroADCafterRpm[{i}]' for i in range(3)]]).dot()
#
#axs[0].plot(timeMsRec, rec['alphaSp[0]'], "-", label="Roll rate reference")
#axs[0].plot(timeMsRec, alpha.y[:, 0], ls="none", marker='x', markevery=25, markersize=4, label="Roll rate")
#axs[0].plot(timeMsRec, rec['alphaSp[1]'], "--", label="Pitch rate reference")
#axs[0].plot(timeMsRec, alpha.y[:, 1], ls="none", marker='s', markevery=25, markersize=4, label="Pitch rate", markerfacecolor="none")
axs[0].set_ylabel("Body Rate [rad/s]")
axs[0].legend(loc="upper right", ncols=2)
axs[1].plot(timeMsRec, rec[[f'motor[{i}]' for i in range(4)]])
axs[1].set_xlabel("Time [ms]")
axs[1].set_ylabel("Motor command $\delta$ [-]")
axs[1].legend([f"Motor {i+1}" for i in range(4)], loc="lower right", ncols=2)
frec.savefig(path.join(outputPath, "Recovery.pdf"), format='pdf')

plt.rcParams.update({
    'figure.subplot.left': 0.05,
    'figure.subplot.right': 0.95,
    'figure.subplot.top': 0.85,
    'figure.subplot.hspace': 0.3,
    'figure.subplot.wspace': 0.35,
    "axes.formatter.limits": [-2, 3]
})

# motors
timeMs = log.data['timeMs'] - 1435
boolarr = (timeMs > 0) & (timeMs < 457)
timeMs = timeMs[boolarr]
crop = log.data[boolarr]

order = 2
fc = 40 # Hz

omegaRaw = Signal(crop['timeS'], crop[[f'omegaUnfiltered[{i}]' for i in range(4)]])
dRaw = Signal(crop['timeS'], crop[[f'motor[{m}]' for m in range(4)]])
dSqrtRaw = Signal(crop['timeS'], np.sqrt(crop[[f'motor[{m}]' for m in range(4)]]))

dFilt = dRaw.filter('lowpass', order, fc)
dSqrtFilt = dSqrtRaw.filter('lowpass', order, fc)

omegaMotorFilt = omegaRaw.filter('lowpass', order, fc)

for motor in range(4):
    frls, axs = plt.subplots(1, 3, figsize=(9, 3), sharex=True)

    theta = crop[[f'motor_{motor}_rls_x[{i}]' for i in range(4)]].to_numpy()
    wm = theta[:, 0] + theta[:, 1]
    k = theta[:, 0] / wm
    a = wm * k
    b = 1 - a
    theta[:, 0] = a
    theta[:, 1] = b
    regMotor = np.zeros_like(theta)
    regMotor[:, 0] = dFilt.y[:, motor]
    regMotor[:, 1] = dSqrtFilt.y[:, motor]
    regMotor[:, 2] = 1.
    regMotor[:, 3] = -omegaMotorFilt.dot().y[:, motor]
    reproduction = np.array([t.T @ r for t, r in zip(theta, regMotor)])

    axs[0].plot(timeMs, omegaRaw.y[:, motor], alpha=0.5, lw=0.5, ls='--', label=f"Unfiltered Output")
    axs[0].plot(timeMs, omegaMotorFilt.y[:, motor], lw=1.0, ls='-', label=f"Filtered Output")
    axs[0].plot(timeMs, reproduction, lw=1.5, ls='-.', label=f"Online reproduction")
    axs[0].set_xlabel("Time [ms]")
    axs[0].set_ylabel("Motor Rotation Rate [rad/s]")
    axs[0].set_ylim(bottom=-1e3, top=5e3)
    axs[0].legend(loc="upper left")

    axs[1].plot(timeMs, wm)
    axs[1].plot(timeMs, theta[:, 2])
    axs[1].set_ylim(bottom=-1e3, top=5e3)
    axs[1].set_xlabel("Time [ms]")
    axs[1].set_ylabel("Motor Parameters [rad/s]")
    axs[1].legend(['Max Speed', 'Idle Speed'], loc='upper left', ncols=1)

    axs[2].plot(timeMs, theta[:, 3])
    axs[2].set_ylim(bottom=-3e-2, top=5e-2)
    axs[2].set_xlabel("Time [ms]")
    axs[2].set_ylabel("Motor time constant $\\tau$ [s]")
    axs[2].legend("Time constant")

    frls.suptitle(f"Online Estimation for Motor Model {motor+1}")

    frls.savefig(path.join(outputPath, f"Motor_estimation_{motor+1}.pdf"), format='pdf')
