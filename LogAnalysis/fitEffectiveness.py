from indiflight_log_tools import IndiflightLog
from indiflightLogTools import Signal
import argparse
import numpy as np
from matplotlib import pyplot as plt

model_list = ["quad", "tailsitter"]

parser = argparse.ArgumentParser(description='Generate model from logs.')
parser.add_argument('logfile', type=str, help='Path to log file')
parser.add_argument('--model', choices=model_list, default="quad", help='Model name')
parser.add_argument('--range', type=int, nargs=2, help='Start and end timerange (ms). Default entire log')

args = parser.parse_args()

#%% import and filter data

# order = 2 # todo: implement
cutoff = 5 # Hz

log = IndiflightLog(args.logfile, timeRange=args.range, resetTime=True)

avgDtUs = (log.data.timeUs.iloc[-1] - log.data.timeUs.iloc[0]) / log.data.timeUs.count()
log.resample(avgDtUs)

t         = log.data["timeS"].to_numpy()
acc       = Signal(t, log.data[[f"accADCafterRpm[{i}]" for i in range(3)]].to_numpy())
gyro      = Signal(t, log.data[[f"gyroADCafterRpm[{i}]" for i in range(3)]].to_numpy())
rpm       = Signal(t, log.data[[f"omegaUnfiltered[{i}]" for i in range(2)]].to_numpy())
servo     = Signal(t, 0.01*log.data[[f"servo_feedback[{i}]" for i in range(2)]].to_numpy())
motor_cmd = Signal(t, log.data[[f"motor[{i}]" for i in range(2)]].to_numpy())
servo_cmd = Signal(t, log.data[[f"u[{i}]" for i in range(2,4)]].to_numpy())

#%% build pitch model

pitch = lambda w, s, Dw, Ds, Ddds: np.vstack([
    2*s[:, 0]*w[:, 0]*Dw[:, 0]  +  Ds[:, 0] * w[:, 0]**2,
    2*Dw[:, 0]*w[:, 0],
    Ddds[:, 0],
    2*s[:, 1]*w[:, 1]*Dw[:, 1]  +  Ds[:, 1] * w[:, 1]**2,
    2*Dw[:, 1]*w[:, 1],
    Ddds[:, 1],
]).T

Ap = pitch(rpm.y, servo.y, rpm.diff().y, servo.diff().y, servo.filter("lowpass", order=2, cutoff_hz=cutoff).dot(order=2).diff().y)
yp = gyro.dot().diff().y[:, 1]

ApF = Signal(t, Ap).filter("lowpass", 2, cutoff).y
ApF[:, 2] = Ap[:, 2]
ApF[:, 5] = Ap[:, 5]
ypF = Signal(t, yp).filter("lowpass", 2, cutoff).y

x = (np.linalg.inv(ApF.T @ ApF) @ ApF.T @ ypF).squeeze()
print(x)


#%% evaluate

ypF_fit = ApF @ x

from matplotlib.gridspec import GridSpec

fig = plt.figure(figsize=(15, 12))
gs = GridSpec(5, 2, figure=fig)

axs = []

gyroF = gyro.filter("lowpass", order=2, cutoff_hz=cutoff)
rpmF = rpm.filter("lowpass", order=2, cutoff_hz=cutoff)
servoF = servo.filter("lowpass", order=2, cutoff_hz=cutoff)

ax = fig.add_subplot(gs[0, 0])
ax.plot(t, gyro.y[:, 1], "--", label="raw pitch rate")
ax.plot(t, gyro.y[:, 2], "--", label="raw yaw rate")
ax.plot(t, gyroF.y[:, 1], "-", label="filtered pitch rate")
ax.plot(t, gyroF.y[:, 2], "-", label="filtered yaw rate")
ax.set_ylabel("rad/s")
axs.append(ax)

ax = fig.add_subplot(gs[0, 1], sharex=axs[0])
axs.append(ax)


ax = fig.add_subplot(gs[1, 0], sharex=axs[0])
ax.plot(t, rpm.y[:, 0], label="$\omega_1$")
ax.plot(t, rpmF.y[:, 0], label="Filtered $\omega_1$")
ax.set_ylabel("$rad/s$")
axs.append(ax)

ax = fig.add_subplot(gs[1, 1], sharex=axs[0])
ax.plot(t, rpm.y[:, 1], label="$\omega_2$")
ax.plot(t, rpmF.y[:, 1], label="Filtered $\omega_2$")
axs.append(ax)


ax = fig.add_subplot(gs[2, 0], sharex=axs[0])
ax.plot(t, servo.y[:, 0], label="Servo 1")
ax.plot(t, servoF.y[:, 0], label="Filtered Servo 1")
ax.set_ylabel("$^\circ$")
ax.set_xlabel("time [s]")
axs.append(ax)

ax = fig.add_subplot(gs[2, 1], sharex=axs[0])
ax.plot(t, servo.y[:, 1], label="Servo 2")
ax.plot(t, servo.filter("lowpass", order=2, cutoff_hz=1).dot().dot().diff().y[:, 1])
ax.plot(t, servoF.y[:, 1], label="Filtered Servo 2")
ax.set_xlabel("time [s]")
axs.append(ax)


ax = fig.add_subplot(gs[3, 0], sharex=axs[0])
ax.plot(t, ypF, label="Filtered $\Delta\dot \Omega$")
ax.plot(t, ypF_fit, '--', label="Model")
ax.set_ylabel("rad/s/s")
axs.append(ax)

ax = fig.add_subplot(gs[4, 0], sharex=axs[0])
#axs.plot(t, Ap, "--", label="Regressors")
ax.plot(t, ApF, "-", label="Filtered Regressors")
ax.set_ylabel("$rad^2/s^2$")
ax.set_xlabel("time [s]")
axs.append(ax)

for ax in axs:
    ax.legend(loc="center left")
    ax.grid(which='both')

#fig.tight_layout()
fig.show()



