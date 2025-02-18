#%% 
from indiflight_log_tools import IndiflightLog
from matplotlib import pyplot as plt
import numpy as np

import torch
import torch.nn as nn
import torch.optim as optim

from scipy.signal import savgol_filter

def quaternion_rotate(q, v, inverse=False):
    # helper function to rotate a vector with a quaternion
    w, x, y, z = (-1 if inverse else 1) * q[0, :], q[1, :], q[2, :], q[3, :]
    vx, vy, vz = v[0, :], v[1, :], v[2, :]

    tx =  w * vx + y * vz - z * vy
    ty =  w * vy + z * vx - x * vz
    tz =  w * vz + x * vy - y * vx
    tw = -x * vx - y * vy - z * vz

    # Compute result quaternion (q * v * q_conj)
    rx = tw * -x + tx *  w + ty * -z - tz * -y
    ry = tw * -y + ty *  w + tz * -x - tx * -z
    rz = tw * -z + tz *  w + tx * -y - ty * -x

    return torch.vstack([rx, ry, rz])

#%% DEFINE model
#device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
device = torch.device("cpu")  # LBGFS is faster on cpu

class Tailsitter(nn.Module):
    def __init__(self):
        super(Tailsitter, self).__init__()

        self.r = nn.Parameter(torch.zeros(3, 1))  # IMU offset
        self.d0 = nn.Parameter(torch.zeros(2, 1)) # 0-force elevon angle
        coefficients = [
            'cx0', 'cxw',         'cxd', 'cxv',
            'cy0',                       'cyv',
            'cz0', 'czw',         'czd2',
            'cl0', 'clw', 
            'cm0', 'cmw',         'cmd',       'cmddd',
            'cn0', 'cnw', 'cnwd', 'cnd'
        ]
        for name in coefficients:
            setattr(self, name, nn.Parameter(torch.zeros(1)))

    def forward(self, x):
        # STATE
        # body vel   body rate   prop speeds (derivative)   elevon angles   elevon accelerations
        vx, vy, vz,  Ox, Oy, Oz,   w1, w2, w1d, w2d,           d1, d2,            ddd1, ddd2 =  x
        O = torch.vstack((Ox, Oy, Oz))

        # REGRESSOR primitives
        ww = torch.stack([w1*w1, w2*w2])                              # prop speeds ** 2
        wwDd = ww * torch.tan( (torch.stack([d1, d2]) - self.d0) )    # prop speeds ** 2 * tan ( elevon angles )
        d2abs = torch.tan( d1 ).abs() + torch.tan( d2 ).abs()         # for reduction of the prop thrust
        vxavx, vyavy, vzavz = vx.abs()*vx, vy.abs()*vy, vz.abs()*vz   # for quadratic drag

        # FORCE / MOMENT MODEL
        #     offset      prop thrust contribution         prop rate contribution           elevon contribution          elev acc contrib                  drag
        fx = self.cx0  +  self.cxw * (ww[0] + ww[1])  +  0                        +  self.cxd * (wwDd[0] + wwDd[1])  +  0                           +  self.cxv * vxavx
        fy = self.cy0  +  0                           +  0                        +  0                               +  0                           +  self.cyv * vyavy
        fz = self.cz0  +  self.czw * (ww[0] + ww[1])  +  0                        +  self.czd2 * d2abs               +  0                           +  0
        mx = self.cl0  +  self.clw * (ww[0] - ww[1])  +  0                        +  0                               +  0                           +  0
        my = self.cm0  +  self.cmw * (ww[0] + ww[1])  +  0                        +  self.cmd * (wwDd[0] + wwDd[1])  +  self.cmddd * (ddd1 + ddd2)  +  0
        mz = self.cn0  +  self.cnw * (ww[0] - ww[1])  +  self.cnwd * (w1d - w2d)  +  self.cnd * (wwDd[0] - wwDd[1])  +  0                           +  0

        f = torch.stack([fx, fy, fz])
        m = torch.stack([mx, my, mz])

        # KINETICS (todo: use measured dOdt, for IMU offset contribution to fIMU?)
        dOdt = m # if I known: dOdt = inv(I) * (m - O.cross(self.I * O, dim=0))
        fIMU = f  +  dOdt.cross(self.r, dim=0) + O.cross(O.cross(self.r, dim=0), dim=0)

        return torch.concat([fIMU, dOdt], dim=0)

model = Tailsitter()
model.to(device=device)

# ablations: keep parameters at their initial value from __init__
#exclude = ['cx0', 'cy0', 'cz0', 'cl0', 'cm0', 'cn0', 'czd2', 'cmddd', 'cyv']
exclude = ['cx0', 'cy0', 'cz0', 'cl0', 'cm0', 'cn0', 'czd2', 'cyv']
#exclude = ['cx0', 'cy0', 'cz0', 'cl0', 'cm0', 'cn0', 'cyv']
for par in exclude:
    model.get_parameter(par).requires_grad = False


#%% LOSS and OPTIMIZER
# loss function           accelerations   gyro derivatives
weights = torch.tensor([[0.05, 0.05, 0.05, 0.01, 0.01, 0.01]]).T.to(device=device)
def weighted_mse_loss(pred, true, weight):
    return torch.sum(weight * (pred - true) ** 2)

optimizer = optim.LBFGS(model.parameters(), lr=1e-2); epochs = 100
#optimizer = optim.AdamW(model.parameters(), lr=2e-1); epochs = 2000


#%% DATA loading
log = IndiflightLog("/mnt/data/WorkData/BlackboxLogs/MIRROR_DarkO/LOG00043.BFL", resetTime=True)

data = log.data[(log.data['timeS'] > 10) & (log.data['timeS'] < 50)] # todo: better flight detection
t = data['timeS']
mean_dt = t.diff().mean()

# convert columns to tensors
v_true  = torch.tensor( data[[f'extVel[{i}]' for i in range(3)]].to_numpy(dtype=np.float32).T          ).to(device=device)
q_true  = torch.tensor( data[[f'extAtt[{i}]' for i in range(4)]].to_numpy(dtype=np.float32).T / 8.128  ).to(device=device)
a_true  = torch.tensor( data[[f'accADCafterRpm[{i}]' for i in range(3)]].to_numpy(dtype=np.float32).T  ).to(device=device)
#a_true = torch.tensor( savgol_filter(a_true_raw.cpu(), window_length=9, polyorder=3) ).to(device=device)
O_true  = torch.tensor( data[[f'gyroADCafterRpm[{i}]' for i in range(3)]].to_numpy(dtype=np.float32).T ).to(device=device)
Od_true = torch.tensor( savgol_filter(O_true.cpu(), window_length=9, polyorder=3, deriv=1, delta=mean_dt) ).to(device=device)
w_true_raw  = torch.tensor( data[[f'omegaUnfiltered[{i}]' for i in range(2)]].to_numpy(dtype=np.float32).T ).to(device=device)
w_true = torch.tensor( savgol_filter(w_true_raw.cpu(), window_length=25, polyorder=4) ).to(device=device)
wd_true = torch.tensor( savgol_filter(w_true_raw.cpu(), window_length=25, polyorder=4, deriv=1, delta=mean_dt) ).to(device=device)
d_true_raw = torch.tensor( data[[f'servo_feedback[{i}]' for i in range(2)]] .to_numpy(dtype=np.float32).T ).to(device=device) / 100 * np.pi/180
d_true_raw[1] *= -1
d_true = torch.tensor( savgol_filter(d_true_raw.cpu(), window_length=50, polyorder=4) ).to(device=device)
dd_true = torch.tensor( savgol_filter(d_true_raw.cpu(), window_length=50, polyorder=4, deriv=1, delta=mean_dt) ).to(device=device)
ddd_true = torch.tensor( savgol_filter(d_true_raw.cpu(), window_length=50, polyorder=4, deriv=2, delta=mean_dt) ).to(device=device)

# central differences
#Od_true = 0.5/mean_dt*(O_true.roll(shifts=-1, dims=1) - O_true.roll(shifts=1, dims=1))

# get velocity in body frame for drag model
v_body_true  = quaternion_rotate(q_true, v_true, inverse=True)

# INPUTS AND TARGETS
x = torch.vstack([
    v_body_true,
    O_true,
    w_true / 1000,  # to make regressors roughly unit
    wd_true / 20,
    d_true,
    ddd_true / 100, # servo angular acceleration
])[:, 1:-1]

y_true = torch.concat((a_true, Od_true))[:, 1:-1]


#%% RUN optimisation
for epoch in range(epochs):
    def closure():
        optimizer.zero_grad()
        y_pred = model(x)
        loss = weighted_mse_loss(y_pred, y_true, weights)
        loss.backward()
        return loss

    optimizer.step(closure)

    if epoch % 10 == 0:
        print(f'Epoch {epoch}: Loss = {closure().item():.5f}')


#%% PLOTS
fig, axs = plt.subplots(6, 2, sharex=True)
fig.suptitle("DarkO Hover-model (based on motor speeds, elevon deflection, and speed)")
fig.subplots_adjust(left=0.086, bottom=0.11, right=0.952, top=0.907)

# model
IMU_LABELS = ["a_x\ (N/kg)", "a_y\ (N/kg)", "a_z\ (N/kg)", "\dot\Omega_x\ (rad/s^2)", "\dot\Omega_y\ (rad/s^2)", "\dot\Omega_z\ (rad/s^2)"]
for i in range(6):
    axs[i, 0].plot(t[1:-1], y_true[i].cpu(), '-')
    axs[i, 0].plot(t[1:-1], model.forward(x).detach().cpu()[i], '--')
    axs[i, 0].set_ylabel(f"${IMU_LABELS[i]}$")

axs[0, 0].legend(["Raw Measured", "Modelled"])
[ax.set_ylim((-10, 10)) for ax in axs[0:2, 0]]
axs[2, 0].set_ylim((-30, 10))
[ax.set_ylim((-150, +150)) for ax in axs[3:, 0]]

# state and inputs
VEL_LABELS = ["v_{xy}^B\ (m/s)", "v_z^B\ (m/s)"]
for i in range(2):
    axs[0, 1].plot(t[1:-1], v_true[i, 1:-1].cpu().T)
    axs[0, 1].set_ylabel(f"${VEL_LABELS[i]}$")
    axs[0, 1].set_ylim((-3, 3))
axs[1, 1].plot(t[1:-1], v_true[2, 1:-1].cpu().T)
axs[1, 1].set_ylabel(f"${VEL_LABELS[1]}$")
axs[1, 1].set_ylim((-3, 3))

for i in range(2):
    axs[2, 1].plot(t[1:-1], w_true[i, 1:-1].cpu().T, label=f"Motor {i+1}")
    axs[2, 1].set_ylabel("$\omega\ (rad/s)$")
    # axs[3, 1].plot(t[1:-1], 180/np.pi*d_true_raw[i, 1:-1].cpu().T)
    axs[3, 1].plot(t[1:-1], 180/np.pi*d_true[i, 1:-1].cpu().T, label=f"Elevon {i+1}")
    axs[3, 1].set_ylabel("$\delta\ (deg)$")
    axs[4, 1].plot(t[1:-1], 180/np.pi*dd_true[i, 1:-1].cpu().T, label=f"Elevon {i+1}")
    axs[4, 1].set_ylabel("$\dot\delta\ (deg/s)$")
    axs[5, 1].plot(t[1:-1], 180/np.pi*ddd_true[i, 1:-1].cpu().T, label=f"Elevon {i+1}")
    axs[5, 1].set_ylabel("$\ddot\delta\ (deg/s^2)$")
[ax.legend() for ax in axs[2:6, 1]]
[ax.legend() for ax in axs[2:6, 1]]

# dress up and plot
[ax.grid(True) for ax in axs.flatten()]
[ax.set_xlabel("Time [s]") for ax in axs[-1, :]]
fig.show()

#%% TRIM

class Trimmer(nn.Module):
    def __init__(self):
        super(Trimmer, self).__init__()
        self.v = nn.Parameter(torch.zeros(3, 1))    # body speed (keep zero)
        self.O = nn.Parameter(torch.zeros(3, 1))    # body rates (keep zero)
        self.w = nn.Parameter(torch.ones(2, 1))     # trim
        self.wd = nn.Parameter(torch.zeros(2, 1))   # keep zero
        self.d = nn.Parameter(torch.zeros(2, 1))    # trim
        self.ddd = nn.Parameter(torch.zeros(2, 1))  # keep zero

        self.v.requires_grad = False
        self.O.requires_grad = False
        self.wd.requires_grad = False
        self.ddd.requires_grad = False

    def forward(self, model):
        state = torch.concat((self.v, self.O, self.w, self.wd, self.d, self.ddd))
        return model.forward(state)

def trim_loss(output):
    return torch.sum(output[3:, 0] ** 2)  +  ( torch.sum(output[:3, 0] ** 2) - 9.81**2 ) ** 2

trim = Trimmer()
optimizer = optim.LBFGS(trim.parameters(), lr=5e-1)

model.requires_grad_(False)
model.r.set_(torch.zeros((3,1)))
print(f'Epoch {0}: Loss = {trim_loss(trim.forward(model)):.5f}')
for epoch in range(10):
    def closure():
        optimizer.zero_grad()
        e = trim(model)
        loss = trim_loss(e)
        loss.backward()
        return loss

    optimizer.step(closure)

    if epoch % 1 == 0:
        print(f'Epoch {epoch+1}: Loss = {closure().item():.5f}')

# %% Hover jacobian

trim.wd.requires_grad = True
trim.ddd.requires_grad = True
state = torch.concat((trim.v, trim.O, trim.w, trim.wd, trim.d, trim.ddd))
y = model(state)

G1 = torch.zeros((6, 4))
G2 = torch.zeros((6, 2))
G3 = torch.zeros((6, 2))
for i in range(6):
    output = torch.zeros((6, 1))
    output[i, 0] = 1.

    G1[i, 0:2] = 1 / (2 * trim.w.T * 1000 ** 2) * torch.autograd.grad(y, trim.w, grad_outputs=output, retain_graph=True)[0].T
    G2[i, 0:2] = 1 / 20 * torch.autograd.grad(y, trim.wd, grad_outputs=output, retain_graph=True)[0].T
    G1[i, 2:4] = torch.autograd.grad(y, trim.d, grad_outputs=output, retain_graph=True)[0].T
    G3[i, 0:2] = 1 / 100 * torch.autograd.grad(y, trim.ddd, grad_outputs=output, retain_graph=True)[0].T

wmax = 2800

G1_indi = G1

# to u-units
G1_indi[:, :2] *= 2800*2800       # omega**2 = omega_max**2 * u
G1_indi[:, 2:] *= 100*np.pi / 180 # u is in hectodegrees (...)
G1_indi[:,  3] *= -1              # right servo is flipped

# to integers
G1_indi[:3, :] *= 100
G1_indi[3:, :] *= 10
G2_indi         = G2 * 1e5
G3_indi         = G3 * 1e3

print("\n===== G1 =====")
print(G1_indi.round().type(torch.int16))

print("\n===== G2 =====")
print(G2_indi.round().type(torch.int16))

print("\n===== G3 =====")
print(G3_indi.round().type(torch.int16))