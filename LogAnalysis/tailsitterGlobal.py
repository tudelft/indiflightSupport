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
            'cx0', 'cxw',         'cxd',  'cxv',
            'cy0',                        'cyv',
            'cz0', 'czw',         'czd2', 'czv',
            'cl0', 'clw', 
            'cm0', 'cmw',         'cmd',       'cmddd',
            'cn0', 'cnw', 'cnwd', 'cnd'
        ]
        for name in coefficients:
            setattr(self, name, nn.Parameter(torch.zeros(1)))

    def forward(self, x):
        # STATE
        # body vel    body rate   body rate deriv    prop speeds (derivative)   elevon angles   elevon accelerations
        vx, vy, vz,  Ox, Oy, Oz,   Odx, Ody, Odz,      w1, w2, w1d, w2d,           d1, d2,            d1dd, d2dd =  x
        O  = torch.vstack((Ox, Oy, Oz))
        Od = torch.vstack((Odx, Ody, Odz))

        # REGRESSOR primitives
        ww = torch.stack([w1*w1, w2*w2])                              # prop speeds ** 2
        wwDd = ww * torch.sin( (torch.stack([d1, d2]) - self.d0) )    # prop speeds ** 2 * sin ( elevon angles )
        #wwDd = ww * ( (torch.stack([d1, d2]) - self.d0) )    # prop speeds ** 2 * sin ( elevon angles )
        d2abs = torch.sin( d1 ).abs() + torch.sin( d2 ).abs()         # for reduction of the prop thrust
        vxavx, vyavy, vzavz = vx.abs()*vx, vy.abs()*vy, vz.abs()*vz   # for quadratic drag

        # FORCE / MOMENT MODEL
        #     offset      prop thrust contribution         prop rate contribution           elevon contribution          elev acc contrib                  drag
        fx = self.cx0  +  self.cxw * (ww[0] + ww[1])  +  0                        +  self.cxd * (wwDd[0] + wwDd[1])  +  0                           +  self.cxv * vxavx
        fy = self.cy0  +  0                           +  0                        +  0                               +  0                           +  self.cyv * vyavy
        fz = self.cz0  +  self.czw * (ww[0] + ww[1])  +  0                        +  self.czd2 * d2abs               +  0                           +  self.czv * vzavz
        mx = self.cl0  +  self.clw * (ww[0] - ww[1])  +  0                        +  0                               +  0                           +  0
        my = self.cm0  +  self.cmw * (ww[0] + ww[1])  +  0                        +  self.cmd * (wwDd[0] + wwDd[1])  +  self.cmddd * (d1dd + d2dd)  +  0
        mz = self.cn0  +  self.cnw * (ww[0] - ww[1])  +  self.cnwd * (w1d - w2d)  +  self.cnd * (wwDd[0] - wwDd[1])  +  0                           +  0

        f = torch.stack([fx, fy, fz])
        m = torch.stack([mx, my, mz])

        # KINETICS (todo: use measured dOdt, for IMU offset contribution to fIMU?)
        a_modelled = f  +  Od.cross(self.r, dim=0)  +  O.cross(O.cross(self.r, dim=0), dim=0)
        dO_modelled = m # if I known: dOdt = inv(I) * (m - O.cross(self.I * O, dim=0))

        return torch.concat([a_modelled, dO_modelled], dim=0)

class Motor(nn.Module):
    def __init__(self):
        super(Motor, self).__init__()

        self.tau = nn.Parameter(0.1*torch.ones(1))
        self.idle = nn.Parameter(0*torch.ones(1))
        self.max = nn.Parameter(1000*torch.ones(1))
        self.k = nn.Parameter(0.0*torch.ones(1))

    def forward(self, delta, wd):
        return self.max * (self.k * delta + (1-self.k) * delta**0.5) + self.idle - self.tau * wd


#%% DATA loading
log = IndiflightLog("/mnt/data/WorkData/BlackboxLogs/MIRROR_DarkO/LOG00043.BFL", resetTime=True)

data = log.data[(log.data['timeS'] > 10) & (log.data['timeS'] < 50)] # todo: better flight detection
t = data['timeS']
mean_dt = t.diff().mean()

# convert columns to tensors
v_true  = torch.tensor( data[[f'extVel[{i}]' for i in range(3)]].to_numpy(dtype=np.float32).T          ).to(device=device)
q_true  = torch.tensor( data[[f'extAtt[{i}]' for i in range(4)]].to_numpy(dtype=np.float32).T / 8.128  ).to(device=device)
#a_true  = torch.tensor( data[[f'accADCafterRpm[{i}]' for i in range(3)]].to_numpy(dtype=np.float32).T  ).to(device=device)
a_true_raw  = torch.tensor( data[[f'accADCafterRpm[{i}]' for i in range(3)]].to_numpy(dtype=np.float32).T  ).to(device=device)
a_true = torch.tensor( savgol_filter(a_true_raw.cpu(), window_length=9, polyorder=3) ).to(device=device)
O_true  = torch.tensor( data[[f'gyroADCafterRpm[{i}]' for i in range(3)]].to_numpy(dtype=np.float32).T ).to(device=device)
Od_true = torch.tensor( savgol_filter(O_true.cpu(), window_length=15, polyorder=4, deriv=1, delta=mean_dt) ).to(device=device)
w_true_raw  = torch.tensor( data[[f'omegaUnfiltered[{i}]' for i in range(2)]].to_numpy(dtype=np.float32).T ).to(device=device)
w_true = torch.tensor( savgol_filter(w_true_raw.cpu(), window_length=25, polyorder=4) ).to(device=device)
wd_true = torch.tensor( savgol_filter(w_true_raw.cpu(), window_length=25, polyorder=4, deriv=1, delta=mean_dt) ).to(device=device)
d_true_raw = torch.tensor( data[[f'servo_feedback[{i}]' for i in range(2)]] .to_numpy(dtype=np.float32).T ).to(device=device) / 100 * np.pi/180
d_true_raw[1] *= -1
d_true = torch.tensor( savgol_filter(d_true_raw.cpu(), window_length=50, polyorder=4) ).to(device=device)
dd_true = torch.tensor( savgol_filter(d_true_raw.cpu(), window_length=50, polyorder=4, deriv=1, delta=mean_dt) ).to(device=device)
ddd_true = torch.tensor( savgol_filter(d_true_raw.cpu(), window_length=50, polyorder=4, deriv=2, delta=mean_dt) ).to(device=device)
del_12 = torch.tensor( data[[f'motor[{i}]' for i in range(2)]] .to_numpy(dtype=np.float32).T ).to(device=device)

# get velocity in body frame for drag model
v_body_true  = quaternion_rotate(q_true, v_true, inverse=True)


#%% MODEL optimisation
model = Tailsitter()
model.to(device=device)

# ablations: keep parameters at their initial value from __init__
#exclude = ['cx0', 'cy0', 'cz0', 'cl0', 'cm0', 'cn0', 'czd2', 'cmddd', 'cyv']
#exclude = ['cx0', 'cy0', 'cz0', 'cl0', 'cm0', 'cn0', 'czd2', 'cyv']
exclude = ['cx0', 'cy0', 'cz0', 'cl0', 'cm0', 'cn0', 'czd2', 'cxw']
#exclude = ['cx0', 'cy0', 'cz0', 'cl0', 'cm0', 'cn0', 'czd2'] #, 'cmddd', 'cyv']
#exclude = ['cx0', 'cy0', 'cz0', 'cl0', 'cm0', 'cn0']
#exclude = ['cx0', 'cy0', 'cz0']
#exclude = []
for par in exclude:
    model.get_parameter(par).requires_grad = False

# regressors and targets
x = torch.vstack([
    v_body_true,
    O_true,
    Od_true,
    w_true / 1000,  # to make regressors roughly unit
    wd_true / 20,
    d_true,
    ddd_true / 100, # servo angular acceleration
])[:, 1:-1]
y_true = torch.concat((a_true, Od_true))[:, 1:-1]

# loss function           accelerations   gyro derivatives
weights = torch.tensor([[0.05, 0.05, 0.05, 0.01, 0.01, 0.01]]).T.to(device=device)
def weighted_mse_loss(pred, true, weight):
    return torch.sum(weight * (pred - true) ** 2)

optimizer = optim.LBFGS(model.parameters(), lr=5e-1); epochs = 20
#optimizer = optim.AdamW(model.parameters(), lr=1e-1); epochs = 2000

for epoch in range(epochs):
    def closure():
        optimizer.zero_grad()
        y_pred = model(x)
        loss = weighted_mse_loss(y_pred, y_true, weights)
        loss.backward()
        return loss

    optimizer.step(closure)

    if epoch % 1 == 0:
        print(f'Model Epoch {epoch}: Loss = {closure().item():.5f}')


#%% MOTOR optimization for Motor 1 only for now
motor = Motor()
motor.to(device=device)

# ablations: keep parameters at their initial value from __init__
motor.get_parameter('idle').requires_grad = False

# regressors and targets
x1, x2 = del_12[0], wd_true[0]
y_motor_true = w_true[0]

# loss
def mse_loss(pred, true):
    return torch.sum((pred - true)**2)

optimizer = optim.LBFGS(motor.parameters(), lr=2e-1); epochs = 100

for epoch in range(epochs):
    def closure():
        optimizer.zero_grad()
        y_pred = motor(x1, x2)
        loss = mse_loss(y_pred, y_motor_true)
        loss.backward()
        return loss

    optimizer.step(closure)

    if epoch % 10 == 0:
        print(f'Motor Epoch {epoch}: Loss = {closure().item():.5f}')


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
# trim condition / initial guesses. False means not optimized
v_trim   = nn.Parameter(torch.zeros(3, 1), requires_grad=False) # body speed
O_trim   = nn.Parameter(torch.zeros(3, 1), requires_grad=False) # body rates
Od_trim  = nn.Parameter(torch.zeros(3, 1), requires_grad=False) # body rate derivatives
w_trim   = nn.Parameter(torch.ones (2, 1), requires_grad=True)  # motor speeds
wd_trim  = nn.Parameter(torch.zeros(2, 1), requires_grad=False) # motor rate
d_trim   = nn.Parameter(torch.zeros(2, 1), requires_grad=True)  # elevon angle
ddd_trim = nn.Parameter(torch.zeros(2, 1), requires_grad=False) # elevon angle second derivative
trim_pars = [v_trim, O_trim, Od_trim, w_trim, wd_trim, d_trim, ddd_trim]

# dont allow model parameters to change, and set IMU offset to zero for the jacobians to make more sense
from copy import deepcopy
model_to_trim = deepcopy(model)
model_to_trim.requires_grad_(False)
model_to_trim.r.set_(torch.zeros((3,1)))

# define loss as  norm(dOdt)**2  +  (norm(f)**2 - G**2) ** 2
def trim_loss(output):
    return torch.sum(output[3:, 0] ** 2)  +  ( torch.sum(output[:3, 0] ** 2) - 9.81**2 ) ** 2

# optimize!
optimizer = optim.LBFGS(trim_pars, lr=5e-1)
for epoch in range(5):
    def closure():
        optimizer.zero_grad()
        v = model_to_trim(torch.concat(trim_pars))
        loss = trim_loss(v)
        loss.backward()
        return loss

    optimizer.step(closure)

    if epoch % 1 == 0:
        print(f'Trimming epoch {epoch+1}: Loss = {closure().item():.5f}')


# %% Hover jacobian

# turn on automatic gradient computation for all inputs
#with torch.no_grad():
    #w_trim.set_(torch.tensor([[1, 1.]]).T)
    #d_trim.set_(torch.tensor([[0.8, 0.8]]).T)
wd_trim.requires_grad = True
ddd_trim.requires_grad = True

# calculate 
y = model_to_trim(torch.concat(trim_pars))

# assemble jacobians 
G1 = torch.zeros((6, 4))
G2 = torch.zeros((6, 2))
G3 = torch.zeros((6, 2))
for i in range(6):
    output = torch.zeros((6, 1))
    output[i, 0] = 1.

    G1[i, 0:2] = 1/(1000**2 * 2*w_trim.T) * torch.autograd.grad(y, w_trim, grad_outputs=output, retain_graph=True)[0].T
    G1[i, 2:4] = torch.autograd.grad(y, d_trim, grad_outputs=output, retain_graph=True)[0].T
    G2[i, 0:2] = 1/20 * torch.autograd.grad(y, wd_trim, grad_outputs=output, retain_graph=True)[0].T
    G3[i, 0:2] = 1/100 * torch.autograd.grad(y, ddd_trim, grad_outputs=output, retain_graph=True)[0].T

G1_indi = G1
G2_indi = G2
G3_indi = G3

# to u-units
G1_indi[:, :2] *= motor.max[0] ** 2 # omega**2 = omega_max**2 * u
G1_indi[:, 2:] *= 100*np.pi / 180   # u is in hectodegrees (...)
G1_indi[:,  3] *= -1                # right servo is flipped
G1_indi[5, :2] *= -1                # I have no idea why
G2_indi[5, :2] *= -1                # I have no idea why
G3_indi[:,  1] *= -1                # right servo is flipped

# to integers
G1_indi[:3, :] *= 100
G1_indi[3:, :] *= 10
G2_indi        *= 1e5
G3_indi        *= 1e3

print("\n===== G1 =====")
print(G1_indi.round().type(torch.int16))

print("\n===== G2 =====")
print(G2_indi.round().type(torch.int16))

print("\n===== G3 =====")
print(G3_indi.round().type(torch.int16))

print("\n===== TAILSITTER parameters =====")
print()
print(f"set indi_tails_use_scheduled = 1")
print(f"set indi_tails_use_sine = 1")
print()
print(f"set indi_tails_d0 = {int(model.d0[0,0]*100*180/np.pi)}, {int(model.d0[1,0]*100*180/np.pi)}")
print()
print(f"set indi_tails_cxw = {int(model.cxw[0]*1e-6*1e9)}")
print(f"set indi_tails_cyw = 0")
print(f"set indi_tails_czw = {int(model.czw[0]*1e-6*1e9)}")
print(f"set indi_tails_clw = {int(model.clw[0]*1e-6*1e8)}")
print(f"set indi_tails_cmw = {int(model.cmw[0]*1e-6*1e8)}")
print(f"set indi_tails_cnw = {int(model.cnw[0]*1e-6*1e8)}")
print()
print(f"set indi_tails_cnwd = {int(model.cnwd[0]/20*1e5)}")
print()
print(f"set indi_tails_cxd = {int(model.cxd[0]*1e-6*1e8)}")
print(f"set indi_tails_cmd = {int(model.cmd[0]*1e-6*1e8)}")
print(f"set indi_tails_cnd = {int(model.cnd[0]*1e-6*1e8)}")
print()
