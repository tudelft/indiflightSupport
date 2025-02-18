from indiflight_log_tools import IndiflightLog
from matplotlib import pyplot as plt
import numpy as np

import torch
import torch.nn as nn
import torch.optim as optim
from torchdiffeq import odeint

def quaternion_rotate(q, v):
    w, x, y, z = -q[0, :], q[1, :], q[2, :], q[3, :]
    vx, vy, vz = v[0, :], v[1, :], v[2, :]

    tx =  w * vx + y * vz - z * vy
    ty =  w * vy + z * vx - x * vz
    tz =  w * vz + x * vy - y * vx
    tw = -x * vx - y * vy - z * vz

    # Compute result quaternion (q * v * q_conj)
    rx = tw * -x + tx *  w + ty * -z - tz * -y
    ry = tw * -y + ty *  w + tz * -x - tx * -z
    rz = tw * -z + tz *  w + tx * -y - ty * -x

    return np.array([rx, ry, rz])

class Tailsitter(nn.Module):
    def __init__(self):
        super(Tailsitter, self).__init__()
        self.r = nn.Parameter(torch.zeros(3,1))
        self.cx0 = nn.Parameter(torch.zeros(1))
        self.cxw = nn.Parameter(torch.zeros(1))
        self.cxd = nn.Parameter(torch.zeros(1))
        self.cxv = nn.Parameter(torch.zeros(1))
        self.cy0 = nn.Parameter(torch.zeros(1))
        self.cyv = nn.Parameter(torch.zeros(1))
        self.cz0 = nn.Parameter(torch.zeros(1))
        self.czw = nn.Parameter(torch.zeros(1))
        self.czd2 = nn.Parameter(torch.zeros(1))
        self.cmx0 = nn.Parameter(torch.zeros(1))
        self.cmxw = nn.Parameter(torch.zeros(1))
        self.cmy0 = nn.Parameter(torch.zeros(1))
        self.cmyw = nn.Parameter(torch.zeros(1))
        self.cmyd = nn.Parameter(torch.zeros(1))
        self.cmyddd = nn.Parameter(torch.zeros(1))
        self.cmz0 = nn.Parameter(torch.zeros(1))
        self.cmzw = nn.Parameter(torch.zeros(1))
        self.cmzd = nn.Parameter(torch.zeros(1))
        self.cmzwd = nn.Parameter(torch.zeros(1))
        self.d0 = nn.Parameter(torch.zeros(2, 1))

    def forward(self, x):
        vx, vy, vz, Ox, Oy, Oz,  w1, w2, w1d, w2d, d1, d2  =  x

        # state
        vxavx, vyavy, vzavz = vx.abs()*vx, vy.abs()*vy, vz.abs()*vz
        O = torch.vstack((Ox, Oy, Oz))

        # input combinations
        ww = torch.stack([w1*w1, w2*w2])
        wwDd = ww * torch.tan( (torch.stack([d1, d2]) - self.d0) )
        d2abs = torch.tan( d1 ).abs() + torch.tan( d2 ).abs()

        fx = self.cx0  + self.cxw  * (ww[0] + ww[1]) + self.cxd * (wwDd[0] - wwDd[1]) + self.cxv * vxavx
        fy = self.cy0  + torch.zeros(ww.shape[1])  + self.cyv * vyavy
        fz = self.cz0  + self.czw  * (ww[0] + ww[1]) + self.czd2 * d2abs
        mx = self.cmx0 + self.cmxw * (ww[0] - ww[1])
        my = self.cmy0 + self.cmyw * (ww[0] - ww[1]) + self.cmyd * (wwDd[0] - wwDd[1])
        mz = self.cmz0 + self.cmzw * (ww[0] - ww[1]) + self.cmzd * (wwDd[0] + wwDd[1]) + self.cmzwd * (w1d - w2d)

        F = torch.stack([fx, fy, fz])
        M = torch.stack([mx, my, mz])

        dOdt = M # if known: - O.cross(self.I * O, dim=0)

        a = F + dOdt.cross(self.r, dim=0) + O.cross(O.cross(self.r, dim=0), dim=0)

        return torch.concat([a, dOdt], dim=0)

# device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = Tailsitter()

log = IndiflightLog("/mnt/data/WorkData/BlackboxLogs/MIRROR_DarkO/LOG00043.BFL", resetTime=True)

in_flight = (log.data['timeS'] > 10) & (log.data['timeS'] < 50)
data = log.data[in_flight]
t = data['timeS']

v_true  = data[[f'extVel[{i}]' for i in range(3)]].to_numpy(dtype=np.float32).T
q_true  = data[[f'extAtt[{i}]' for i in range(4)]].to_numpy(dtype=np.float32).T / 8.128
a_true  = data[[f'accADCafterRpm[{i}]' for i in range(3)]].to_numpy(dtype=np.float32).T
O_true  = data[[f'gyroADCafterRpm[{i}]' for i in range(3)]].to_numpy(dtype=np.float32).T
w_true  = data[[f'omegaUnfiltered[{i}]' for i in range(2)]].to_numpy(dtype=np.float32).T
w_true /= 1000
d_true  = data[[f'servo_feedback[{i}]' for i in range(2)]] .to_numpy(dtype=np.float32).T
d_true /= 100
d_true *= np.pi / 180

w_torch = torch.tensor(w_true)
dw_torch = 0.5*(w_torch.roll(shifts=-1, dims=1) - w_torch.roll(shifts=1, dims=1))
dw_torch /= t.diff().mean()
dw_torch /= 20.

v_body_true  = quaternion_rotate(q_true, v_true)

x = torch.vstack([
    torch.tensor(v_body_true),
    torch.tensor(O_true),
    w_torch,
    dw_torch,
    torch.tensor(d_true),
])

a_torch = torch.tensor(a_true)
O_torch = torch.tensor(O_true)
dO_torch = 0.5*(O_torch.roll(shifts=-1, dims=1) - O_torch.roll(shifts=1, dims=1))
dO_torch /= t.diff().mean()


y_true = torch.concat((a_torch, dO_torch))
y_true = y_true[:, 1:-1]


optimizer = optim.LBFGS(model.parameters(), lr=2e-1)

def weighted_mse_loss(pred, true, weight):
    return torch.mean(weight * (pred - true) ** 2)

for epoch in range(100):
    def closure():
        optimizer.zero_grad()
        y_pred = model(x[:, 1:-1])
        loss = weighted_mse_loss(y_pred, y_true, torch.tensor([[0.05, 0.05, 0.05, 0.01, 0.01, 0.01]]).T)
        loss.backward()
        return loss

    optimizer.step(closure)

    if epoch % 10 == 0:
        print(f'Epoch {epoch}: Loss = {closure().item():.5f}')


fig, axs = plt.subplots(6, 2, sharex=True)
for i in range(6):
    axs[i, 0].plot(t[1:-1], y_true[i], '-')
    axs[i, 0].plot(t[1:-1], model.forward(x[:, 1:-1]).detach()[i], '--')

axs[0, 1].plot(t[1:-1], v_true[:, 1:-1].T)