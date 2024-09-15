import numpy as np
import quaternion

import matplotlib.pyplot as plt

from scipy.spatial.transform import Rotation as R

#%% toplevel

f = 100.
dt = 1. / f
T = 10.
N = int(f*T + 0.5)
accNoise = 1.
velNoise = 1 * 0.3
rot = np.array([200., -100., -50.]) / 180. * np.pi
#rot = np.array([0., 0., 0.]) / 180. * np.pi
G = 9.81

#%% generate data

def quatDot(q, O):
    Ox, Oy, Oz = O
    qDot = np.array([
        .5 * (-Ox*q.x - Oy*q.y - Oz*q.z),
        .5 * ( Ox*q.w + Oz*q.y - Oy*q.z),
        .5 * ( Oy*q.w - Oz*q.x + Ox*q.z),
        .5 * ( Oz*q.w + Oy*q.x - Ox*q.y),
    ])
    return qDot

Rc = 1.
Vc = 1.
Oc = Vc / Rc
ac = Vc*Vc / Rc
pNED_0 = np.array([0., 1., 0.])
vNED_0 = np.array([1., 0., 0.])
aNED_0 = np.array([0., 0., 0.])
q_0 = quaternion.quaternion(1., 0., 0., 0.)
yaw_0 = 180. * np.pi / 180.
q_offset = quaternion.from_rotation_vector([0., 0., yaw_0])

pNED  = pNED_0.copy()
vNED  = vNED_0.copy()
aNED  = aNED_0.copy()
q     = q_0.copy()
qMeas = q_offset * q_0

pNED_hist = np.zeros((N+1, 3)); pNED_hist[0] = pNED_0.copy()
vNED_hist = np.zeros((N+1, 3)); vNED_hist[0] = vNED_0.copy()
aNED_hist = np.zeros((N+1, 3)); aNED_hist[0] = aNED_0.copy()
acc_hist = np.zeros((N+1, 3)); acc_hist[0] = quaternion.rotate_vectors(q.inverse(), aNED + np.array([0., 0., -G]))
q_hist = [q.copy()]
qMeas_hist = [qMeas.copy()]
for i in range(N):
    aNED = -ac * pNED
    vNED += dt * aNED
    pNED += dt * vNED

    qDot = quatDot(q, rot)
    q.w += dt * qDot[0]
    q.x += dt * qDot[1]
    q.y += dt * qDot[2]
    q.z += dt * qDot[3]
    q = q.normalized()
    qMeas = q_offset * q

    acc = quaternion.rotate_vectors(q.inverse(), aNED + np.array([0., 0., -G]))

    acc_hist[i+1] = acc.copy()

    qMeas_hist.append(qMeas.copy())
    q_hist.append(q.copy())

    aNED_hist[i+1] = aNED.copy()
    vNED_hist[i+1] = vNED.copy()
    pNED_hist[i+1] = pNED.copy()

xkk = np.array([0., 0., 0.])
Pkk = 1000. * np.diag([1., 1., 10.])

y_hist = np.zeros((N+1, 2))
xkk_hist = np.zeros((N+1, 3))
Pkk_hist = np.zeros((N+1, 3, 3))
xkk_hist[0] = xkk.copy()
Pkk_hist[0] = Pkk.copy()
acc_noise = acc_hist + np.random.normal(0, accNoise, (N+1, 3))
vNED_noise = vNED_hist + np.random.normal(0, velNoise, (N+1, 3))
for i in range(N):
    aqMeas = quaternion.rotate_vectors(qMeas_hist[i], acc_hist[i])
    xiEst = xkk[2]
    xk1k = xkk + dt * np.array([
        [np.cos(xiEst), +np.sin(xiEst)],
        [-np.sin(xiEst),  np.cos(xiEst)],
        [0., 0.],
    ]) @ aqMeas[:2]

    # kalman gain and Pkk estimation is completely meaningless for the very
    # nonlinear influence of PsiEst, so we handle this separately

    #xk1k[:2] += 0.01 * ( np.random.random() - 0.5 )
    F = np.eye(3)
    #F[0, 2] = dt * ( -aqMeas[0] * np.sin(xiEst) + aqMeas[1] * np.cos(xiEst) )
    #F[1, 2] = dt * ( -aqMeas[0] * np.cos(xiEst) - aqMeas[1] * np.sin(xiEst) )
    H = np.eye(2, 3)
    Q = np.diag([1., 1., 0.1])
    Rk = 10. * np.diag([1., 1.])

    Pk1k = F @ Pkk @ F.T + Q
    z = vNED_noise[i][:2]
    zhat = H @ xk1k
    y = z - zhat
    S = H @ Pk1k @ H.T + Rk
    K = Pk1k @ H.T @ np.linalg.inv(S)
    xkk = xk1k + K @ y
    deltaXi = np.arctan2(z[1], z[0]) - np.arctan2(zhat[1], zhat[0])
    xkk[2] -= 0.1*np.linalg.norm(y)**2*deltaXi
    Pkk = Pk1k - K @ H @ Pk1k

    y_hist[i+1] = y.copy()
    xkk_hist[i+1] = xkk.copy()
    Pkk_hist[i+1] = Pkk.copy()

print(yaw_0)
print(xkk)

plt.close()
f, axs = plt.subplots(5, 1)
axs[0].plot(np.linspace(0, T, N+1), acc_noise[:, :2])
axs[0].plot(np.linspace(0, T, N+1), acc_hist[:, :2])
axs[0].set_ylabel("Acceleration")

axs[1].plot(np.linspace(0, T, N+1), vNED_noise[:, :2])
axs[1].plot(np.linspace(0, T, N+1), vNED_hist[:, :2])
axs[1].set_ylabel("Velocity")

#axs[2].plot(np.linspace(0, T, N+1), vNED_hist[:, :2])
axs[2].plot(np.linspace(0, T, N+1), yaw_0*np.ones(N+1))
axs[2].plot(np.linspace(0, T, N+1), xkk_hist[:, :])
axs[2].set_ylabel("State and yaw GT")

axs[3].plot(np.linspace(0, T, N+1), y_hist)
axs[3].set_ylabel("Innovation")

axs[4].plot(np.linspace(0, T, N+1), np.array([np.diag(P) for P in Pkk_hist]))
axs[4].set_yscale('log')
axs[4].set_ylabel("Covariance")
axs[4].set_xlabel("time [s]")

f.show()
