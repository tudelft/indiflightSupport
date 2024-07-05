#!/usr/bin/env python3
import numpy as np
from scipy.spatial.transform import Rotation

N_ACT = 6
G = 9.81 # can be set to 1 (direction only) or 9.81 (correctly scaled alloc)
W = np.eye(N_ACT) # actuator weighing matrix in hover

# fast inverse square root aka Quake 3 algorithm
# https://www.youtube.com/watch?v=p8u_k2LIZyo
import struct
def fisqrt(x):
    # mostly just for fun.. implementing this is c may give faster power iteration performance
    # pack and unpack to simulate reinterpret_cast using float precision
    ix = struct.unpack('<I', struct.pack('<f', x))[0]
    iy = 0x5F3759DF - (ix >> 1)
    y = struct.unpack('<f', struct.pack('<I', iy))[0]

    # newton iteration
    y *= 1.5 - (0.5 * x * y * y)
    #y *= 1.5 - (0.5 * x * y * y)
    return y


#%% problem data

# standard effectiveness matrix of a quadrotor
BfGT = np.zeros((3, N_ACT))
BfGT[2, :] = -1

BrGT = np.zeros((3, N_ACT))
#BrGT[0, :] = [-1, -1, 1, 1, 2, 3]
#BrGT[1, :] = [-1, 1, -1, 1, 2, 3]
#BrGT[2, :] = [1, -1, -1, 1, 2, 3]
BrGT[0, :] = [-1, -1, 1, 1, 2, 3]
BrGT[1, :] = [-1, 1, -1, 1, 2, 3]
BrGT[2, :] = [1, -1, -1, 1, 2, 3]

#AT P = QR
#AT = QR PT
#A = P RT QT

# rotate randomly
#R = np.array([[-0.4891257 ,  0.26580112,  0.83072608],
#              [ 0.78974489, -0.2693009 ,  0.55116244],
#              [ 0.37021487,  0.92564939, -0.07819308]])
#np.random.seed(42)
R = Rotation.random().as_matrix()
Bf = R @ BfGT + 0.25*(np.random.random((3,6)) - 0.5)
Br = R @ BrGT + 0.25*(np.random.random((3,6)) - 0.5)

#%% solve most efficient hover distribution and attitude

# get rotation nullspace
Q, _ = np.linalg.qr(Br.T, 'complete')
Nr = Q[:, 3:]

# get eigenpair
H = Nr.T @ W @ Nr
A = Nr.T @ Bf.T @ Bf @ Nr
L, V = np.linalg.eig(np.linalg.inv(H) @ A)
vTrue = V[:, np.abs(L).argmax()] # abs probably not necessary since H and A are pos def? lets keep it to guard against spurious complex numbers

#%% now do the same with power iteration, to demonstrate that it works
# power iteration, let's do two per estimation loop iteration (use warmstarting eventually!)
v = np.random.random(N_ACT - 3)
#v = np.array([0.5, 1., 1.])
i = 3
while i > 0:
    Av = A @ v
    Av2 = Av.T @ Av
    if Av2 < 1e-7:
        # almost orthogonal to largest eigenvector, reset
        v = np.random.random(N_ACT - 3)
        continue
    #v = Av * fisqrt(Av2)
    v = Av * 1/np.sqrt(Av2)
    i -= 1

# adjust eigenpairs for solutions
#for i, v in enumerate(V.T):
for i in range(1):
    scale = np.sqrt( G**2 / ( v.T @ A @ v ) )
    eta = scale * v

    u = Nr @ eta
    if sum(u) < 0:
        # is this nice/sufficient?
        u = -u

    print()
    print(f"--- Solution {i} ---")
    print(f"True eig(A): {vTrue}")
    print(f"Power iteration eig(A) with fisqrt: {v}")
    print(f"Hover allocation: {u}")
    print(f"Hover rotational acceleration: {Br @ u}")
    print(f"Hover thrust direction: {Bf @ u}")
    print(f"Local z from hover thrust: {(-Bf @ u) / np.linalg.norm(Bf @ u)}")
    print(f"Rotation times local z: {R @ np.array([0., 0., 1.])}")
    print(f"sqrt(u.T (Bf.T Bf) u): {np.sqrt(u.T @ Bf.T @ Bf @ u)}")
