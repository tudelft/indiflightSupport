
import numpy as np
from scipy.spatial.transform import rotation as R
from scipy.constants import g as GRAVITY

from helpers import (
    cross,
    quatRotate,
    quaternionDerivative,
    angularRateDerivative,
    rotatingMassTorques,
    motorModel
    )

class Rotor:
    def __init__(self, r=[0., 0., 0.], axis=[0., 0., -1.], wmax=4000., Tmax=4., kESC=0., cm=0.01, tau=0.03, Izz=1e-6, dir='rh'):
        self.r = np.asarray(r, dtype=np.float32)
        self.axis = np.asarray(axis, dtype=np.float32)
        self.wmax = wmax
        self.Tmax = Tmax
        self.kESC = kESC
        self.k = Tmax / self.wmax / self.wmax
        self.cm = cm
        self.tau = tau
        self.Izz = Izz
        if dir in ['rh', 'lh']:
            self.dir = -1. if dir=='lh' else +1.
        else:
            raise ValueError("dir must be one of 'rh', or 'lh'!")

        self.F = np.array([0., 0., 0.], dtype=np.float32)
        self.M = np.array([0., 0., 0.], dtype=np.float32)
        self.w = 0.

    def step(self, u, Omega, dt):
        wDot = motorModel( u, self.kESC, self.wmax, self.w, self.tau )
        self.w += dt * wDot
        self.F = self.axis * self.k * self.w*self.w
        self.M = cross(self.r, self.F)
        self.M -= self.dir * self.cm * self.F
        self.M -= rotatingMassTorques(self.Izz, self.axis*self.dir, self.w, wDot, Omega)

class MultiRotor:
    def __init__(self):
        self.rotors = []
        self.m = 1.
        self.I = np.eye(3, dtype=np.float32)
        self.Iinv = np.eye(3, dtype=np.float32)
        self.xI = np.array([0., 0., 0.], dtype=np.float32)
        self.vI = np.array([0., 0., 0.], dtype=np.float32)
        self.fspB = np.array([0., 0., 0.], dtype=np.float32)
        self.q = np.array([1., 0., 0., 0.], dtype=np.float32)
        self.wDotB = np.array([0., 0., 0.], dtype=np.float32)
        self.wB = np.array([0., 0., 0.], dtype=np.float32)

    def __repr__(self):
        qWrong = np.zeros_like(self.q)
        qWrong[3] = self.q[0]
        qWrong[:3] = self.q[1:]
        rot = R.from_quat(qWrong)
        eulers = rot.as_euler('ZYX', degrees=True)
        return f"MultiRotor( n={len(self.rotors)}, x={self.xI}m, v={self.vI}m/s, roll={eulers[2]}deg, pitch={eulers[1]}deg, yaw={eulers[0]}deg )"

    def setInertia(self, m, I):
        self.m = m
        self.I = I.astype(np.float32)
        self.Iinv = np.linalg.inv(I).astype(np.float32)

    def addRotor(self, rotor):
        self.rotors.append(rotor)
        self.n = len(self.rotors)
        self.rotorVelocity = np.zeros(self.n, np.float32)
        self.inputs = np.zeros(self.n, np.float32)

    def setPose(self, x=[0., 0., 0.], q=[1., 0., 0., 0.]):
        self.xI[:] = np.asarray(x, dtype=np.float32)
        self.q[:] = np.asarray(q, dtype=np.float32)

    def setTwist(self, v=[0., 0., 0.], w=[0., 0., 0.]):
        self.vI[:] = np.asarray(v, dtype=np.float32)
        self.wB[:] = np.asarray(w, dtype=np.float32)

    def step(self, dt):
        F = np.zeros(3, dtype=np.float32)
        M = np.zeros(3, dtype=np.float32)
        for i, (u, rotor) in enumerate(zip(self.inputs, self.rotors)):
            rotor.step(u, self.wB, dt)
            self.rotorVelocity[i] = rotor.w
            F += rotor.F
            M += rotor.M

        if self.xI[2] > 0.:
            # handle ground contact
            qInv = self.q.copy()
            qInv[0] *= -1.
            down = self.vI[2] > 0.
            F += (1000 if down else 1000)  * self.m * quatRotate( qInv, np.array([0., 0., -1.], dtype=np.float32) * self.xI )
            F += (100  if down else 1) * self.m * quatRotate( qInv, -self.vI )
            M += 1000 * self.I @ ( np.sign(qInv[0]) * qInv[1:] )
            M += 100 * self.I @ -self.wB

        self.wDotB[:] = angularRateDerivative( self.wB, M, self.I, self.Iinv )
        qDot = quaternionDerivative( self.q, self.wB )
        self.fspB[:] = F / self.m
        vDot = quatRotate( self.q, self.fspB )  +  np.array([0., 0., GRAVITY])
        xDot = self.vI

        self.wB += dt * self.wDotB
        self.q += dt * qDot
        self.q /= np.linalg.norm(self.q)
        self.vI += dt * vDot
        self.xI += dt * xDot

class IMU:
    def __init__(self, uav, r=[0., 0., 0.], qBody=[1., 0., 0., 0.], accBias=[0., 0., 0.], accStd=0.0, gyroBias=[0., 0., 0.], gyroStd=0.0):
        self.uav = uav

        self.r = np.asarray(r, dtype=np.float32)
        self.qInv = np.asarray(qBody, dtype=np.float32)
        self.qInv[0] *= -1.

        self.acc = np.zeros(3, dtype=np.float32)
        self.accBias = np.asarray(accBias, dtype=np.float32)
        self.accStd = accStd

        self.gyro = np.zeros(3, dtype=np.float32)
        self.gyroBias = np.asarray(gyroBias, dtype=np.float32)
        self.gyroStd = gyroStd

    def update(self):
        accAtImu = self.uav.fspB + cross(self.uav.wDotB, self.r) + cross(self.uav.wB, cross(self.uav.wB, self.r))
        self.acc[:] = quatRotate(self.qInv, accAtImu) + np.random.normal(self.accBias, self.accStd)
        self.gyro[:] = quatRotate(self.qInv, self.uav.wB) + np.random.normal(self.gyroBias, self.gyroStd)
