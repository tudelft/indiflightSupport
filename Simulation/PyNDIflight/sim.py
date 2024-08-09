from .interfaces import visData

class Sim():
    def __init__(self, uav, imu=None, mocap=None, hil=None, sil=None):
        self.uav = uav
        self.imu = imu
        self.mocap = mocap
        self.hil = hil
        self.sil = sil
        self.t = 0.
        self.i = 0

        # spawn craft
        visData.spawn(self.uav)

    def tick(self, dt):
        if not (self.i % 2):
            self.hil.receive() if self.hil else None

        self.sil.receive() if self.sil else None

        self.uav.tick(dt)

        if self.sil:
            self.sil.sendImuAndMotor()
            self.sil.tick(dt) # ticks the indiflight controller

        if not (self.i % 2): # 1kHz
            self.imu.update() if self.imu else None
            self.hil.send() if self.hil else None
        if not (self.i % 20): # 100 Hz
            visData.update(self.uav)
        if not (self.i % 100): # 20 Hz
            self.mocap.update() if self.mocap else None
            self.sil.sendMocap() if self.sil else None

        self.t += dt
        self.i += 1