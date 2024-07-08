
import numpy as np

#%% serve the visualization js app

import flask as fl
import json
class VisData():
    def __init__(self):
        self.x = np.zeros(3, dtype=np.float64)
        self.q = np.zeros(4, dtype=np.float64)
        self.q[0] = 1.
        self.inputs = np.zeros(4, dtype=np.float64)

    def update(self, uav):
        self.x[:] = uav.xI.round(4)
        self.q[:] = uav.q.round(4)
        self.inputs = uav.inputs.astype(np.float64).round(4)

visData = VisData()
visApp = fl.Flask(__name__, static_url_path='/static')

@visApp.route("/")
def hello_world():
    return fl.render_template("index.html")

@visApp.route("/pose")
def pose():
    arr = []
    pos = list(visData.x.round(4))
    quat = list(visData.q.round(4))
    ctl = list(visData.inputs)
    arr.append({'id': 0, 'type': 0, 'pos': pos, 'quat': quat, 'ctl': ctl})
    return json.dumps(arr)

# dont spam the console
import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.WARNING)


#%% mocap server

import socket
import struct

class Mocap:
    # emulate motion capture system and send data via UDP
    def __init__(self, uav, ip="127.0.0.1", port=5005):
        self.uav = uav
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (ip, port)
        self.fmt = '<I Qfff ffff QIfff fff'
        self.x = np.zeros(3, dtype=np.float32)
        self.v = np.zeros(3, dtype=np.float32)
        self.q = np.zeros(4, dtype=np.float32)
        self.q[0] = 1.
        self.w = np.zeros(3, dtype=np.float32)

    def update(self):
        # convert NED to ENU
        self.x[0] = self.uav.xI[1]
        self.x[1] = self.uav.xI[0]  
        self.x[2] = -self.uav.xI[2] 
        self.v[0] = self.uav.vI[1]  
        self.v[1] = self.uav.vI[0]  
        self.v[2] = -self.uav.vI[2] 
        self.q[3] = self.uav.q[0] # also reverse order...
        self.q[0] = self.uav.q[2]
        self.q[1] = self.uav.q[1]
        self.q[2] = -self.uav.q[3]
        self.w[0] = 666.

        msg_packed = struct.pack(self.fmt, 0, 0, *self.x, *self.q, 0, 0, *self.v, *self.w)
        self.sock.sendto(msg_packed, self.addr)


#%% hardware in the loop interface

import serial
import struct

#import numba as nb
#
#@nb.njit#("u1(u1[::1], u1[::1])")
#def parse_input(bytes, msg, msgFull, msg_pointer, parse_state, escape_state):
#    numBytes = 0
#    for byte in bytes:
#        if byte == IndiflightHIL.STX:
#            msgFull[:] = msg.copy()
#            numBytes = msg_pointer[0]
#            parse_state[0] = IndiflightHIL.MSG
#            msg_pointer[0] = 0
#            continue
#
#        if parse_state[0] == IndiflightHIL.IDLE:
#            continue
#
#        if escape_state[0] == True:
#            if byte == IndiflightHIL.ESC_ESC:
#                msg[msg_pointer[0]] = IndiflightHIL.ESC
#            elif byte == IndiflightHIL.STX_ESC:
#                msg[msg_pointer[0]] = IndiflightHIL.ESC
#            else:
#                parse_state[0] = IndiflightHIL.IDLE
#                escape_state[0] = False
#                continue
#
#            msg_pointer[0] += 1
#            escape_state[0] = False
#
#        elif byte == IndiflightHIL.ESC:
#            escape_state[0] = True
#        else:
#            msg[msg_pointer[0]] = byte
#            msg_pointer += 1
#
#    return numBytes

class IndiflightHIL:
    # serial interface with an INDIflight controller compiled with HIL_BUILD
    HIL_TO_DEGS = 0.1
    HIL_TO_G = 0.001
    HIL_TO_RPM = 10.
    HIL_IN_ID = 5
    HIL_IN_PAYLOAD_LEN = 26
    HIL_OUT_ID = 6
    HIL_OUT_PAYLOAD_LEN = 12
    MOTOR_TO_HIL = 32767
    STX = 0xFE
    ESC = 0x01
    STX_ESC = 0x02
    ESC_ESC = 0x03
    MAX_PACKET_LEN = 256
    IDLE = 0
    MSG = 1

    def __init__(self, uav, imu, device='/dev/ttyUSB0', baud=921600):
        self.uav = uav
        self.imu = imu
        self.ser = serial.Serial(port=device, baudrate=baud, timeout=0.0001)
        self.ser.set_low_latency_mode(True) # only works on linux, i think
        self.fmtSend = '< B I hhh hhh h hhhh'
        self.fmtReceive = '< B I hhhh B' # checksum byte at the end
        self.len_fmtReceive = struct.calcsize(self.fmtReceive)
        self.parse_state = self.IDLE
        self.escape_state = False
        self.u_command = np.zeros(4, dtype=np.float32)
        #self.bytes = np.zeros( self.MAX_PACKET_LEN, dtype=np.uint8)
        self.bytes = bytearray()
        self.len_bytes = 0
        self.num_msgs = 0

    def send(self):
        w = np.zeros(4, dtype=np.float32)
        if self.uav.n > 4:
            w[:] = self.uav.rotorVelocity[:4]
        else:
            w[:self.uav.n] = self.uav.rotorVelocity

        msg_packed = struct.pack(self.fmtSend,
                                 self.HIL_IN_ID,
                                 #self.HIL_IN_PAYLOAD_LEN,
                                 0, # timestamp
                                 *(self.imu.gyro * 180 / np.pi / self.HIL_TO_DEGS).astype(np.int16),
                                 *(self.imu.acc / 9.81 / self.HIL_TO_G).astype(np.int16),
                                 0, # baro
                                 *(w * 60 / (2*3.1415) / self.HIL_TO_RPM).astype(np.int16),
                                 )

        checksum = msg_packed[0]
        for byte in msg_packed:
            checksum ^= byte

        msg_packed.append(checksum)

        msg_escaped = bytearray()
        msg_escaped.append(self.STX)
        for byte in msg_packed:
            if byte == self.STX:
                msg_escaped.append(self.ESC)
                msg_escaped.append(self.STX_ESC)
            elif byte == self.ESC:
                msg_escaped.append(self.ESC)
                msg_escaped.append(self.ESC_ESC)
            else:
                msg_escaped.append(byte)

        self.ser.write(msg_escaped)
        #self.ser.write(msg_packed)

    def receive(self):
        data = self.ser.read(self.MAX_PACKET_LEN*2)
        for byte in data:
            if byte == self.STX:
                self.parse_state = self.MSG
                self.bytes = bytearray()
                self.len_bytes = 0
                continue

            if self.parse_state == self.IDLE:
                continue

            if self.escape_state == True:
                if byte == self.ESC_ESC:
                    self.bytes.append(self.ESC)
                elif byte == self.STX_ESC:
                    self.bytes.append(self.STX)
                else:
                    self.parse_state = self.IDLE
                    self.escape_state = False
                    continue

                self.len_bytes += 1
                self.escape_state = False

            elif byte == self.ESC:
                self.escape_state = True
            else:
                self.bytes.append(byte)
                self.len_bytes += 1

            if self.len_bytes == self.len_fmtReceive:
                self.parse_state = self.IDLE # definitely return to idle, but checksum decides if we accept the package

                checksum = self.bytes[0]
                for byte in self.bytes[1:-1]:
                    checksum ^= byte

                if checksum != self.bytes[-1]:
                    continue

                self.num_msgs += 1
                msg_unpacked = struct.unpack(self.fmtReceive, self.bytes)
                if msg_unpacked[0] == self.HIL_OUT_ID:
                    self.uav.inputs[0] = msg_unpacked[2] / self.MOTOR_TO_HIL
                    self.uav.inputs[1] = msg_unpacked[3] / self.MOTOR_TO_HIL
                    self.uav.inputs[2] = msg_unpacked[4] / self.MOTOR_TO_HIL
                    self.uav.inputs[3] = msg_unpacked[5] / self.MOTOR_TO_HIL
                    self.uav.inputs = np.clip(self.uav.inputs, 0., 1.)

    #def receive(self):
    #    data = self.ser.read(self.MAX_PACKET_LEN*2)

    #    for byte in data:
    #        if byte == self.STX:
    #            self.parse_state = self.MSG
    #            self.len_bytes = 0
    #            continue

    #        if self.parse_state == self.IDLE:
    #            continue

    #        if self.escape_state == True:
    #            if byte == self.ESC_ESC:
    #                self.bytes[self.len_bytes] = self.ESC
    #            elif byte == self.STX_ESC:
    #                self.bytes[self.len_bytes] = self.STX
    #            else:
    #                self.parse_state = self.IDLE
    #                self.escape_state = False
    #                continue

    #            self.len_bytes += 1
    #            self.escape_state = False

    #        elif byte == self.ESC:
    #            self.escape_state = True
    #        else:
    #            self.bytes[self.len_bytes] = byte
    #            self.len_bytes += 1

    #        if self.len_bytes == self.len_fmtReceive:
    #            self.num_msgs += 1
    #            self.parse_state = self.IDLE
    #            msg_unpacked = struct.unpack(self.fmtReceive, self.bytes[:self.len_bytes])
    #            if msg_unpacked[0] == self.HIL_OUT_ID:
    #                self.uav.inputs[0] = msg_unpacked[2] / self.MOTOR_TO_HIL
    #                self.uav.inputs[1] = msg_unpacked[3] / self.MOTOR_TO_HIL
    #                self.uav.inputs[2] = msg_unpacked[4] / self.MOTOR_TO_HIL
    #                self.uav.inputs[3] = msg_unpacked[5] / self.MOTOR_TO_HIL
    #                self.uav.inputs = np.clip(self.uav.inputs, 0., 1.)