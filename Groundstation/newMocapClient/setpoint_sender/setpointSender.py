#!/usr/bin/env python3

import argparse
import struct
import socket

"""
Format:

!: big-endian (network)
B: unigned char 1 byte
h: signed short 2 byte
H: unsigned short 2 byte
i: signed int 4 byte
I: unsigned int 4 byte
f: float 4 byte
"""

POS_SETPOINT = {
    'time_ms': 0,
    'ned_x': 0.,
    'ned_y': 0.,
    'ned_z': 0.,
    'ned_xd': 0.,
    'ned_yd': 0.,
    'ned_zd': 0.,
    'yaw': 0.,
}

POS_SETPOINT_format = '!Ifffffff'

# parse args
parser = argparse.ArgumentParser()
parser.add_argument('--host', required=False, default="10.0.0.1", type=str)
parser.add_argument('--port', required=False, default=5006, type=int)
parser.add_argument('--pos', required=True, nargs=3, help="NED frame", type=float)
parser.add_argument('--vel', required=False, nargs=3, default=[0., 0., 0.], type=float)
parser.add_argument('--yaw', required=True, help="Degrees in NED", type=float)
args = parser.parse_args()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

msg = POS_SETPOINT.copy()
msg['time_ms'] = int(0)
msg['ned_x'] = args.pos[0]
msg['ned_y'] = args.pos[1]
msg['ned_z'] = args.pos[2]
msg['ned_xd'] = args.vel[0]
msg['ned_yd'] = args.vel[1]
msg['ned_zd'] = args.vel[2]
msg['yaw'] = args.yaw

msg_packed = struct.pack(POS_SETPOINT_format, *msg.values())
sock.sendto(msg_packed, (args.host, args.port))

print(f"sent message {msg}")
