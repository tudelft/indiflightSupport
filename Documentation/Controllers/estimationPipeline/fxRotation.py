#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as R
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument( "roll" )
parser.add_argument( "pitch" )
parser.add_argument( "yaw" )
args = parser.parse_args()

eulersZYX = [args.yaw, args.pitch, args.roll]

G1f = np.array([
        [0,0,0,0],
        [0,0,0,0],
        [-1050,-1050,-1050,-1050],
    ], dtype=float)

G1r = np.array([
        [-4000, -4000, 4000, 4000],
        [-2600, 2600, -2600, 2600],
        [-510, 510, 510, -510],
    ], dtype=float)

G2 = np.array([
        [0,0,0,0],
        [0,0,0,0],
        [-100,100,100,-100],
    ], dtype=float)

rot = R.from_euler('zyx', eulersZYX, degrees=True)
rotM = rot.as_matrix()

G1frot = rotM @ G1f
G1rrot = rotM @ G1r
G2rot = rotM @ G2

print("Rotation matrix:")
print(rotM)

print("")
print("G1f")
for row in G1frot.astype(int):
    print(', '.join(map(str, row)))

print("")
print("G1r")
for row in G1rrot.astype(int):
    print(', '.join(map(str, row)))

print("")
print("G2")
for row in G2rot.astype(int):
    print(', '.join(map(str, row)))
