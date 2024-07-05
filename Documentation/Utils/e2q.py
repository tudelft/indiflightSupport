#!/usr/bin/env python3
"""Convert ZYX Euler Angles in quaternion"""

from scipy.spatial.transform import Rotation as R
from argparse import ArgumentParser

parser = ArgumentParser(description=__doc__)
parser.add_argument("roll", help="degrees")
parser.add_argument("pitch", help="degrees")
parser.add_argument("yaw", help="degrees")

args = parser.parse_args()
eulers = [args.yaw, args.pitch, args.roll]

q = R.from_euler('ZYX', eulers, degrees=True).as_quat()
print(f"\
q.w: {q[3]:8.3f}\n\
q.x: {q[0]:8.3f}\n\
q.y: {q[1]:8.3f}\n\
q.z: {q[2]:8.3f}\
        ")

