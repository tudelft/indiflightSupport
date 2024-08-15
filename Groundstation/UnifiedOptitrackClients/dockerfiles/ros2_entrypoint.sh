#!/bin/bash
. /opt/ros/humble/setup.bash
cd build && ./mocap2ros2 "$@"