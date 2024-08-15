#!/bin/bash
. ./scripts/source_ros_and_msgs.sh
cd build && ./mocap2ros2px4 "$@"