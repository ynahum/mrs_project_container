#!/bin/bash
set -e


cd /home/dev/ros_ws/
colcon build
source /home/dev/ros_ws/install/setup.bash
cd src/