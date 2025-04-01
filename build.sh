#!/bin/bash
set -e


cd /home/dev/ros_ws/
colcon build
#colcon build --packages-select my_car
cd -