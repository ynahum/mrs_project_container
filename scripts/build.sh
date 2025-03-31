#!/bin/bash
set -e


cd /home/dev/ros_ws/
colcon build --packages-select my_car
cd src/