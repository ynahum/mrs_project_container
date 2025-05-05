#!/bin/bash
set -e


cd /home/dev/ros_ws/
#colcon build
colcon build --packages-select my_car
colcon build --packages-select nav2_bt_navigator
#colcon build --packages-select nav2_mppi_controller --executor sequential  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

cd - > /dev/null