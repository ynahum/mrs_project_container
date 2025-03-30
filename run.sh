#!/bin/bash
set -e

source /home/dev/ros_ws/src/setup.sh

# launch AutoDRIVE Devkit with GUI and Rviz
ros2 launch my_car sim_bringup_rviz.launch.xml

# also slam
#ros2 launch my_car sim_bringup_rviz_slam.launch.xml

# wall follow launch
#ros2 launch wall_follow wall_follow_launch.xml
