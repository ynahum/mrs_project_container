#!/bin/bash
set -e

source /home/dev/ros_ws/src/setup.sh

# launch wall_follow which also launches AutoDRIVE Devkit with GUI and Rviz
ros2 launch wall_follow sim_bridge_plus_rviz_plus_slam.launch.xml
#ros2 launch wall_follow wall_follow_launch.xml