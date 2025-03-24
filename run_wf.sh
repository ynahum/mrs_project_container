#!/bin/bash
set -e

# Setup development environment
source /home/dev/ros_ws/src/common.sh

# launch wall_follow which also launches AutoDRIVE Devkit with GUI and Rviz
ros2 launch wall_follow wall_follow_launch.xml