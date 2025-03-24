#!/bin/bash
set -e

# Setup development environment
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
source /home/dev/ros_ws/install/setup.bash

# launch wall_follow which also launches AutoDRIVE Devkit with GUI and Rviz
ros2 launch wall_follow wall_follow_launch.xml