#!/bin/bash
set -e

# Setup development environment
source /home/dev/ros_ws/src/setup.sh

# Launch AutoDRIVE Devkit with GUI
ros2 launch autodrive_f1tenth simulator_bringup_rviz.launch.py

