#!/bin/bash
set -e

# Setup development environment
source /home/dev/ros_ws/src/scripts/setup.sh

# Launch AutoDRIVE Devkit with GUI
ros2 launch autodrive_f1tenth simulator_bringup_rviz.launch.py

# Launch AutoDRIVE Devkit Headless
# ros2 launch autodrive_f1tenth simulator_bringup_headless.launch.py

#./run.sh
