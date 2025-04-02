#!/bin/bash
set -e

source /home/dev/ros_ws/src/setup.sh

# launch nav2 and Rviz
ros2 launch my_car nav2_rviz_bridge.launch.xml

# launch AutoDRIVE Devkit with GUI and Rviz
#ros2 launch my_car rviz_bridge.launch.xml

# also slam
#ros2 launch my_car slam_rviz_bridge.launch.xml

# wall follow launch
#ros2 launch my_car wf.launch.xml
