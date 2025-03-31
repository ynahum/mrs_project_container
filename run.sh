#!/bin/bash
set -e

source /home/dev/ros_ws/src/scripts/setup.sh

# launch AutoDRIVE Devkit with GUI and Rviz
#ros2 launch my_car rviz_sim_bringup.launch.xml

# also slam
#ros2 launch my_car slam_rviz_sim_bringup.launch.xml

# launch AutoDRIVE Devkit with GUI, nav2 and Rviz
ros2 launch my_car rviz_nav2_sim_bringup.launch.xml

# wall follow launch
#ros2 launch my_car wf_slam_rviz_sim_bringup.launch.xml
