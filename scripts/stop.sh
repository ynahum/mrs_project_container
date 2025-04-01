#!/bin/bash
set -e

ros2 topic pub /autodrive/f1tenth_1/throttle_command std_msgs/msg/Float32 '{data: 0.0}' --once
ros2 topic pub /autodrive/f1tenth_1/steering_command std_msgs/msg/Float32 '{data: 0.0}' --once

