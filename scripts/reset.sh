#!/bin/bash
set -e

ros2 topic pub /autodrive/reset_command std_msgs/msg/Bool '{data: true}' --once
ros2 topic pub /autodrive/reset_command std_msgs/msg/Bool '{data: false}' --once

