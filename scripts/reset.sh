#!/bin/bash
set -e

timeout 2s ros2 topic pub /autodrive/reset_command std_msgs/msg/Bool '{data: true}'
