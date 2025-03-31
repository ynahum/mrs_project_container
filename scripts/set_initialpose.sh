#!/bin/bash
set -e

timeout 2s ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
{
    header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'},
    pose: {pose: {position: {x: 0.74, y: 3.1594, z: 0.0596}, orientation: {x: 0.0037, y: 0.0037, z: -0.7071, w: 0.7071}}, 
    covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
}"
