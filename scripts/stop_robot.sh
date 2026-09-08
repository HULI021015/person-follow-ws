#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash

for _ in {1..10}; do
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null
done

echo "已向 /cmd_vel 连续发布零速。"

