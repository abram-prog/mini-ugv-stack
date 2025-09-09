#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
cd "$(dirname "$0")/.."
colcon build --symlink-install
source install/setup.bash
echo "[ok] ROS2 env ready"
