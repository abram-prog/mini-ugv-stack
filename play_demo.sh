#!/usr/bin/env bash
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/humble/setup.bash
cd "$DIR"
colcon build --symlink-install >/dev/null
source install/setup.bash

BAG="${1:-$DIR/video/demo_bag}"
if [ ! -f "$BAG/metadata.yaml" ]; then
  echo "[!] Bag not found: $BAG"; exit 1
fi

echo "[i] Starting bridge..."
( ros2 run ros2_mqtt_bridge_py bridge ) & BRIDGE_PID=$!

# маленькая задержка, чтобы мост поднял REST
sleep 1

echo "[i] Hitting REST:"
curl -s http://127.0.0.1:8000/health || true

echo "[i] Playing bag (loop, 1.5x): $BAG"
ros2 bag play "$BAG" -l -r 1.5

echo "[i] Stopping bridge..."
kill $BRIDGE_PID 2>/dev/null || true
