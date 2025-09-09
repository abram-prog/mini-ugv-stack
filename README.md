
## 🚀 Quickstart

```bash
# 1) Environment (in every new terminal)
source /opt/ros/humble/setup.bash
cd ~/mini-stack/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 2) Launch ROS2 nodes (Terminal A)
ros2 launch control_services_py robot_bringup.launch.py

# 3) Start the bridge (Terminal B)
ros2 run ros2_mqtt_bridge_py bridge
# REST:  http://127.0.0.1:8000/health
# UI:    http://127.0.0.1:8000/ui

# 4) Video (optional)
./video/gst_rx.sh 5004             # receiver
./video/gst_tx.sh 127.0.0.1 5004   # sender (or VIDEO_FILE=sample.mp4 ...)

### UI (Operator Panel)
![UI Screenshot](docs/img/ui.jpg)
