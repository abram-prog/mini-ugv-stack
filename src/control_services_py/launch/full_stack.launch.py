from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    ws = os.path.expanduser('~/mini-stack/ros2_ws')
    tx_script = os.path.join(ws, 'video', 'gst_tx.sh')
    # приёмник лучше держать отдельно (на «пульте»), но для демо можно и здесь

    return LaunchDescription([
        Node(package='sensor_driver_cpp', executable='imu_publisher', name='imu_publisher', output='screen'),
        Node(package='control_services_py', executable='health_node',  name='health_node',  output='screen'),
        ExecuteProcess(cmd=[tx_script, '127.0.0.1', '5004'], shell=False),
        Node(package='ros2_mqtt_bridge_py', executable='bridge', name='ros2_mqtt_bridge', output='screen'),
    ])
