from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_driver_cpp',
            executable='imu_publisher',
            name='imu_publisher',
            output='screen'
        ),
        Node(
            package='control_services_py',
            executable='health_node',
            name='health_node',
            output='screen'
        ),
    ])
