from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='icm20948_imu_interface',
            executable='imu_publisher',
            name='imu_publisher'
            )
        ])
