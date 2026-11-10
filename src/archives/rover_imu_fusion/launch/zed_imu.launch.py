import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('rover_imu_fusion'), 'config', 'zed_imu.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        Node(
            package='rover_imu_fusion',
            executable='zed_imu_node',
            name='zed_imu',
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
        ),
    ])
