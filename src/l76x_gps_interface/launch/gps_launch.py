import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('l76x_gps_interface'),
        'config',
        'gps_params.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument(
            'calibrate_gps', default_value='false',
            description='Measure real GPS error at startup (rover must stay parked ~60 s) '
                        'and set position_variance from it.'),
        Node(
            package='l76x_gps_interface',
            executable='gps_publisher',
            name='l76x_gps_publisher',
            parameters=[LaunchConfiguration('params_file')],
        ),
        Node(
            package='l76x_gps_interface',
            executable='gps_error_calibrator',
            name='gps_error_calibrator',
            output='screen',
            condition=IfCondition(LaunchConfiguration('calibrate_gps')),
        ),
    ])
