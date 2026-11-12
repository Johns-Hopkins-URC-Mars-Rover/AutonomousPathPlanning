from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('lat', default_value='1.0'),
        DeclareLaunchArgument('lon', default_value='1.0'),
        DeclareLaunchArgument('heading', default_value='0.0'),
        Node(
            package='rover_navigation',
            executable='navigation_client',
            name='navigation_client',
            parameters=[{
                'lat': LaunchConfiguration('lat'),
                'lon': LaunchConfiguration('lon'),
                'heading': LaunchConfiguration('heading'),
            }],
        ),
    ])
