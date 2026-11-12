from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        # Gazebo park.world's spherical-coordinate origin. Callers normally
        # override these with the desired target coordinates.
        DeclareLaunchArgument('lat', default_value='39.323178'),
        DeclareLaunchArgument('lon', default_value='-76.627894'),
        DeclareLaunchArgument('heading', default_value='0.0'),
        DeclareLaunchArgument('max_goal_distance', default_value='70.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='rover_navigation',
            executable='navigation_client',
            name='navigation_client',
            parameters=[{
                'lat': LaunchConfiguration('lat'),
                'lon': LaunchConfiguration('lon'),
                'heading': LaunchConfiguration('heading'),
                'max_goal_distance': LaunchConfiguration('max_goal_distance'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
