"""Start the complete Gazebo, localization and optional Nav2 simulation."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('rover_description')

    def include(filename, arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                share, 'launch', filename])),
            launch_arguments=arguments.items())

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=PathJoinSubstitution([
            share, 'world', 'flat.world'])),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('enable_nav2', default_value='true'),
        DeclareLaunchArgument('startup_timeout', default_value='120.0'),
        include('gazebo_launch.py', {
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
        }),
        include('gps_nav_launch.py', {
            'use_sim_time': 'true',
            'enable_nav2': LaunchConfiguration('enable_nav2'),
            'startup_timeout': LaunchConfiguration('startup_timeout'),
        }),
    ])
