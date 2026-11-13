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
        DeclareLaunchArgument('heading', default_value='0.0', description='Map yaw degrees: +X=0, +Y=90'),
        DeclareLaunchArgument('navigation_timeout', default_value='600.0',
                              description='Timeout in seconds for each Nav2 waypoint.'),
        DeclareLaunchArgument('max_goal_distance', default_value='70.0',
                              description='Maximum distance per waypoint; final GPS targets may be farther.'),
        DeclareLaunchArgument('waypoint_margin', default_value='1.0',
                              description='Keep intermediate goals this far inside the rolling costmap [m].'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='rover_navigation',
            executable='navigation_client',
            name='navigation_client',
            parameters=[{
                'lat': LaunchConfiguration('lat'),
                'lon': LaunchConfiguration('lon'),
                'heading': LaunchConfiguration('heading'),
                'navigation_timeout': LaunchConfiguration('navigation_timeout'),
                'max_goal_distance': LaunchConfiguration('max_goal_distance'),
                'waypoint_margin': LaunchConfiguration('waypoint_margin'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
