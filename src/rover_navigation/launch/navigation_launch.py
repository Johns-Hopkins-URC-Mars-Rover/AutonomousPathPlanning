from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

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
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('rover_description'), 'config', 'nav2_params.yaml']),
            description='Nav2 parameters for the physical rover.',
        ),
        # rover_bringup owns the dual EKFs. This launch owns only Nav2 and the
        # GPS goal client, preventing duplicate localization/TF publishers.
        GroupAction(actions=[
            SetRemap(
                src='/cmd_vel',
                dst='/diff_drive_controller/cmd_vel_unstamped'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py',
                ])),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                }.items(),
            ),
        ]),
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
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
