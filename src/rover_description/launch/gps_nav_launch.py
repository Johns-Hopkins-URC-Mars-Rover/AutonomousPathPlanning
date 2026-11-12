"""Launch the simulation localization stack and Nav2."""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = {'use_sim_time': 'true', 'enable_nav2': 'true'}
    def include(package, file, names, condition=None):
        kwargs = {} if condition is None else {'condition': condition}
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), 'launch', file])),
            launch_arguments={name: LaunchConfiguration(name) for name in names}.items(), **kwargs)
    actions = [DeclareLaunchArgument(name, default_value=value) for name, value in args.items()]
    actions += [
        GroupAction(actions=[include('rover_localization', 'dual_ekf_launch.py',
                ('use_sim_time',))]),
        # Give the EKFs time to establish map -> odom -> base_link before the
        # Nav2 lifecycle manager activates its costmaps.
        TimerAction(
            period=3.0,
            condition=IfCondition(LaunchConfiguration('enable_nav2')),
            actions=[GroupAction(actions=[
                IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'params_file': PathJoinSubstitution([
                        FindPackageShare('rover_description'), 'config', 'nav2_params.yaml']),
                }.items()),
                # Keep Nav2's standard controller -> velocity smoother topic
                # chain, then bridge its final Twist to Gazebo's unstamped
                # diff-drive input.
                Node(
                    package='rover_navigation',
                    executable='cmd_vel_relay',
                    name='cmd_vel_relay',
                    output='screen',
                    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                ),
            ])],
        ),
    ]
    return LaunchDescription(actions)
