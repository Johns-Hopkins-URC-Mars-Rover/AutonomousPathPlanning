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
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetRemap


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
                # Humble's velocity smoother publishes its final Twist on
                # /cmd_vel. Gazebo's diff-drive controller expects Twist on
                # its unstamped controller input.
                SetRemap(src='cmd_vel', dst='/diff_drive_controller/cmd_vel_unstamped'),
                IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'params_file': PathJoinSubstitution([
                        FindPackageShare('rover_description'), 'config', 'nav2_params.yaml']),
                }.items()),
            ])],
        ),
    ]
    return LaunchDescription(actions)
