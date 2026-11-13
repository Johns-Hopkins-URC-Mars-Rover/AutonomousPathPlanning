"""Launch dual EKFs, then start Nav2 when localization is ready."""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, GroupAction,
    IncludeLaunchDescription, RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    clock = LaunchConfiguration('use_sim_time')
    readiness = Node(
        package='rover_navigation', executable='wait_for_localization',
        name='wait_for_localization', output='screen',
        condition=IfCondition(LaunchConfiguration('enable_nav2')),
        parameters=[{'use_sim_time': clock,
                     'timeout': LaunchConfiguration('startup_timeout')}],
    )
    navigation = GroupAction(actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])),
            launch_arguments={
                'use_sim_time': clock,
                'params_file': PathJoinSubstitution([
                    FindPackageShare('rover_description'), 'config', 'nav2_params.yaml']),
            }.items()),
        Node(package='rover_navigation', executable='cmd_vel_relay',
             name='cmd_vel_relay', output='screen',
             parameters=[{'use_sim_time': clock}]),
    ])

    def after_readiness(event, context):
        if event.returncode == 0:
            return [navigation]
        return [EmitEvent(event=Shutdown(reason='Localization readiness check failed'))]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('enable_nav2', default_value='true'),
        DeclareLaunchArgument('startup_timeout', default_value='120.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('rover_localization'), 'launch', 'dual_ekf_launch.py'])),
            launch_arguments={'use_sim_time': clock}.items()),
        RegisterEventHandler(OnProcessExit(
            target_action=readiness, on_exit=after_readiness)),
        readiness,
    ])
