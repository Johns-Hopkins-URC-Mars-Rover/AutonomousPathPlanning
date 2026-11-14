"""Send a GPS mission to an existing hardware navigation stack.

rover_bringup/rover.launch.py owns localization and Nav2 by default. Set
launch_nav2:=true only when those sensors/localization run without Nav2 already.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    numeric = {
        # No geographic default: never drive to an old simulation destination
        # because an operator forgot to supply a coordinate.
        'lat': None, 'lon': None,
        # NaN means choose approach bearing, not an arbitrary final yaw of zero.
        # Arrival is position-only; a finite heading changes the planned approach.
        'heading': 'nan',
        'navigation_timeout': '600.0',
        'max_goal_distance': '10.0', 'waypoint_margin': '1.0',
        'arrival_tolerance': '1.0',
    }
    arguments = [DeclareLaunchArgument(name, **(
        {} if default is None else {'default_value': default}))
        for name, default in numeric.items()]
    client = Node(package='rover_navigation', executable='navigation_client',
                  name='navigation_client', output='screen', parameters=[{
                      **{name: ParameterValue(LaunchConfiguration(name), value_type=float)
                         for name in numeric},
                      'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                  }])

    def completed(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            # ros2 launch otherwise exits successfully even if its only child
            # failed, obscuring a canceled or aborted GPS mission.
            raise RuntimeError(f'GPS mission failed (client exit {event.returncode})')
        return [EmitEvent(event=Shutdown(reason='GPS mission completed'))]

    return LaunchDescription([
        *arguments,
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('launch_nav2', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('rover_description'), 'config', 'nav2_params.yaml'])),
        DeclareLaunchArgument('cmd_vel_topic',
                              default_value='/diff_drive_controller/cmd_vel_unstamped'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('rover_navigation'), 'launch', 'nav2.launch.py'])),
            condition=IfCondition(LaunchConfiguration('launch_nav2')),
            launch_arguments={key: LaunchConfiguration(key)
                              for key in ('use_sim_time', 'params_file', 'cmd_vel_topic')}.items()),
        RegisterEventHandler(OnProcessExit(target_action=client, on_exit=completed)),
        client,
    ])
