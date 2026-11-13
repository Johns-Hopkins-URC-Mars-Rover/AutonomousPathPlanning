"""Gazebo Classic rover; world defaults to a portable flat test fixture."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('rover_description')
    with open(os.path.join(share, 'urdf', 'simp_rover.urdf'), encoding='utf-8') as stream:
        description = stream.read().replace('@ROVER_CONTROLLER_CONFIG@',
            os.path.join(share, 'config', 'rover_controller.yaml'))
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'rover', '-z', '0.5',
                   '-timeout', '120'], output='screen')
    # Spawners wait for controller-manager services instead of assuming fixed
    # startup duration. Start after spawn exits so the plugin can initialize.
    joint_states = Node(package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '120'],
        output='screen')
    drive = Node(package='controller_manager', executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager-timeout', '120'],
        output='screen')
    def next_or_shutdown(next_actions):
        def callback(event, context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason='Robot spawn/controller startup failed'))]
            return next_actions
        return callback
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(share, 'world', 'flat.world')),
        DeclareLaunchArgument('gui', default_value='false'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.dirname(share) + ':' +
            os.environ.get('GAZEBO_MODEL_PATH', '') + ':/usr/share/gazebo-11/models'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world'),
                              'gui': LaunchConfiguration('gui')}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(description, value_type=str),
                          'use_sim_time': True}], output='screen'),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=next_or_shutdown([joint_states]))),
        RegisterEventHandler(OnProcessExit(target_action=joint_states, on_exit=next_or_shutdown([drive]))),
        RegisterEventHandler(OnProcessExit(target_action=drive, on_exit=next_or_shutdown([]))),
        spawn,
    ])
