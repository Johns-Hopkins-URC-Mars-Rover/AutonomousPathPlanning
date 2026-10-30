from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'rover_description'
    urdf_name = 'simp_rover.urdf'
    entity_name = 'rover'

    package_share = get_package_share_directory(package_name)
    urdf_file = os.path.join(package_share, 'urdf', urdf_name)
    world_file = os.path.join(package_share, 'world', 'park.world')

    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={'world': world_file}.items()
    )

    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(package_share, '..') + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', entity_name, '-z', '0.5'],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen'
    )

    return LaunchDescription([
        set_model_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,

        TimerAction(
            period=3.0,
            actions=[joint_state_broadcaster_spawner]
        ),

        TimerAction(
            period=5.0,
            actions=[diff_drive_spawner]
        ),
    ])