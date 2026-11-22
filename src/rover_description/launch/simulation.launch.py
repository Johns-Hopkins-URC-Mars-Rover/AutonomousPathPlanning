"""Start Gazebo sensors and the same localization stack used on the rover."""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the simulated hardware boundary, localization, and optional Nav2."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_nav2 = LaunchConfiguration('enable_nav2')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('rover_description'), 'launch',
            'gazebo_launch.py',
        ])),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('rover_localization'), 'launch',
            'dual_ekf_launch.py',
        ])),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    magnetic_field = Node(
        package='rover_simulation',
        executable='magnetic_field_simulator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    madgwick = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('rover_simulation'), 'config',
            'madgwick_sim.yaml',
        ]), {'use_sim_time': use_sim_time}],
        remappings=[('imu/data', '/imu/data')],
    )

    readiness = Node(
        package='rover_navigation',
        executable='wait_for_localization',
        name='wait_for_localization',
        output='screen',
        condition=IfCondition(enable_nav2),
        parameters=[{
            'use_sim_time': use_sim_time,
            'timeout': LaunchConfiguration('startup_timeout'),
        }],
    )
    nav2 = GroupAction(actions=[
        SetRemap(
            src='/cmd_vel',
            dst='/diff_drive_controller/cmd_vel_unstamped'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch',
                'navigation_launch.py',
            ])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': PathJoinSubstitution([
                    FindPackageShare('rover_navigation'), 'config',
                    'nav2_params.yaml',
                ]),
            }.items(),
        ),
    ])

    def after_readiness(event, _context):
        if event.returncode == 0:
            return [nav2]
        return [EmitEvent(event=Shutdown(
            reason='Localization readiness check failed'))]

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([
                FindPackageShare('rover_description'), 'world', 'flat.world',
            ]),
        ),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'enable_nav2',
            default_value='false',
            description=(
                'Start Nav2 without a GPS goal. Leave false when launching '
                'rover_navigation/navigation_launch.py separately.'),
        ),
        DeclareLaunchArgument('startup_timeout', default_value='120.0'),
        gazebo,
        magnetic_field,
        madgwick,
        localization,
        RegisterEventHandler(OnProcessExit(
            target_action=readiness,
            on_exit=after_readiness,
        )),
        readiness,
    ])
