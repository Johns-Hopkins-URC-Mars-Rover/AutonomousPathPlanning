from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import SetRemap
import os

def generate_launch_description():
    package_share = get_package_share_directory('rover_description')
    ekf_config = os.path.join(package_share, 'config', 'dual_ekf_navsat.yaml')
    nav2_config = os.path.join(package_share, 'config', 'nav2_params.yaml')

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Local EKF: fuses wheel odometry + IMU -> /odometry/local
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/local')]
    )

    # Global EKF: fuses wheel odometry + IMU + GPS-derived odom -> /odometry/global
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/global')]
    )

    # Converts GPS lat/lon into local X/Y odometry, using global EKF output for yaw
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
        remappings=[
            ('imu', '/rover/imu/data'),
            ('gps/fix', '/rover/gps/fix'),
            ('odometry/filtered', 'odometry/global')
        ]
    )

    # Nav2 bringup (planner, controller, costmaps, behavior tree navigator)
    nav2 = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst='/diff_drive_controller/cmd_vel_unstamped'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': nav2_config
                }.items()
            )
        ]
    )

    return LaunchDescription([
        ekf_local_node,
        ekf_global_node,
        navsat_transform_node,
        nav2,
    ])