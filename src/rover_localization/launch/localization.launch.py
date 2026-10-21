"""Fuse ZED optical odometry and the upstream joint IMU in a local EKF."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('rover_localization'), 'config', 'ekf_local.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument('zed_odom_topic', default_value='/zed/zed_node/odom'),
        DeclareLaunchArgument('fused_imu_topic', default_value='/imu/fused'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[config, {
                'odom0': LaunchConfiguration('zed_odom_topic'),
                'imu0': LaunchConfiguration('fused_imu_topic'),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool
                ),
            }],
            remappings=[('odometry/filtered', '/odometry/local')],
        ),
    ])
