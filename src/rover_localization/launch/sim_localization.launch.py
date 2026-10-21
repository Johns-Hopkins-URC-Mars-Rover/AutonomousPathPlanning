"""Run the local EKF against synthetic odometry and joint IMU messages."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    local_launch = PathJoinSubstitution(
        [FindPackageShare('rover_localization'), 'launch', 'localization.launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument('zed_odom_topic', default_value='/zed/zed_node/odom'),
        DeclareLaunchArgument('fused_imu_topic', default_value='/imu/fused'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(local_launch),
            launch_arguments={
                'zed_odom_topic': LaunchConfiguration('zed_odom_topic'),
                'fused_imu_topic': LaunchConfiguration('fused_imu_topic'),
            }.items(),
        ),
        Node(
            package='rover_localization',
            executable='simulated_sensors',
            name='simulated_sensors',
            output='screen',
            parameters=[{
                'zed_odom_topic': LaunchConfiguration('zed_odom_topic'),
                'fused_imu_topic': LaunchConfiguration('fused_imu_topic'),
            }],
        ),
    ])
