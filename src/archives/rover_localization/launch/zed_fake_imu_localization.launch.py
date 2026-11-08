"""Run real ZED optical odometry with a stationary fake joint IMU."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    localization_launch = PathJoinSubstitution(
        [FindPackageShare('rover_localization'), 'launch', 'zed_localization.launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera_model', default_value='zed2i'),
        DeclareLaunchArgument('zed_odom_topic', default_value='/zed/zed_node/odom'),
        DeclareLaunchArgument('fused_imu_topic', default_value='/imu/fused'),
        DeclareLaunchArgument('fake_imu_rate_hz', default_value='50.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                'camera_model': LaunchConfiguration('camera_model'),
                'zed_odom_topic': LaunchConfiguration('zed_odom_topic'),
                'fused_imu_topic': LaunchConfiguration('fused_imu_topic'),
            }.items(),
        ),
        Node(
            package='rover_localization',
            executable='fake_imu_publisher.py',
            name='fake_imu_publisher',
            output='screen',
            parameters=[{
                'topic': LaunchConfiguration('fused_imu_topic'),
                'frame_id': 'base_link',
                'rate_hz': LaunchConfiguration('fake_imu_rate_hz'),
            }],
        ),
    ])
