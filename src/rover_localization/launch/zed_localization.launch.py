"""Start the existing ZED wrapper in optical-only mode and the local EKF."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    zed_launch = PathJoinSubstitution(
        [FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py']
    )
    local_launch = PathJoinSubstitution(
        [FindPackageShare('rover_localization'), 'launch', 'localization.launch.py']
    )
    zed_config = PathJoinSubstitution(
        [FindPackageShare('rover_localization'), 'config', 'zed_optical.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera_model', default_value='zed2i'),
        DeclareLaunchArgument('zed_odom_topic', default_value='/zed/zed_node/odom'),
        DeclareLaunchArgument('fused_imu_topic', default_value='/imu/fused'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_launch),
            launch_arguments={
                'camera_model': LaunchConfiguration('camera_model'),
                'ros_params_override_path': zed_config,
                'publish_tf': 'false',
                'publish_map_tf': 'false',
                'enable_gnss': 'false',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(local_launch),
            launch_arguments={
                'zed_odom_topic': LaunchConfiguration('zed_odom_topic'),
                'fused_imu_topic': LaunchConfiguration('fused_imu_topic'),
            }.items(),
        ),
    ])
