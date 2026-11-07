from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the rover's sensors and IMU fusion nodes."""
    imu_launch = PathJoinSubstitution([FindPackageShare('icm20948_imu_interface'), 'launch', 'imu_launch.py'])
    gps_launch = PathJoinSubstitution([FindPackageShare('l76x_gps_interface'), 'launch', 'gps_launch.py'])
    zed_launch = PathJoinSubstitution([FindPackageShare('rover_localization'), 'launch', 'zed_driver.launch.py'])

    enable_imu = LaunchConfiguration('enable_imu')
    enable_gps = LaunchConfiguration('enable_gps')
    enable_zed = LaunchConfiguration('enable_zed')
    enable_fusion = LaunchConfiguration('enable_fusion')
    camera_model = LaunchConfiguration('camera_model')
    fusion_launch = PathJoinSubstitution([FindPackageShare('rover_imu_fusion'), 'launch', 'imu_fusion.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_imu',
            default_value='false',
            description='Launch the ICM-20948 driver.',
        ),
        DeclareLaunchArgument(
            'enable_gps',
            default_value='true',
            description='Launch the L76X GPS driver.',
        ),
        DeclareLaunchArgument(
            'enable_zed',
            default_value='true',
            description='Launch the ZED camera driver.',
        ),
        DeclareLaunchArgument(
            'enable_fusion',
            default_value='true',
            description='Launch the IMU fusion node (ZED + ICM-20948).',
        ),
        DeclareLaunchArgument(
            'camera_model',
            default_value='zed2i',
            description='ZED camera model (for example: zed2i, zedx, or zedxm).',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_launch),
            condition=IfCondition(enable_imu),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gps_launch),
            condition=IfCondition(enable_gps),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_launch),
            condition=IfCondition(enable_zed),
            launch_arguments={
                'camera_model': camera_model,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fusion_launch),
            condition=IfCondition(enable_fusion),
            launch_arguments={
                'use_icm': enable_imu,
                'publish_icm_tf': 'false',
            }.items(),
        ),
    ])
