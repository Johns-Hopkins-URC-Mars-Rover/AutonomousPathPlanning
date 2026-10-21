import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Include the IMU and ZED launch files."""
    imu_launch = os.path.join(get_package_share_directory('icm20948_imu_interface'),'launch','imu_launch.py',)
    zed_launch = os.path.join(get_package_share_directory('zed_wrapper'),'launch','zed_camera.launch.py',)

    enable_imu = LaunchConfiguration('enable_imu')
    enable_zed = LaunchConfiguration('enable_zed')
    enable_fusion = LaunchConfiguration('enable_fusion')
    camera_model = LaunchConfiguration('camera_model')
    fusion_launch = os.path.join(get_package_share_directory('rover_imu_fusion'),'launch','imu_fusion.launch.py',)

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_imu',
            default_value='true',
            description='Launch the ICM-20948 driver.',
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
            }.items(),
        ),
    ])
