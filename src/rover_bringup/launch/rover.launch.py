from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch sensors and optional direct dual-EKF localization."""
    imu_launch = PathJoinSubstitution([FindPackageShare('icm20948_imu_interface'), 'launch', 'imu_launch.py'])
    gps_launch = PathJoinSubstitution([FindPackageShare('l76x_gps_interface'), 'launch', 'gps_launch.py'])
    zed_launch = PathJoinSubstitution([FindPackageShare('rover_localization'), 'launch', 'zed_driver.launch.py'])

    enable_imu = LaunchConfiguration('enable_imu')
    enable_gps = LaunchConfiguration('enable_gps')
    enable_zed = LaunchConfiguration('enable_zed')
    camera_model = LaunchConfiguration('camera_model')

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
            'calibrate_gps',
            default_value='false',
            description='Optional: at startup, measure real GPS error (rover parked ~60 s) and set its variance.',
        ),
        DeclareLaunchArgument(
            'enable_zed',
            default_value='true',
            description='Launch the ZED camera driver.',
        ),
        DeclareLaunchArgument('enable_localization', default_value='false'),
        DeclareLaunchArgument('enable_global', default_value='false'),
        DeclareLaunchArgument('heading_mode', default_value='unset'),
        DeclareLaunchArgument('initial_heading_deg', default_value='nan'),
        DeclareLaunchArgument('heading_topic', default_value='/imu/heading'),
        DeclareLaunchArgument('publish_approximate_mounts', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_zed_odom', default_value='true'),
        DeclareLaunchArgument('use_wheel_odom', default_value='true'),
        DeclareLaunchArgument('use_zed_imu', default_value='true'),
        DeclareLaunchArgument('use_icm_imu', default_value='true'),
        DeclareLaunchArgument(
            'camera_model',
            default_value='zed2i',
            description='ZED camera model (for example: zed2i, zedx, or zedxm).',
        ),
        # Scope includes so their generic params_file arguments cannot leak
        # between the IMU, GPS and EKF launches.
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_launch),
            condition=IfCondition(enable_imu),
        )]),
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gps_launch),
            condition=IfCondition(enable_gps),
            launch_arguments={'calibrate_gps': LaunchConfiguration('calibrate_gps')}.items(),
        )]),
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_launch),
            condition=IfCondition(enable_zed),
            launch_arguments={
                'camera_model': camera_model,
            }.items(),
        )]),
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('rover_localization'), 'launch', 'hardware_localization.launch.py'])),
            condition=IfCondition(LaunchConfiguration('enable_localization')),
            launch_arguments={name: LaunchConfiguration(name) for name in (
                'enable_global', 'heading_mode', 'initial_heading_deg', 'heading_topic',
                'publish_approximate_mounts', 'use_sim_time',
                'use_zed_odom', 'use_wheel_odom', 'use_zed_imu', 'use_icm_imu')}.items(),
        )]),
    ])
