from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch rover sensors, localization, and costmaps."""
    imu_launch = PathJoinSubstitution([FindPackageShare('icm20948_imu_interface'), 'launch', 'imu_launch.py'])
    gps_launch = PathJoinSubstitution([FindPackageShare('l76x_gps_interface'), 'launch', 'gps_launch.py'])
    zed_launch = PathJoinSubstitution([FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'])
    localization_launch = PathJoinSubstitution([
        FindPackageShare('rover_localization'), 'launch', 'dual_ekf_launch.py'])
    costmap_launch = PathJoinSubstitution([
        FindPackageShare('rover_costmap'), 'launch', 'costmap.launch.py'])

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
            'enable_zed',
            default_value='true',
            description='Launch the ZED camera driver.',
        ),
        DeclareLaunchArgument('enable_localization', default_value='false'),
        DeclareLaunchArgument('enable_costmaps', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
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
        )]),
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_launch),
            condition=IfCondition(enable_zed),
            launch_arguments={
                'camera_model': camera_model,
                # robot_localization is the only owner of map/odom TF.
                'publish_tf': 'false',
                'publish_map_tf': 'false',
            }.items(),
        )]),
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            condition=IfCondition(LaunchConfiguration('enable_localization')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        )]),
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(costmap_launch),
            condition=IfCondition(LaunchConfiguration('enable_costmaps')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        )]),
    ])
