import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch rover sensors, localization, and costmaps."""
    bringup_share = get_package_share_directory('rover_bringup')
    description_share = get_package_share_directory('rover_description')
    urdf_path = os.path.join(description_share, 'urdf', 'rover.urdf')
    controller_config = os.path.join(
        description_share, 'config', 'rover_controller.yaml')
    madgwick_config = os.path.join(
        bringup_share, 'config', 'madgwick.yaml')
    with open(urdf_path, encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read().replace(
            '@ROVER_CONTROLLER_CONFIG@', controller_config)

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
    enable_costmap = LaunchConfiguration('enable_costmap')
    enable_nav2 = LaunchConfiguration('enable_nav2')
    zed_use_pub_timestamps = LaunchConfiguration(
        'zed_use_pub_timestamps')
    zed_grab_frame_rate = LaunchConfiguration('zed_grab_frame_rate')
    zed_point_cloud_freq = LaunchConfiguration('zed_point_cloud_freq')
    enable_robot_state_publisher = LaunchConfiguration(
        'enable_robot_state_publisher')
    camera_model = LaunchConfiguration('camera_model')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_imu',
            default_value='true',
            description='Launch the ICM-20948 driver and Madgwick orientation filter.',
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
            'enable_robot_state_publisher',
            default_value='true',
            description='Publish the rover URDF fixed and joint transforms.',
        ),
        DeclareLaunchArgument('enable_localization', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('rover_description'), 'config', 'nav2_params.yaml'])),
        DeclareLaunchArgument('cmd_vel_topic',
                              default_value='/diff_drive_controller/cmd_vel_unstamped'),
        DeclareLaunchArgument(
            'enable_costmap',
            default_value='false',
            description='Launch standalone costmaps when Nav2 is disabled.',
        ),
        DeclareLaunchArgument(
            'zed_use_pub_timestamps',
            default_value='false',
            description=(
                'Stamp ZED messages at ROS publication time. Enable only as '
                'a same-computer workaround for delayed camera timestamps.'),
        ),
        DeclareLaunchArgument(
            'zed_grab_frame_rate',
            default_value='30',
            description=(
                'ZED camera grab rate. The 30 FPS soft-test default reduces '
                'localization and point-cloud processing load.'),
        ),
        DeclareLaunchArgument(
            'zed_point_cloud_freq',
            default_value='5.0',
            description=(
                'ZED registered point-cloud publication rate. Five Hz is '
                'sufficient for the stationary costmap soft test.'),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'camera_model',
            default_value='zed2i',
            description='ZED camera model (for example: zed2i, zedx, or zedxm).',
        ),
        DeclareLaunchArgument('enable_nav2', default_value='true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            condition=IfCondition(enable_robot_state_publisher),
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
        # Scope includes so their generic params_file arguments cannot leak
        # between the IMU, GPS and EKF launches.
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_launch),
            condition=IfCondition(enable_imu),
        )]),
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            condition=IfCondition(enable_imu),
            output='screen',
            # /imu/data belongs to rover_imu_fusion, which the EKFs consume.
            remappings=[('imu/data', '/imu/madgwick')],
            parameters=[madgwick_config, {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
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
                # This ZED parameter is read-only after node initialization.
                'param_overrides': [
                    'debug.use_pub_timestamps:=',
                    zed_use_pub_timestamps,
                    ';general.grab_frame_rate:=',
                    zed_grab_frame_rate,
                    ';depth.point_cloud_freq:=',
                    zed_point_cloud_freq,
                ],
            }.items(),
        )]),
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            condition=IfCondition(LaunchConfiguration('enable_localization')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                # Fuse the ICM with the ZED IMU whenever its driver runs.
                'use_icm': enable_imu,
            }.items(),
        )]),
        # Nav2's planner and controller own their costmaps when enabled.
        GroupAction(condition=UnlessCondition(enable_nav2), actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(costmap_launch),
            condition=IfCondition(enable_costmap),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        )]),
        GroupAction(condition=IfCondition(enable_nav2), actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('rover_navigation'), 'launch', 'nav2.launch.py'])),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'params_file': LaunchConfiguration('params_file'),
                              'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            }.items()),
        ]),
    ])
