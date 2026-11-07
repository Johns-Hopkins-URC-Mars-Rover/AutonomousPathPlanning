"""Camera localization + optional GPS global localization and optional Nav2.

Hardware drivers run separately (normally on the Jetson). This launch owns the
estimators; do not also start localization.launch.py or another local EKF.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetRemap


def generate_launch_description():
    args = {'use_sim_time': 'false', 'base_frame': 'base_link',
            'zed_odom_topic': '/zed/zed_node/odom', 'fused_imu_topic': '/imu/data',
            'gps_topic': '/gps/fix', 'heading_topic': '/imu/heading',
            'enable_gps_fusion': 'false', 'heading_verified': 'false',
            'mounts_verified': 'false', 'enable_nav2': 'false'}
    def include(package, file, names, condition=None):
        kwargs = {} if condition is None else {'condition': condition}
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), 'launch', file])),
            launch_arguments={name: LaunchConfiguration(name) for name in names}.items(), **kwargs)
    actions = [DeclareLaunchArgument(name, default_value=value) for name, value in args.items()]
    actions += [
        include('rover_localization', 'localization.launch.py',
                ('use_sim_time', 'base_frame', 'zed_odom_topic', 'fused_imu_topic')),
        include('rover_localization', 'gps_localization.launch.py',
                ('use_sim_time', 'base_frame', 'gps_topic', 'heading_topic',
                 'enable_gps_fusion', 'heading_verified', 'mounts_verified')),
        GroupAction(condition=IfCondition(LaunchConfiguration('enable_nav2')), actions=[
            SetRemap(src='/cmd_vel', dst='/diff_drive_controller/cmd_vel_unstamped'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'params_file': PathJoinSubstitution([FindPackageShare('rover_description'),
                                                                  'config', 'nav2_params.yaml'])}.items()),
        ]),
    ]
    return LaunchDescription(actions)
