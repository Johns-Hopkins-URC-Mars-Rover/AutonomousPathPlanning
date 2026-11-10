"""Self-contained robot_localization EKFs and optional Nav2.

Hardware drivers run separately (normally on the Jetson). This launch owns the
estimators; do not also start another local EKF from rover_bringup.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetRemap


def generate_launch_description():
    args = {'use_sim_time': 'false',
            'zed_odom_topic': '/zed/zed_node/odom',
            'wheel_odom_topic': '/diff_drive_controller/odom',
            'zed_imu_topic': '/zed/zed_node/imu/data', 'icm_imu_topic': '/imu/data_raw',
            'gps_topic': '/gps/fix', 'heading_topic': '/imu/heading',
            'enable_global': 'false', 'heading_mode': 'unset',
            'initial_heading_deg': 'nan', 'publish_approximate_mounts': 'false',
            'use_zed_odom': 'true', 'use_wheel_odom': 'true',
            'use_zed_imu': 'true', 'use_icm_imu': 'true',
            'enable_nav2': 'false'}
    def include(package, file, names, condition=None):
        kwargs = {} if condition is None else {'condition': condition}
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), 'launch', file])),
            launch_arguments={name: LaunchConfiguration(name) for name in names}.items(), **kwargs)
    actions = [DeclareLaunchArgument(name, default_value=value) for name, value in args.items()]
    actions += [
        GroupAction(actions=[include('robot_localization', 'dual_ekf.launch.py',
                tuple(name for name in args if name != 'enable_nav2'))]),
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
