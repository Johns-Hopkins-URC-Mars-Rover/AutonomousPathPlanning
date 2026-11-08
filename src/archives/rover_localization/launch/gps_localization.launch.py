"""Optional global localization; the continuous local EKF runs separately."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def start_global(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    if value('enable_gps_fusion').lower() != 'true':
        return []
    if any(value(name).lower() != 'true' for name in ('heading_verified', 'mounts_verified')):
        raise RuntimeError('GPS fusion requires heading_verified:=true and mounts_verified:=true. '
                           'Measure mounts and verify an earth-referenced heading; startup-relative IMU yaw is insufficient.')
    clock = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    config = PathJoinSubstitution([FindPackageShare('rover_localization'), 'config', 'ekf_global.yaml'])
    base = LaunchConfiguration('base_frame')
    return [
        Node(package='rover_localization', executable='sensor_inputs.py', name='gps_sensor_inputs',
             output='screen', parameters=[{'mode': 'gps', 'gps_topic': LaunchConfiguration('gps_topic'),
                                          'heading_topic': LaunchConfiguration('heading_topic'),
                                          'base_frame': base, 'max_age': 2.0, 'use_sim_time': clock}]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_global',
             output='screen', parameters=[config, {'base_link_frame': base, 'use_sim_time': clock}],
             remappings=[('odometry/filtered', '/odometry/global')]),
        Node(package='robot_localization', executable='navsat_transform_node', name='navsat_transform',
             output='screen', parameters=[{
                 'use_sim_time': clock, 'frequency': 10.0, 'delay': 3.0,
                 'use_odometry_yaw': False, 'zero_altitude': False,
                 'broadcast_cartesian_transform': False, 'publish_filtered_gps': False,
                 # Supply an ENU, true-east-zero orientation on the heading topic.
                 # Correct magnetic declination / sensor offsets upstream.
                 'yaw_offset': 0.0, 'magnetic_declination_radians': 0.0,
             }], remappings=[('imu', '/localization/heading'),
                             ('gps/fix', '/gps/validated'),
                             ('odometry/filtered', '/odometry/global'),
                             ('odometry/gps', '/odometry/gps')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_gps_fusion', default_value='false'),
        DeclareLaunchArgument('heading_verified', default_value='false'),
        DeclareLaunchArgument('mounts_verified', default_value='false'),
        DeclareLaunchArgument('gps_topic', default_value='/gps/fix'),
        DeclareLaunchArgument('heading_topic', default_value='/imu/heading'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=start_global),
    ])
