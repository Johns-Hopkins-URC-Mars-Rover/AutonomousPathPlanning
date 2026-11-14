"""IMU fusion consumes measured TF; it never invents sensor mounts."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def check_mount_owner(context):
    if LaunchConfiguration('publish_icm_tf').perform(context).lower() == 'true':
        raise RuntimeError('Publish measured base_link -> imu_link through rover_description '
                           'sensor_mounts.launch.py. Placeholder IMU mount publishing is disabled.')
    return []


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('rover_imu_fusion'), 'config', 'imu_fusion.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_icm', default_value='false'),
        DeclareLaunchArgument('publish_icm_tf', default_value='false'),
        OpaqueFunction(function=check_mount_owner),
        Node(package='rover_imu_fusion', executable='imu_fusion_node', name='imu_fusion',
             output='screen', parameters=[LaunchConfiguration('params_file'), {
                 'icm.enabled': ParameterValue(LaunchConfiguration('use_icm'), value_type=bool),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
             }]),
    ])
