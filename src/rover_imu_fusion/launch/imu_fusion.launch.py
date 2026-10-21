import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('rover_imu_fusion'), 'config', 'imu_fusion.yaml')

    use_icm = LaunchConfiguration('use_icm')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='true when replaying a bag with ros2 bag play --clock'),
        DeclareLaunchArgument(
            'use_icm', default_value='false',
            description='Fuse the ICM-20948 as well (needs measured covariances and TF)'),
        DeclareLaunchArgument(
            'publish_icm_tf', default_value='true',
            description='Publish the placeholder base_link -> imu_link below. '
                        'Set false when a real transform exists (e.g. replaying a bag).'),
        # Placeholder mount of the ICM relative to base_link. Replace with the
        # measured values (metres and radians) before using the ICM.
        DeclareLaunchArgument('icm_x', default_value='0.0'),
        DeclareLaunchArgument('icm_y', default_value='0.0'),
        DeclareLaunchArgument('icm_z', default_value='0.0'),
        DeclareLaunchArgument('icm_yaw', default_value='0.0'),
        DeclareLaunchArgument('icm_pitch', default_value='0.0'),
        DeclareLaunchArgument('icm_roll', default_value='0.0'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='icm_mount_tf',
            condition=IfCondition(PythonExpression([
                "'", use_icm, "' == 'true' and '",
                LaunchConfiguration('publish_icm_tf'), "' == 'true'"])),
            arguments=[
                '--x', LaunchConfiguration('icm_x'),
                '--y', LaunchConfiguration('icm_y'),
                '--z', LaunchConfiguration('icm_z'),
                '--yaw', LaunchConfiguration('icm_yaw'),
                '--pitch', LaunchConfiguration('icm_pitch'),
                '--roll', LaunchConfiguration('icm_roll'),
                '--frame-id', 'base_link',
                '--child-frame-id', 'imu_link',
            ],
        ),
        Node(
            package='rover_imu_fusion',
            executable='imu_fusion_node',
            name='imu_fusion',
            parameters=[
                LaunchConfiguration('params_file'),
                {'icm.enabled': ParameterValue(use_icm, value_type=bool),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                                value_type=bool)},
            ],
            output='screen',
        ),
    ])
