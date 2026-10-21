# combine ZED optical odometry with the already-fused joint IMU stream

# import ros2 launch tools
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# ROS calls this function when you launch this file
def generate_launch_description():
    # finds yaml file in the rover_localization package that has filter settings
    config = PathJoinSubstitution(
        [FindPackageShare('rover_localization'), 'config', 'ekf_local.yaml']
    )

    # TODO: default topic names are assumed - may meed to change
    # starts ekf node
    return LaunchDescription([ # defines three launch arguments
        DeclareLaunchArgument('zed_odom_topic', default_value='/zed/zed_node/odom'), # tells efk which zed odom topic to subscribe to
        DeclareLaunchArgument('fused_imu_topic', default_value='/imu/fused'), # tells it where the joint imu stream will arrive
        DeclareLaunchArgument('use_sim_time', default_value='false'), # chooses whether nodes follow ros /clock
        Node( # starts ros process
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[config, {
                'odom0': LaunchConfiguration('zed_odom_topic'),
                'imu0': LaunchConfiguration('fused_imu_topic'),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool
                ),
            }],
            remappings=[('odometry/filtered', '/odometry/local')],
        ),
    ])
