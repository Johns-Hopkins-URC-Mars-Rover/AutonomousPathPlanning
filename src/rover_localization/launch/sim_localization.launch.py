"""Run the production local EKF against generated fake ZED and joint-IMU data.

``simulated_sensors`` owns both fake inputs, using the same default topic names
as the live rover. It also publishes comparison-only ground truth on
``/simulation/ground_truth``; the EKF never subscribes to that topic.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    local_launch = PathJoinSubstitution(
        [FindPackageShare('rover_localization'), 'launch', 'localization.launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument('zed_odom_topic', default_value='/zed/zed_node/odom'),
        DeclareLaunchArgument('fused_imu_topic', default_value='/imu/fused'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(local_launch),
            launch_arguments={
                'zed_odom_topic': LaunchConfiguration('zed_odom_topic'),
                'fused_imu_topic': LaunchConfiguration('fused_imu_topic'),
            }.items(),
        ),
        Node(
            # This node generates the fake ZED Odometry and fake joint Imu
            # messages; it is not a second EKF and does not use camera hardware.
            package='rover_localization',
            executable='simulated_sensors',
            name='simulated_sensors',
            output='screen',
            parameters=[{
                'zed_odom_topic': LaunchConfiguration('zed_odom_topic'),
                'fused_imu_topic': LaunchConfiguration('fused_imu_topic'),
            }],
        ),
    ])
