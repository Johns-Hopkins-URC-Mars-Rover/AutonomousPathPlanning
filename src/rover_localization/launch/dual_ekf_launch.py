import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_localization')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')

    return LaunchDescription([
        # 1. Local EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='local_ekf_node',
            output='screen',
            parameters=[ekf_config_path]
        ),

        # 2. Global EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='global_ekf_node',
            output='screen',
            parameters=[ekf_config_path],
            # Rename default output topic so it doesn't conflict with local EKF
            remappings=[('/odometry/filtered', '/odometry/filtered/global')]
        ),

        # 3. Navsat Transform Node (GPS handler)
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[ekf_config_path],
            remappings=[
                ('/gps/fix', '/gps/fix'), # Change right-side topic if your GPS uses a different name
                ('/imu', '/zed/zed_node/imu/data'), # Input IMU for global heading orientation
                ('/odometry/filtered', '/odometry/filtered/global'),
                ('/odometry/gps', '/odometry/gps')
            ]
        )
    ])
