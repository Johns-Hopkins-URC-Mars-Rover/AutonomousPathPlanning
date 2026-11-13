import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_localization')
    local_config = os.path.join(pkg_share, 'config', 'local_ekf.yaml')
    global_config = os.path.join(pkg_share, 'config', 'global_ekf.yaml')
    navsat_config = os.path.join(pkg_share, 'config', 'navsat.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # 1. Local EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='local_ekf_node',
            output='screen',
            parameters=[local_config, {'use_sim_time': use_sim_time}],
            remappings=[('/odometry/filtered', '/odometry/local')],
        ),

        # 2. Global EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='global_ekf_node',
            output='screen',
            parameters=[global_config, {'use_sim_time': use_sim_time}],
            remappings=[('/odometry/filtered', '/odometry/global')],
        ),

        # 3. Navsat Transform Node (GPS handler)
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[navsat_config, {'use_sim_time': use_sim_time}],
            remappings=[
                ('/gps/fix', '/gps/fix'),
                ('/imu', '/zed/zed_node/imu/data'),
                # GPS poses must be expressed in the global EKF's map frame,
                # without depending on its changing map -> odom transform.
                ('/odometry/filtered', '/odometry/global'),
                ('/odometry/gps', '/odometry/gps'),
            ],
        ),
    ])
