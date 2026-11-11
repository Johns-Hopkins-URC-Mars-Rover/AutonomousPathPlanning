import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('rover_costmap'),
        'config',
        'costmap_params.yaml',
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    costmap_parameters = [params_file, {'use_sim_time': use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            namespace='global_costmap',
            name='global_costmap',
            output='screen',
            parameters=costmap_parameters,
        ),
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            namespace='local_costmap',
            name='local_costmap',
            output='screen',
            parameters=costmap_parameters,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='costmap_lifecycle_manager',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'global_costmap/global_costmap',
                    'local_costmap/local_costmap',
                ],
            }],
        ),
    ])
