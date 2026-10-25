import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("nav2_costmap"),
        "config",
        "zed_costmap.yaml",
    )

    return LaunchDescription([
        Node(
            package="nav2_costmap_2d",
            executable="nav2_costmap_2d",
            output="screen",
            parameters=[config_path],
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="costmap_lifecycle_manager",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "autostart": True,
                "node_names": [
                    "/costmap/costmap",
                    ],
            }],
        ),
    ])
