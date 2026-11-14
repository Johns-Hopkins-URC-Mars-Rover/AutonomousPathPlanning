"""One hardware Nav2 stack, with no command path around its stop filters."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    clock = LaunchConfiguration('use_sim_time')
    # Use the same parameter file for servers, monitor, and command guard.
    params = ParameterFile(RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        param_rewrites={'use_sim_time': clock}, convert_types=True),
        allow_substs=True)
    specifications = [
        ('nav2_controller', 'controller_server', 'controller_server',
         [('cmd_vel', '/cmd_vel_nav')]),
        ('nav2_smoother', 'smoother_server', 'smoother_server', []),
        ('nav2_planner', 'planner_server', 'planner_server', []),
        ('nav2_behaviors', 'behavior_server', 'behavior_server',
         [('cmd_vel', '/cmd_vel_nav')]),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', []),
        ('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower', []),
        ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
         [('cmd_vel', '/cmd_vel_nav'), ('cmd_vel_smoothed', '/cmd_vel_filtered')]),
        ('nav2_collision_monitor', 'collision_monitor', 'collision_monitor', []),
    ]
    # Start the guard before lifecycle activation. It publishes zero until all
    # inputs are fresh. It alone owns autonomous commands to the motor interface.
    nodes = [Node(
        package='rover_navigation', executable='velocity_guard',
        name='velocity_guard', output='screen', parameters=[params],
        remappings=[('/diff_drive_controller/cmd_vel_unstamped',
                     LaunchConfiguration('cmd_vel_topic'))])]
    nodes += [Node(package=package, executable=executable, name=name,
                   output='screen', parameters=[params], remappings=remaps)
              for package, executable, name, remaps in specifications]
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': clock, 'autostart': True,
                     # Activate output filtering before any motion-producing node.
                     'node_names': ['collision_monitor', 'velocity_smoother',
                                    'controller_server', 'smoother_server',
                                    'planner_server', 'behavior_server',
                                    'bt_navigator', 'waypoint_follower']}]))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('rover_description'), 'config', 'nav2_params.yaml'])),
        DeclareLaunchArgument('cmd_vel_topic',
                              default_value='/diff_drive_controller/cmd_vel_unstamped'),
        *nodes,
    ])
