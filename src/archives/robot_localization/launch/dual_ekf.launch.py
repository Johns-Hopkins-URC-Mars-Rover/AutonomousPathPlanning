"""Wheel + ZED optical odometry + two raw gyros, with optional GPS correction."""
import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler,
)
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def ekf_actions(name, config, clock):
    # The user's 4.x source clone is lifecycle-managed; the Humble 3.x binary
    # is a regular node. Detect the installed package, not the ROS distro.
    manifest = os.path.join(get_package_share_directory('robot_localization'), 'package.xml')
    lifecycle = int(ET.parse(manifest).getroot().findtext('version').split('.')[0]) >= 4
    node_type = LifecycleNode if lifecycle else Node
    node = node_type(
        package='robot_localization', executable='ekf_node', name=name,
        namespace='', output='screen', parameters=[config, {'use_sim_time': clock}],
        remappings=[('odometry/filtered', '/odometry/' + name.removeprefix('ekf_')),
                    ('set_pose', '/' + name + '/set_pose')],
    )
    if not lifecycle:
        return [node]
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=node, goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_ACTIVATE))],
    ))
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node),
        transition_id=Transition.TRANSITION_CONFIGURE))
    return [activate, node, configure]


def start(context):
    def value(name):
        return context.launch_configurations[name]

    clock = value('use_sim_time').lower() == 'true'
    global_enabled = value('enable_global').lower() == 'true'
    config = value('params_file')
    actions = []
    if value('publish_approximate_mounts').lower() == 'true':
        # User-provided layout: 27 inches long, 21 inches wide; Jetson/ICM/GPS
        # centered, camera at front. Heights/rotations are unmeasured. These
        # are explicit bench assumptions, never run alongside URDF mount TF.
        for child, forward in (('imu_link', 0.0), ('gps_link', 0.0),
                               ('zed_camera_link', 0.3429)):
            actions.append(Node(
                package='tf2_ros', executable='static_transform_publisher',
                name='approximate_' + child + '_mount', output='screen',
                arguments=['--x', str(forward), '--y', '0', '--z', '0',
                           '--roll', '0', '--pitch', '0', '--yaw', '0',
                           '--frame-id', 'base_link', '--child-frame-id', child],
            ))
    if global_enabled:
        mode = value('heading_mode')
        if mode not in ('startup', 'imu'):
            raise ValueError('enable_global requires heading_mode:=startup or heading_mode:=imu')
        if mode == 'startup':
            heading = float(value('initial_heading_deg'))
            if not math.isfinite(heading) or not 0 <= heading < 360:
                raise ValueError('Supply measured initial_heading_deg in [0,360): N=0, E=90')
            actions.append(Node(
                package='robot_localization', executable='startup_heading.py',
                name='startup_heading', output='screen',
                parameters=[{'initial_heading_deg': heading, 'use_sim_time': clock}],
            ))
            heading_topic = '/localization/startup_heading'
        else:
            heading_topic = value('heading_topic')
        actions.append(Node(
            package='robot_localization', executable='sensor_inputs.py',
            name='global_sensor_inputs', output='screen', parameters=[{
                'mode': 'gps', 'gps_topic': value('gps_topic'),
                'heading_topic': heading_topic, 'max_age': 2.0, 'use_sim_time': clock,
            }],
        ))
        actions.extend(ekf_actions('ekf_global', config, clock))
        actions.append(Node(
            package='robot_localization', executable='navsat_transform_node',
            name='navsat_transform', output='screen',
            parameters=[config, {'use_sim_time': clock}],
            remappings=[('imu', '/localization/heading'),
                        ('gps/fix', '/gps/validated'),
                        ('odometry/filtered', '/odometry/global'),
                        ('odometry/gps', '/odometry/gps'),
                        ('gps/filtered', '/gps/filtered')],
        ))
    actions.append(Node(
        package='robot_localization', executable='sensor_inputs.py',
        name='dual_sensor_inputs', output='screen', parameters=[{
            'mode': 'dual', 'use_sim_time': clock,
            **{name: value(name) for name in (
                'zed_odom_topic', 'wheel_odom_topic', 'zed_imu_topic', 'icm_imu_topic')},
            **{name: value(name).lower() == 'true' for name in (
                'use_zed_odom', 'use_wheel_odom', 'use_zed_imu', 'use_icm_imu')},
        }],
    ))
    actions.extend(ekf_actions('ekf_local', config, clock))
    return actions


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robot_localization'), 'params', 'dual_ekf.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_global', default_value='false'),
        DeclareLaunchArgument('publish_approximate_mounts', default_value='false',
                              description='Use assumed centered IMU/GPS and front ZED mounts; disable with URDF TF'),
        DeclareLaunchArgument('heading_mode', default_value='unset',
                              description='startup: measured parked heading; imu: verified ENU heading topic'),
        DeclareLaunchArgument('initial_heading_deg', default_value='nan',
                              description='True compass bearing at startup: N=0 E=90 S=180 W=270'),
        DeclareLaunchArgument('heading_topic', default_value='/imu/heading'),
        DeclareLaunchArgument('gps_topic', default_value='/gps/fix'),
        DeclareLaunchArgument('zed_odom_topic', default_value='/zed/zed_node/odom'),
        DeclareLaunchArgument('wheel_odom_topic', default_value='/diff_drive_controller/odom'),
        DeclareLaunchArgument('zed_imu_topic', default_value='/zed/zed_node/imu/data'),
        DeclareLaunchArgument('icm_imu_topic', default_value='/imu/data_raw'),
        DeclareLaunchArgument('use_zed_odom', default_value='true'),
        DeclareLaunchArgument('use_wheel_odom', default_value='true'),
        DeclareLaunchArgument('use_zed_imu', default_value='true'),
        DeclareLaunchArgument('use_icm_imu', default_value='true'),
        OpaqueFunction(function=start),
    ])
