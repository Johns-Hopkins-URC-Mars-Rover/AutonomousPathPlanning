"""Publish measured mounts on a headless sensor host; no placeholder transforms."""
import json
import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def mount_nodes(context):
    path = LaunchConfiguration('mounts_file').perform(context)
    with open(path, encoding='utf-8') as stream:
        mounts = json.load(stream)
    if mounts.get('verified') is not True:
        raise RuntimeError('Mount file must contain verified: true after physical measurement')
    transforms = mounts.get('transforms', [])
    if not transforms:
        raise ValueError('Mount file contains no transforms')
    nodes, children = [], set()
    for index, mount in enumerate(transforms):
        parent, child = mount['parent'], mount['child']
        xyz, rpy = mount['xyz'], mount['rpy']
        if parent != 'base_link' or child not in ('zed_camera_link', 'gps_link', 'imu_link') or child in children:
            raise ValueError('Mounts must have unique sensor children and parent base_link')
        if len(xyz) != 3 or len(rpy) != 3 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in xyz+rpy):
            raise ValueError('Mount xyz [m] and rpy [rad] must each have 3 measured finite values')
        children.add(child)
        args = []
        for key, val in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), xyz+rpy):
            args.extend(['--' + key, str(val)])
        args += ['--frame-id', parent, '--child-frame-id', child]
        nodes.append(Node(package='tf2_ros', executable='static_transform_publisher',
                          name='measured_mount_' + str(index), arguments=args))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mounts_file', description='Absolute path to measured mount JSON'),
        OpaqueFunction(function=mount_nodes),
    ])
