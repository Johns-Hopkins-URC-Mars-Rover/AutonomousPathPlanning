"""Evaluate launch construction with lightweight ROS launch API stand-ins."""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

SRC = Path(__file__).resolve().parents[2]


class Action:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


class Configuration:
    def __init__(self, name):
        self.name = name
    def perform(self, context):
        return context[self.name]


def load(relative):
    modules = {}
    mapping = {
        'launch': {'LaunchDescription': lambda items: items},
        'launch.actions': {name: Action for name in ('DeclareLaunchArgument', 'IncludeLaunchDescription', 'OpaqueFunction', 'GroupAction', 'SetEnvironmentVariable', 'TimerAction')},
        'launch.conditions': {'IfCondition': Action},
        'launch.launch_description_sources': {'PythonLaunchDescriptionSource': Action},
        'launch.substitutions': {'LaunchConfiguration': Configuration, 'PathJoinSubstitution': Action, 'PythonExpression': Action},
        'launch_ros.actions': {'Node': Action, 'SetRemap': Action},
        'launch_ros.parameter_descriptions': {'ParameterValue': Action},
        'launch_ros.substitutions': {'FindPackageShare': lambda name: '/share/' + name},
        'ament_index_python.packages': {'get_package_share_directory': lambda name: str(SRC / name)},
    }
    for name, contents in mapping.items():
        module = types.ModuleType(name)
        module.__dict__.update(contents)
        modules[name] = module
    path = SRC / relative
    with patch.dict(sys.modules, modules):
        spec = importlib.util.spec_from_file_location('launch_under_test', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module


class LaunchContractsTest(unittest.TestCase):
    def test_bringup_does_not_invert_camera_mount_or_double_publish_motion_tf(self):
        actions = load('rover_bringup/launch/rover.launch.py').generate_launch_description()
        nodes = [a for a in actions if a.kwargs.get('executable') == 'static_transform_publisher']
        self.assertEqual(nodes, [], 'sensor mounts need a single measured description owner')
        actions = load('rover_localization/launch/zed_driver.launch.py').generate_launch_description()
        includes = [a for a in actions if 'launch_arguments' in a.kwargs]
        arguments = [dict(a.kwargs['launch_arguments']) for a in includes]
        self.assertTrue(any(a.get('publish_tf') == 'false' and 'ros_params_override_path' in a for a in arguments))

    def test_gps_fusion_refuses_unverified_heading_and_mounts(self):
        mod = load('rover_localization/launch/gps_localization.launch.py')
        self.assertEqual(mod.start_global({'enable_gps_fusion': 'false'}), [])
        context = {'enable_gps_fusion': 'true', 'heading_verified': 'false', 'mounts_verified': 'false'}
        with self.assertRaisesRegex(RuntimeError, 'heading'):
            mod.start_global(context)
        context.update(heading_verified='true', mounts_verified='true')
        actions = mod.start_global(context)
        self.assertEqual(len(actions), 3)
        navsat = next(a for a in actions if a.kwargs.get('executable') == 'navsat_transform_node')
        self.assertIn(('gps/fix', '/gps/validated'), navsat.kwargs['remappings'])

    def test_local_ekf_uses_validated_inputs(self):
        actions = load('rover_localization/launch/localization.launch.py').generate_launch_description()
        ekf = next(a for a in actions if a.kwargs.get('executable') == 'ekf_node')
        params = ekf.kwargs['parameters'][1]
        self.assertEqual(params['odom0'], '/localization/zed_odom_base')
        self.assertEqual(params['imu0'], '/localization/imu')

    def test_mounts_require_measurements_and_reject_duplicate_children(self):
        import tempfile
        import json
        mod = load('rover_description/launch/sensor_mounts.launch.py')
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'mounts.json'
            path.write_text(json.dumps({'verified': False}))
            with self.assertRaises(RuntimeError): mod.mount_nodes({'mounts_file': str(path)})
            mount = {'parent': 'base_link', 'child': 'gps_link', 'xyz': [0.,0.,.5], 'rpy': [0.,0.,0.]}
            path.write_text(json.dumps({'verified': True, 'transforms': [mount]}))
            nodes = mod.mount_nodes({'mounts_file': str(path)})
            self.assertEqual(len(nodes), 1)
            self.assertEqual(nodes[0].kwargs['arguments'][-1], 'gps_link')
            path.write_text(json.dumps({'verified': True, 'transforms': [mount, mount]}))
            with self.assertRaises(ValueError): mod.mount_nodes({'mounts_file': str(path)})

    def test_combined_workflow_owns_one_local_estimator(self):
        actions = load('rover_description/launch/gps_nav_launch.py').generate_launch_description()
        actions = actions + [nested for a in actions for nested in a.kwargs.get('actions', [])]
        includes = [a for a in actions if 'launch_arguments' in a.kwargs]
        self.assertEqual(len(includes), 3)
        self.assertEqual(sum('fused_imu_topic' in dict(a.kwargs['launch_arguments']) for a in includes), 1)


if __name__ == '__main__':
    unittest.main()
