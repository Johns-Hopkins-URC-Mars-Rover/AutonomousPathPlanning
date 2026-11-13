"""Validate installed hardware launch guards using ROS launch APIs."""
import importlib.util
from pathlib import Path
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext


class HardwareLaunchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(get_package_share_directory('rover_localization')) / 'launch' / 'hardware_localization.launch.py'
        spec = importlib.util.spec_from_file_location('hardware_launch_under_test', path)
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def context(self, **overrides):
        context = LaunchContext()
        context.launch_configurations.update({
            'use_sim_time': 'false', 'enable_global': 'true',
            'params_file': 'unused.yaml', 'publish_approximate_mounts': 'false',
            'heading_mode': 'unset', 'initial_heading_deg': 'nan',
        })
        context.launch_configurations.update(overrides)
        return context

    def test_global_requires_explicit_heading_source(self):
        with self.assertRaisesRegex(ValueError, 'heading_mode'):
            self.module.start(self.context())

    def test_startup_requires_measured_finite_compass_bearing(self):
        for value in ('nan', 'inf', '-1', '360'):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'initial_heading_deg'):
                self.module.start(self.context(heading_mode='startup', initial_heading_deg=value))

    def test_helpers_and_configuration_are_installed(self):
        share = Path(get_package_share_directory('rover_localization'))
        self.assertTrue((share / 'config' / 'hardware_ekf.yaml').is_file())
        self.assertTrue((share / 'config' / 'zed_optical.yaml').is_file())
        prefix = share.parents[1]
        for name in ('sensor_inputs.py', 'sensor_contracts.py', 'startup_heading.py', 'zed_imu_tf.py'):
            self.assertTrue((prefix / 'lib' / 'rover_localization' / name).is_file(), name)


if __name__ == '__main__':
    unittest.main()
