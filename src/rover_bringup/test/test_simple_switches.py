"""Configuration behavior for the simple rover launch."""
import importlib.util
from pathlib import Path
import unittest

path = Path(__file__).resolve().parents[1] / 'launch' / 'simple_switches.py'
spec = importlib.util.spec_from_file_location('simple_switches', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class SimpleSwitchTests(unittest.TestCase):
    def config(self, **overrides):
        values = dict(zed='yes', icm='yes', gps='yes', wheel='yes',
                      localization='yes', drivers='yes')
        values.update(overrides)
        return module.launch_arguments(values)

    def test_full_configuration(self):
        args = self.config()
        for key in ('enable_zed', 'enable_imu', 'enable_gps', 'enable_localization',
                    'enable_global', 'use_zed_odom', 'use_zed_imu',
                    'use_icm_imu', 'use_wheel_odom'):
            self.assertEqual(args[key], 'true', key)

    def test_icm_no_disables_driver_and_fusion(self):
        args = self.config(icm='no')
        self.assertEqual(args['enable_imu'], 'false')
        self.assertEqual(args['use_icm_imu'], 'false')
        self.assertEqual(args['use_zed_imu'], 'true')

    def test_components_can_be_disabled_independently(self):
        args = self.config(zed='no', gps='no', wheel='no', localization='no')
        for key in ('enable_zed', 'enable_gps', 'enable_global',
                    'enable_localization', 'use_zed_odom', 'use_zed_imu',
                    'use_wheel_odom'):
            self.assertEqual(args[key], 'false', key)
        self.assertEqual(args['use_icm_imu'], 'true')

    def test_external_drivers_preserve_requested_inputs(self):
        args = self.config(drivers='no')
        for key in ('enable_zed', 'enable_imu', 'enable_gps'):
            self.assertEqual(args[key], 'false')
        for key in ('use_zed_imu', 'use_icm_imu', 'use_wheel_odom', 'enable_global'):
            self.assertEqual(args[key], 'true')

    def test_rejects_ambiguous_values(self):
        for value in ('maybe', '', 'on', 'TRUEE'):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'icm'):
                self.config(icm=value)

    def test_ros_boolean_aliases(self):
        args = self.config(icm='FALSE', wheel='1', gps='0')
        self.assertEqual(args['use_icm_imu'], 'false')
        self.assertEqual(args['use_wheel_odom'], 'true')
        self.assertEqual(args['enable_global'], 'false')


if __name__ == '__main__':
    unittest.main()
