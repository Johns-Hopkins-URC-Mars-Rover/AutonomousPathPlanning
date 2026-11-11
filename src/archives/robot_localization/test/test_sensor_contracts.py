"""ROS-free regression tests for the actual sensor-contract math."""
import importlib.util
import math
from pathlib import Path
import unittest

PATH = Path(__file__).resolve().parents[1] / 'scripts' / 'sensor_contracts.py'


class SensorContractsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('sensor_contracts', PATH)
        cls.c = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.c)

    def test_camera_mount_is_removed_in_world_axes(self):
        # Camera is 1 m ahead of base; camera world yaw is 90 degrees.
        q = (0., 0., math.sqrt(.5), math.sqrt(.5))
        p, qb, cov = self.c.base_pose((2., 3., 0.), q, (-1., 0., 0.),
                                     (0., 0., 0., 1.), self.c.diagonal([1.] * 6))
        for actual, expected in zip(p, (2., 2., 0.)):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(qb[2], q[2])
        self.assertGreater(cov[0], 1.)  # lever-arm attitude uncertainty propagated

    def test_rotated_mount_changes_orientation(self):
        q = (0., 0., math.sqrt(.5), math.sqrt(.5))
        p, qb, cov = self.c.base_pose((0., 0., 0.), q, (0., 0., 0.),
                                     (0., 0., -q[2], q[3]), self.c.diagonal([1.] * 6))
        self.assertAlmostEqual(qb[2], 0.)
        self.assertAlmostEqual(qb[3], 1.)

    def test_rejects_bad_covariance_and_quaternion(self):
        for bad in ([0.] * 9, [math.nan] * 9, self.c.diagonal([-1., 1., 1.]),
                    [1., 2., 0., 2., 1., 0., 0., 0., 1.],
                    [1., .2, 0., 0., 1., 0., 0., 0., 1.]):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                self.c.validate_covariance(bad, 3)
        with self.assertRaises(ValueError):
            self.c.normalize_quaternion((0., 0., 0., 0.))
        with self.assertRaises(ValueError):
            self.c.normalize_quaternion((0., 0., 0., 2.))

    def test_age_and_order_must_be_valid(self):
        self.c.validate_stamp(10., 10.1, 9.)
        for stamp, now, previous in ((10., 12., 9.), (12., 10., 9.),
                                     (10., 10., 10.), (0., 0., None)):
            with self.assertRaises(ValueError):
                self.c.validate_stamp(stamp, now, previous)

    def test_gps_requires_fix_covariance_and_finite_altitude(self):
        cov = self.c.diagonal([4., 4., 9.])
        self.c.validate_fix(0, 39., -76., 50., 2, cov)
        for changes in ((-1,39.,-76.,50.,2,cov), (0,39.,-76.,math.nan,2,cov),
                        (0,39.,-76.,50.,0,cov), (0,100.,-76.,50.,2,cov)):
            with self.assertRaises(ValueError):
                self.c.validate_fix(*changes)

    def test_manual_compass_bearing_converts_to_enu(self):
        for compass, yaw in ((0., math.pi/2), (90., 0.),
                             (180., -math.pi/2), (270., -math.pi)):
            actual = self.c.compass_to_enu(compass)
            self.assertAlmostEqual(math.sin(actual), math.sin(yaw))
            self.assertAlmostEqual(math.cos(actual), math.cos(yaw))
        for value in (math.nan, math.inf, -1., 360.):
            with self.assertRaises(ValueError):
                self.c.compass_to_enu(value)


if __name__ == '__main__':
    unittest.main()
