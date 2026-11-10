"""Noise generators must perturb measurements while keeping truth independent."""
import importlib.util
import math
from pathlib import Path
import unittest

PATH = Path(__file__).with_name('noisy_sensor_model.py')


class NoisySensorModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('noisy_sensor_model', PATH)
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def test_measurements_are_seeded_but_not_perfect(self):
        a = self.module.NoiseModel(197, 'noisy')
        b = self.module.NoiseModel(197, 'noisy')
        c = self.module.NoiseModel(211, 'noisy')
        self.assertEqual(a.camera(1.), b.camera(1.))
        self.assertNotEqual(a.camera(1.1)['position'], c.camera(1.1)['position'])
        self.assertEqual(a.truth(1.)[0], (0., 0., 0.))
        self.assertNotEqual(a.imu(1.)['omega'], (0., 0., 0.))

    def test_covariance_matches_declared_white_noise(self):
        model = self.module.NoiseModel(197, 'noisy')
        sample = model.camera(1.)
        self.assertAlmostEqual(sample['covariance'][0], model.position_sigma**2)
        self.assertAlmostEqual(model.imu(1.)['covariance'][0], model.gyro_sigma**2)
        xs = [model.camera(1.+i/30.)['position'][0] for i in range(25)]
        self.assertGreater(max(xs)-min(xs), .03)

    def test_camera_fault_recovers_without_corrupting_truth(self):
        model = self.module.NoiseModel(197, 'tracking_fault')
        self.assertLess(model.camera(5.5)['position'][2], -10.)
        self.assertLess(abs(model.camera(9.)['position'][2]), .5)
        self.assertEqual(model.truth(5.5)[0][2], 0.)
        self.assertEqual(model.truth(11.)[0], (0., 0., 0.))

    def test_profiles_and_latency_are_bounded(self):
        with self.assertRaises(ValueError):
            self.module.NoiseModel(197, 'unknown')
        model = self.module.NoiseModel(197, 'noisy')
        for i in range(100):
            self.assertTrue(0 <= model.latency('camera') <= .08)
            self.assertTrue(0 <= model.latency('imu') <= .015)
        with self.assertRaises(ValueError):
            model.latency('bad_sensor')


if __name__ == '__main__':
    unittest.main()
