"""Checks for the hardware-free localization smoke-test evaluator."""

import importlib.util
from pathlib import Path
import unittest
import math


PROBE = Path(__file__).resolve().parents[1] / 'scripts' / 'probe_simulation.py'
SPEC = importlib.util.spec_from_file_location('probe_simulation', PROBE)
probe = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(probe)


class SmokeEvaluationTest(unittest.TestCase):
    def setUp(self):
        self.truth = [(0.0, 0.0, 0.0), (0.5, 0.25, 0.0),
                      (1.0, 0.5, 0.0), (1.5, 0.75, 0.0),
                      (2.0, 1.0, 0.0)]
        self.estimate = [(t, x + 0.05, y) for t, x, y in self.truth]

    def test_accepts_moving_estimate_close_to_truth_with_tf(self):
        result = probe.evaluate_samples(self.estimate, self.truth, True)
        self.assertGreaterEqual(result['matched_samples'], 5)
        self.assertLess(result['mean_error_m'], 0.2)

    def test_rejects_missing_tf(self):
        with self.assertRaisesRegex(ValueError, 'odom.*base_link'):
            probe.evaluate_samples(self.estimate, self.truth, False)

    def test_rejects_stationary_output(self):
        stationary = [(t, 0.0, 0.0) for t, _, _ in self.truth]
        with self.assertRaisesRegex(ValueError, 'not moving'):
            probe.evaluate_samples(stationary, self.truth, True)

    def test_rejects_large_position_error(self):
        far = [(t, x + 3.0, y) for t, x, y in self.truth]
        with self.assertRaisesRegex(ValueError, 'error'):
            probe.evaluate_samples(far, self.truth, True)

    def test_rejects_missing_samples(self):
        with self.assertRaisesRegex(ValueError, 'samples'):
            probe.evaluate_samples(self.estimate[:2], self.truth[:2], True)

    def test_rejects_nonfinite_positions(self):
        for bad in (math.nan, math.inf, -math.inf):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                probe.evaluate_samples([(t, bad, y) for t, x, y in self.truth], self.truth, True)

    def test_rejects_repeated_timestamps(self):
        with self.assertRaises(ValueError):
            probe.evaluate_samples([(0., i * .1, 0.) for i in range(5)],
                                   [(0., 0., 0.)] * 5, True)

    def test_rejects_bad_vertical_position(self):
        with self.assertRaisesRegex(ValueError, 'error'):
            probe.evaluate_samples([(t, x, y, -15.) for t, x, y in self.truth],
                                   [(t, x, y, 0.) for t, x, y in self.truth], True)

    def test_health_snapshot_expires_without_new_diagnostics(self):
        state = probe.InputHealth()
        state.update('local_sensor_inputs/camera', True, 10.)
        state.update('local_sensor_inputs/imu', True, 10.)
        self.assertTrue(state.ready(10.1))
        self.assertFalse(state.ready(13.))
        state.update('local_sensor_inputs/camera', True, 13.)
        self.assertFalse(state.ready(13.1))
        state.update('local_sensor_inputs/imu', True, 13.)
        self.assertTrue(state.ready(13.1))

    def test_future_or_failed_health_is_not_ready(self):
        state = probe.InputHealth()
        state.update('local_sensor_inputs/camera', True, 10.)
        state.update('local_sensor_inputs/imu', True, 10.)
        self.assertFalse(state.ready(9.))
        state.update('local_sensor_inputs/imu', False, 10.)
        self.assertFalse(state.ready(10.1))

    def test_rejects_old_output_even_when_input_health_is_fresh(self):
        with self.assertRaisesRegex(ValueError, 'stale'):
            probe.evaluate_samples(self.estimate, self.truth, True, now=10.)


if __name__ == '__main__':
    unittest.main()
