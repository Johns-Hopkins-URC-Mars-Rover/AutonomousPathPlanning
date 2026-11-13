import random

import pytest

from icm20948_imu_interface.gyro_bias import GyroBiasCalibrator


def _feed(cal, samples):
    finished = False
    for s in samples:
        finished = cal.add_sample(*s) or finished
    return finished


def test_averages_still_samples():
    random.seed(0)
    true_bias = (0.0105, -0.004, 0.002)
    cal = GyroBiasCalibrator(num_samples=250)
    samples = [tuple(b + random.gauss(0, 0.002) for b in true_bias)
               for _ in range(250)]
    assert _feed(cal, samples)
    assert cal.done
    for got, want in zip(cal.bias, true_bias):
        assert got == pytest.approx(want, abs=1e-3)


def test_not_done_before_enough_samples():
    cal = GyroBiasCalibrator(num_samples=10)
    _feed(cal, [(0.01, 0.0, 0.0)] * 9)
    assert not cal.done
    assert cal.bias is None


def test_high_spread_restarts_and_discards():
    cal = GyroBiasCalibrator(num_samples=10, max_std=0.05)
    shaking = [(0.5 if i % 2 else -0.5, 0.0, 0.0) for i in range(10)]
    assert not _feed(cal, shaking)
    assert not cal.done
    assert cal.retries == 1
    # A clean window afterwards succeeds and ignores the shaking samples.
    assert _feed(cal, [(0.01, 0.0, 0.0)] * 10)
    assert cal.bias == pytest.approx((0.01, 0.0, 0.0))


def test_steady_rotation_is_rejected():
    # Constant spin has zero spread but a large mean: not a bias.
    cal = GyroBiasCalibrator(num_samples=10, max_mean=0.1)
    assert not _feed(cal, [(0.0, 0.0, 0.5)] * 10)
    assert not cal.done
    assert cal.retries == 1


def test_ignores_samples_after_done():
    cal = GyroBiasCalibrator(num_samples=5)
    _feed(cal, [(0.01, 0.0, 0.0)] * 5)
    bias = cal.bias
    assert not cal.add_sample(1.0, 1.0, 1.0)
    assert cal.bias == bias
