"""Startup gyro bias calibration for the ICM-20948 (no ROS or hardware deps)."""

import math


class GyroBiasCalibrator:
    """Estimate gyro bias from samples taken while the rover sits still.

    Feed it one (x, y, z) gyro sample in rad/s per call to ``add_sample``.
    After ``num_samples`` samples it checks that the rover was actually still.
    If so, ``bias`` is set and ``done`` becomes True. If not, the samples are
    discarded and collection restarts, so a bad bias is never stored.

    A window counts as "moving" when, on any axis, the standard deviation
    exceeds ``max_std`` (rad/s) or the mean magnitude exceeds ``max_mean``
    (rad/s).
    """

    def __init__(self, num_samples=250, max_std=0.05, max_mean=0.1):
        if num_samples < 2:
            raise ValueError("num_samples must be at least 2")
        self.num_samples = int(num_samples)
        self.max_std = float(max_std)
        self.max_mean = float(max_mean)
        self.bias = None
        self.retries = 0
        self._reset()

    @property
    def done(self):
        return self.bias is not None

    def _reset(self):
        self._n = 0
        self._sum = [0.0, 0.0, 0.0]
        self._sum_sq = [0.0, 0.0, 0.0]

    @property
    def count(self):
        """Samples collected in the current window."""
        return self._n

    def running_stats(self):
        """(mean, std) per axis of the current window, or None if empty."""
        if self._n == 0:
            return None
        n = float(self._n)
        mean = [s / n for s in self._sum]
        std = [math.sqrt(max(sq / n - m * m, 0.0))
               for sq, m in zip(self._sum_sq, mean)]
        return mean, std

    def add_sample(self, gx, gy, gz):
        """Add a sample. Returns True once, on the call that finishes calibration."""
        if self.done:
            return False
        for i, v in enumerate((gx, gy, gz)):
            self._sum[i] += v
            self._sum_sq[i] += v * v
        self._n += 1
        if self._n < self.num_samples:
            return False

        n = float(self._n)
        mean = [s / n for s in self._sum]
        std = [math.sqrt(max(sq / n - m * m, 0.0))
               for sq, m in zip(self._sum_sq, mean)]
        moving = (max(std) > self.max_std
                  or max(abs(m) for m in mean) > self.max_mean)
        if moving:
            self.retries += 1
            self.last_mean = mean
            self.last_std = std
            self._reset()
            return False
        self.bias = tuple(mean)
        self.last_std = std
        return True
