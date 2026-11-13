import math
import random

import pytest

from l76x_gps_interface.gps_error import estimate_variance, to_local_en


def _cloud(n, sd_e, sd_n, seed=0, offset=(0.0, 0.0)):
    r = random.Random(seed)
    return [(offset[0] + r.gauss(0, sd_e), offset[1] + r.gauss(0, sd_n)) for _ in range(n)]


def test_variance_matches_scatter_with_margin():
    res = estimate_variance(_cloud(400, 3.0, 2.0), inflation=2.0, min_var=0.1)
    assert res['ok']
    e, n, u = res['variance']
    assert e == n
    assert e == pytest.approx(2 * 9.0, rel=0.25)  # worst axis (3 m) squared, doubled
    assert u >= 2 * e


def test_floor_and_ceiling():
    quiet = estimate_variance(_cloud(100, 0.1, 0.1), min_var=4.0)
    assert quiet['variance'][0] == 4.0
    wild = estimate_variance(_cloud(100, 12.0, 12.0), max_var=100.0, max_drift_m=1e9)
    assert wild['variance'][0] == 100.0


def test_moving_rover_is_rejected():
    walk = [(0.5 * i, 0.0) for i in range(120)]  # 60 m over the window
    res = estimate_variance(walk, max_drift_m=15.0)
    assert not res['ok']
    assert 'moved' in res['reason']


def test_too_few_samples():
    assert not estimate_variance([(0.0, 0.0)] * 5)['ok']


def test_correlated_axes_use_worst_direction():
    # Scatter along the diagonal: worst-direction variance exceeds either axis.
    r = random.Random(1)
    pts = []
    for _ in range(500):
        t = r.gauss(0, 4.0)
        pts.append((t + r.gauss(0, 0.3), t + r.gauss(0, 0.3)))
    res = estimate_variance(pts, inflation=1.0, min_var=0.1)
    assert res['variance'][0] > 1.6 * res['std_e'] ** 2


def test_to_local_en_scale():
    en = to_local_en([(39.0, -76.0), (39.0001, -76.0), (39.0, -75.9999)])
    assert en[1][1] == pytest.approx(11.13, abs=0.05)          # 1e-4 deg north
    assert en[2][0] == pytest.approx(11.13 * math.cos(math.radians(39)), abs=0.05)
