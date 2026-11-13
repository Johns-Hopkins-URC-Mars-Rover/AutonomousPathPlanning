"""Estimate GPS position variance from a stationary recording (no ROS deps)."""

import math

EARTH_RADIUS_M = 6378137.0


def to_local_en(samples, origin=None):
    """Convert (lat, lon) degrees to east/north metres about ``origin``."""
    if origin is None:
        origin = samples[0]
    lat0, lon0 = origin
    scale = EARTH_RADIUS_M * math.cos(math.radians(lat0))
    return [(scale * math.radians(lon - lon0),
             EARTH_RADIUS_M * math.radians(lat - lat0)) for lat, lon in samples]


def _mean(values):
    return sum(values) / len(values)


def _var(values):
    m = _mean(values)
    return sum((v - m) ** 2 for v in values) / (len(values) - 1)


def estimate_variance(en, up=None, inflation=2.0, min_var=4.0, max_var=100.0,
                      max_drift_m=15.0):
    """Turn stationary east/north (and optional up) samples into a variance.

    The driver publishes a diagonal covariance, so east and north share one
    value: the largest eigenvalue of the 2x2 sample covariance, which covers
    the worst horizontal direction. A short window underestimates slow GPS
    wander, so the result is multiplied by ``inflation`` and clamped to
    [min_var, max_var]. If the first and last thirds of the window are more
    than ``max_drift_m`` apart the rover probably moved, and the result is
    rejected instead of being reported as GPS error.

    Returns a dict with ``ok``, ``reason``, ``variance`` ([e, n, u] in m^2
    when ok), plus the raw figures for logging.
    """
    n = len(en)
    if n < 10:
        return {'ok': False, 'reason': 'need at least 10 fixes, got %d' % n}
    e = [p[0] for p in en]
    nn = [p[1] for p in en]
    third = max(n // 3, 1)
    drift = math.hypot(_mean(e[-third:]) - _mean(e[:third]),
                       _mean(nn[-third:]) - _mean(nn[:third]))
    var_e, var_n = _var(e), _var(nn)
    m_e, m_n = _mean(e), _mean(nn)
    cov_en = sum((a - m_e) * (b - m_n) for a, b in zip(e, nn)) / (n - 1)
    half_trace = 0.5 * (var_e + var_n)
    radius = math.sqrt(0.25 * (var_e - var_n) ** 2 + cov_en ** 2)
    raw = half_trace + radius  # largest eigenvalue of the 2x2 covariance
    result = {'n': n, 'std_e': math.sqrt(var_e), 'std_n': math.sqrt(var_n),
              'raw_var': raw, 'drift_m': drift}
    if drift > max_drift_m:
        result.update(ok=False, reason='rover appears to have moved (%.1f m drift '
                      'between start and end of the window)' % drift)
        return result
    horizontal = min(max(raw * inflation, min_var), max_var)
    if up is not None and len(up) == n:
        vertical = max(_var(up) * inflation, 2.0 * horizontal)
    else:
        vertical = 4.0 * horizontal
    vertical = min(vertical, 4.0 * max_var)
    result.update(ok=True, reason='ok', variance=[horizontal, horizontal, vertical])
    return result
