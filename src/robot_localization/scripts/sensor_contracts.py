"""ROS-free input validation and rigid pose conversion (no state estimator)."""
import math


def compass_to_enu(compass_degrees):
    """Convert clockwise-from-true-north degrees to CCW-from-east radians."""
    if not math.isfinite(compass_degrees) or not 0.0 <= compass_degrees < 360.0:
        raise ValueError('initial_heading_deg must be measured in [0, 360): N=0, E=90')
    return math.remainder(math.pi / 2.0 - math.radians(compass_degrees), 2.0 * math.pi)


def diagonal(values):
    return [float(v) if r == c else 0. for r, v in enumerate(values)
            for c in range(len(values))]


def validate_covariance(values, size):
    if len(values) != size * size or not all(math.isfinite(v) for v in values):
        raise ValueError('covariance must have finite entries of the expected size')
    if any(values[i * size + i] <= 0 for i in range(size)):
        raise ValueError('covariance diagonal must be positive; unknown covariance is unusable')
    scale = max(abs(v) for v in values)
    if any(abs(values[r * size + c] - values[c * size + r]) > 1e-6 * scale
           for r in range(size) for c in range(size)):
        raise ValueError('covariance must be symmetric')
    # Semidefinite Cholesky, allowing float-derived rounding at the PSD boundary.
    lower = [[0.] * size for _ in range(size)]
    tolerance = 1e-8 * scale
    for r in range(size):
        for c in range(r + 1):
            residual = .5 * (values[r * size + c] + values[c * size + r])
            residual -= sum(lower[r][k] * lower[c][k] for k in range(c))
            if r == c:
                if residual < -tolerance:
                    raise ValueError('covariance must be positive semidefinite')
                lower[r][c] = math.sqrt(max(0., residual))
            elif lower[c][c] > math.sqrt(tolerance):
                lower[r][c] = residual / lower[c][c]
            elif abs(residual) > tolerance:
                raise ValueError('covariance must be positive semidefinite')


def validate_stamp(stamp, now, previous=None, max_age=.5, future_tolerance=.05):
    if not all(math.isfinite(v) for v in (stamp, now)) or stamp <= 0:
        raise ValueError('timestamp is zero or nonfinite')
    if now - stamp > max_age:
        raise ValueError('stale input: check sensor latency and clock synchronization')
    if stamp - now > future_tolerance:
        raise ValueError('future input: synchronize Jetson and ROS-machine clocks')
    if previous is not None and stamp <= previous:
        raise ValueError('input timestamp repeated or moved backwards')


def normalize_quaternion(q):
    if len(q) != 4 or not all(math.isfinite(v) for v in q):
        raise ValueError('orientation must be a finite quaternion')
    norm = math.sqrt(sum(v * v for v in q))
    if abs(norm - 1.) > .01:
        raise ValueError('orientation quaternion is not normalized')
    return tuple(v / norm for v in q)


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def rotate(q, vector):
    q = normalize_quaternion(q)
    return multiply(multiply(q, (*vector, 0.)), (-q[0], -q[1], -q[2], q[3]))[:3]


def base_pose(position, orientation, camera_to_base_position, camera_to_base_orientation, covariance):
    """Compose T_world_camera * T_camera_base; propagate small world-axis errors.

    The 6x6 covariance uses position + small fixed-axis orientation errors.
    Mount calibration uncertainty is not included; mounts must be rigid/verified.
    Twist is deliberately not reused from the same camera measurements.
    """
    if not all(math.isfinite(v) for v in (*position, *camera_to_base_position)):
        raise ValueError('pose contains a nonfinite position')
    q = normalize_quaternion(orientation)
    qb = normalize_quaternion(camera_to_base_orientation)
    validate_covariance(covariance, 6)
    lever = rotate(q, camera_to_base_position)
    x, y, z = lever
    negative_skew = ((0., z, -y), (-z, 0., x), (y, -x, 0.))
    jac = [[float(r == c) for c in range(6)] for r in range(6)]
    for r in range(3):
        for c in range(3):
            jac[r][c+3] = negative_skew[r][c]
    result = [sum(jac[r][i] * covariance[i*6+j] * jac[c][j]
                  for i in range(6) for j in range(6))
              for r in range(6) for c in range(6)]
    return tuple(a+b for a, b in zip(position, lever)), multiply(q, qb), result


def validate_fix(status, latitude, longitude, altitude, covariance_type, covariance):
    if status < 0:
        raise ValueError('GPS has no fix')
    if not all(math.isfinite(v) for v in (latitude, longitude, altitude)):
        raise ValueError('GPS needs finite latitude, longitude and ellipsoid altitude (RMC alone is insufficient)')
    if not -90 <= latitude <= 90 or not -180 <= longitude <= 180:
        raise ValueError('GPS coordinates are outside WGS84 bounds')
    if covariance_type == 0:
        raise ValueError('GPS covariance is unknown; configure measured uncertainty before fusion')
    validate_covariance(covariance, 3)
