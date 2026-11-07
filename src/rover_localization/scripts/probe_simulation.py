#!/usr/bin/env python3
"""Check that the synthetic EKF publishes moving odometry and a usable TF."""

import argparse
import math
import time


class InputHealth:
    """Both input streams must have their own recent healthy diagnostic."""
    names = ('local_sensor_inputs/camera', 'local_sensor_inputs/imu')

    def __init__(self):
        self.snapshots = {}

    def update(self, name, healthy, stamp):
        if name in self.names:
            self.snapshots[name] = (healthy, stamp)

    def ready(self, now, max_age=2.):
        return all(name in self.snapshots and self.snapshots[name][0] and
                   0 <= now - self.snapshots[name][1] <= max_age for name in self.names)


def evaluate_samples(estimated, truth, tf_seen, now=None):
    """Validate timestamped (seconds, x, y[, z]) samples without needing ROS imports."""
    if len(estimated) < 5 or len(truth) < 5:
        raise ValueError('not enough odometry or ground-truth samples')
    if not tf_seen:
        raise ValueError('odom -> base_link transform was not observed')

    for samples in (estimated, truth):
        if any(not all(math.isfinite(v) for v in sample) for sample in samples):
            raise ValueError('nonfinite sample')
        if any(b[0] <= a[0] for a, b in zip(samples, samples[1:])):
            raise ValueError('timestamps must increase strictly')
        if samples[-1][0] - samples[0][0] < 1.0:
            raise ValueError('samples must span at least one second')
    if len({len(sample) for sample in estimated + truth}) != 1:
        raise ValueError('position dimensions must match')

    if now is not None and any(not 0 <= now - samples[-1][0] <= .5
                               for samples in (estimated, truth)):
        raise ValueError('stale or future odometry/ground-truth output')

    displacement = math.dist(estimated[0][1:], estimated[-1][1:])
    if displacement < 0.2:
        raise ValueError('filtered odometry is not moving')

    errors = []
    for sample in estimated:
        stamp, position = sample[0], sample[1:]
        nearest = min(truth, key=lambda sample: abs(sample[0] - stamp))
        if abs(nearest[0] - stamp) <= 0.2:
            errors.append(math.dist(position, nearest[1:]))
    if len(errors) < 5:
        raise ValueError('not enough time-matched odometry samples')

    mean_error = sum(errors) / len(errors)
    if mean_error > 1.0:
        raise ValueError(f'mean position error too large: {mean_error:.2f} m')

    return {'matched_samples': len(errors), 'mean_error_m': mean_error}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout-sec', type=float, default=40.0)
    args = parser.parse_args()
    if not math.isfinite(args.timeout_sec) or args.timeout_sec <= 0:
        parser.error('--timeout-sec must be positive')

    # Keep ROS imports here so the evaluator can be unit-tested without ROS.
    import rclpy
    from nav_msgs.msg import Odometry
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.node import Node
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    class Probe(Node):
        def __init__(self):
            super().__init__('localization_smoke_probe')
            self.estimated = []
            self.truth = []
            self.tf_seen = False
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.odom_subscription = self.create_subscription(
                Odometry, '/odometry/local',
                lambda msg: self.record(self.estimated, msg), 10)
            self.diag_subscription = self.create_subscription(
                DiagnosticArray, '/diagnostics', self.diagnostics, 10)
            self.input_health = InputHealth()
            self.truth_subscription = self.create_subscription(
                Odometry, '/simulation/ground_truth',
                lambda msg: self.record(self.truth, msg), 10)

        def diagnostics(self, msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            for status in msg.status:
                values = {v.key: v.value for v in status.values}
                try:
                    healthy = status.level == 0 and int(values.get('accepted', '0')) >= 5
                except ValueError:
                    healthy = False
                self.input_health.update(status.name, healthy, stamp)

        @staticmethod
        def record(samples, msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            samples.append((stamp, msg.pose.pose.position.x, msg.pose.pose.position.y,
                            msg.pose.pose.position.z))
            if len(samples) > 500:
                del samples[:len(samples) - 500]

    rclpy.init()
    node = Probe()
    deadline = time.monotonic() + args.timeout_sec
    last_error = 'waiting for messages'
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            try:
                tf = node.buffer.lookup_transform('odom', 'base_link', Time())
                stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
                age = node.get_clock().now().nanoseconds * 1e-9 - stamp
                node.tf_seen = 0 <= age <= .5
            except TransformException:
                node.tf_seen = False
            now = node.get_clock().now().nanoseconds * 1e-9
            if not node.input_health.ready(now):
                last_error = 'camera and IMU have not both supplied fresh accepted measurements; inspect /diagnostics'
                continue
            try:
                result = evaluate_samples(node.estimated, node.truth, node.tf_seen, now=now)
            except ValueError as exc:
                last_error = str(exc)
                continue
            print('PASS: /odometry/local moves and follows ground truth; '
                  f"{result['matched_samples']} matched samples, "
                  f"mean error {result['mean_error_m']:.2f} m; "
                  'odom -> base_link TF present')
            return 0
        print(f'FAIL: localization smoke test timed out: {last_error}')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
