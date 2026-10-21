#!/usr/bin/env python3
"""Check that the synthetic EKF publishes moving odometry and a usable TF."""

import argparse
import math
import time


def evaluate_samples(estimated, truth, tf_seen):
    """Validate timestamped (seconds, x, y) samples without needing ROS imports."""
    if len(estimated) < 5 or len(truth) < 5:
        raise ValueError('not enough odometry or ground-truth samples')
    if not tf_seen:
        raise ValueError('odom -> base_link transform was not observed')

    displacement = math.dist(estimated[0][1:], estimated[-1][1:])
    if displacement < 0.2:
        raise ValueError('filtered odometry is not moving')

    errors = []
    for stamp, x, y in estimated:
        nearest = min(truth, key=lambda sample: abs(sample[0] - stamp))
        if abs(nearest[0] - stamp) <= 0.2:
            errors.append(math.dist((x, y), nearest[1:]))
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
    if args.timeout_sec <= 0:
        parser.error('--timeout-sec must be positive')

    # Keep ROS imports here so the evaluator can be unit-tested without ROS.
    import rclpy
    from nav_msgs.msg import Odometry
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
            self.truth_subscription = self.create_subscription(
                Odometry, '/simulation/ground_truth',
                lambda msg: self.record(self.truth, msg), 10)

        @staticmethod
        def record(samples, msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            samples.append((stamp, msg.pose.pose.position.x, msg.pose.pose.position.y))
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
                node.buffer.lookup_transform('odom', 'base_link', Time())
                node.tf_seen = True
            except TransformException:
                pass
            try:
                result = evaluate_samples(node.estimated, node.truth, node.tf_seen)
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
