#!/usr/bin/env python3
"""Record what the localization stack outputs during a fake-sensor run.

Writes to ``out_dir``:
  estimates.csv    5 Hz: local EKF, global EKF and navsat GPS odometry positions/yaw
  diagnostics.txt  last diagnostics status of every node (levels + messages)
  topic_rates.txt  message counts and rates for each key topic
Stops 2 s after fake_sensors.py writes <out_dir>/done (or after ``duration_s`` if set).
"""
import csv
import math
import os
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix

LEVEL = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}


def yaw_deg(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


class Recorder(Node):
    def __init__(self):
        super().__init__('run_recorder')
        self.out_dir = self.declare_parameter('out_dir', '.').value
        self.duration = self.declare_parameter('duration_s', 0.0).value
        os.makedirs(self.out_dir, exist_ok=True)
        self.start = time.time()
        self.latest = {}
        self.counts = {}
        self.first = {}
        self.diag = {}
        for key, topic in (('local', '/odometry/local'), ('global', '/odometry/global'),
                           ('gpsodom', '/odometry/gps')):
            self.create_subscription(Odometry, topic, lambda m, k=key: self.on_odom(k, m), 10)
        for key, topic, kind in (
                ('gps_fix', '/gps/fix', NavSatFix), ('gps_validated', '/gps/validated', NavSatFix),
                ('zed_odom_base', '/localization/zed_odom_base', Odometry),
                ('wheel_odom', '/localization/wheel_odom', Odometry),
                ('zed_imu', '/localization/zed_imu', Imu), ('icm_imu', '/localization/icm_imu', Imu),
                ('heading', '/localization/heading', Imu)):
            self.create_subscription(kind, topic, lambda m, k=key: self.tick(k), 10)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diag, 10)
        self.file = open(os.path.join(self.out_dir, 'estimates.csv'), 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['t_abs', 'local_x', 'local_y', 'local_yaw_deg', 'local_cov_xx',
                              'global_x', 'global_y', 'global_yaw_deg', 'global_cov_xx',
                              'gpsodom_x', 'gpsodom_y'])
        self.create_timer(0.2, self.sample)
        if self.duration > 0:
            self.create_timer(self.duration, self.finish)
        self.done_seen = None
        self.create_timer(0.5, self.check_done)

    def check_done(self):
        # fake_sensors.py creates <out_dir>/done at the end of the path; keep
        # recording 2 s more so the filters' final state is captured.
        if self.done_seen is None and os.path.exists(os.path.join(self.out_dir, 'done')):
            self.done_seen = time.time()
        if self.done_seen is not None and time.time() - self.done_seen > 2.0:
            self.finish()

    def tick(self, key):
        now = time.time()
        self.counts[key] = self.counts.get(key, 0) + 1
        self.first.setdefault(key, now)

    def on_odom(self, key, msg):
        self.tick(key)
        p = msg.pose.pose.position
        self.latest[key] = (p.x, p.y, yaw_deg(msg.pose.pose.orientation), msg.pose.covariance[0])

    def on_diag(self, msg):
        for s in msg.status:
            self.diag[s.name] = (LEVEL.get(ord(s.level) if isinstance(s.level, bytes) else s.level, '?'),
                                 s.message)

    def sample(self):
        row = [time.time()]
        for key in ('local', 'global'):
            row += ['%.4f' % v for v in self.latest[key]] if key in self.latest else [''] * 4
        row += ['%.4f' % v for v in self.latest['gpsodom'][:2]] if 'gpsodom' in self.latest else [''] * 2
        self.writer.writerow(row)

    def finish(self):
        self.file.close()
        now = time.time()
        with open(os.path.join(self.out_dir, 'diagnostics.txt'), 'w') as f:
            for name, (level, message) in sorted(self.diag.items()):
                f.write('%-5s %s: %s\n' % (level, name, message))
        with open(os.path.join(self.out_dir, 'topic_rates.txt'), 'w') as f:
            for key in sorted(self.counts):
                span = max(now - self.first[key], 1e-3)
                f.write('%-16s %6d msgs  %6.1f Hz\n' % (key, self.counts[key], self.counts[key] / span))
        self.get_logger().info('Recording saved to %s' % self.out_dir)
        raise SystemExit


def main():
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
