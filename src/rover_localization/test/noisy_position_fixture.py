#!/usr/bin/env python3
"""Asynchronous noisy ROS fixture with independent truth and explicit impairments."""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster

from noisy_sensor_model import NoiseModel
from sensor_contracts import normalize_quaternion, validate_stamp


def seconds(msg):
    return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9


class Sensors(Node):
    def __init__(self, seed, profile):
        super().__init__('noisy_position_sensors')
        self.model = NoiseModel(seed, profile)
        self.seed, self.profile = seed, profile
        self.start = time.monotonic()
        self.origin = self.get_clock().now().nanoseconds*1e-9
        self.camera = self.create_publisher(Odometry, '/zed/zed_node/odom', 10)
        self.imu = self.create_publisher(Imu, '/imu/fused', 10)
        self.truth = self.create_publisher(Odometry, '/test/noisy_truth', 10)
        self.meta = self.create_publisher(String, '/test/noisy_scenario', 10)
        self.pending = []
        self.sent_stale = self.sent_future = False
        self.counts = dict(camera_dropout=0, imu_dropout=0, both_dropout=0,
                           stale_stamp=0, future_stamp=0)
        self.broadcaster = StaticTransformBroadcaster(self)
        transforms = []
        for child, xyz, q in (
                ('synthetic_camera_link', self.model.camera_mount_position, self.model.camera_mount_orientation),
                ('synthetic_imu_link', (0.,0.,0.), self.model.imu_mount_orientation)):
            msg = TransformStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id, msg.child_frame_id = 'base_link', child
            msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = xyz
            msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = q
            transforms.append(msg)
        self.broadcaster.sendTransform(transforms)
        self.fixture_timers = [self.create_timer(1./30., self.publish_camera),
                       self.create_timer(.01, self.publish_imu),
                       self.create_timer(.02, self.publish_truth),
                       self.create_timer(.005, self.flush),
                       self.create_timer(.1, self.metadata)]

    def elapsed(self):
        return time.monotonic()-self.start

    def publish_truth(self):
        p, q, w = self.model.truth(self.elapsed())
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = p
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q
        msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = w
        self.truth.publish(msg)

    def publish_camera(self):
        elapsed = self.elapsed()
        if 8.8 <= elapsed < 10.5:
            self.counts['both_dropout'] += 1
            return
        if 6. <= elapsed < 7.:
            self.counts['camera_dropout'] += 1
            return
        sample = self.model.camera(elapsed)
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'odom', 'synthetic_camera_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = sample['position']
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = sample['orientation']
        msg.pose.covariance = sample['covariance']
        if elapsed >= 2.6 and not self.sent_stale:
            msg.header.stamp.sec -= 1
            self.sent_stale = True
            self.counts['stale_stamp'] += 1
        elif elapsed >= 3.3 and not self.sent_future:
            msg.header.stamp.sec += 1
            self.sent_future = True
            self.counts['future_stamp'] += 1
        self.pending.append((time.monotonic()+self.model.latency('camera'), self.camera, msg))

    def publish_imu(self):
        elapsed = self.elapsed()
        if 8.8 <= elapsed < 10.5:
            self.counts['both_dropout'] += 1
            return
        if 7.2 <= elapsed < 8.4:
            self.counts['imu_dropout'] += 1
            return
        sample = self.model.imu(elapsed)
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'synthetic_imu_link'
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = sample['omega']
        msg.angular_velocity_covariance = sample['covariance']
        msg.orientation_covariance[0] = -1.
        msg.linear_acceleration_covariance[0] = -1.
        self.pending.append((time.monotonic()+self.model.latency('imu'), self.imu, msg))

    def flush(self):
        now = time.monotonic()
        ready = sorted((item for item in self.pending if item[0] <= now), key=lambda v: v[0])
        self.pending = [item for item in self.pending if item[0] > now]
        for deadline, publisher, msg in ready:
            publisher.publish(msg)  # Original acquisition timestamp is preserved.

    def metadata(self):
        msg = String()
        msg.data = json.dumps(dict(seed=self.seed, profile=self.profile, origin=self.origin, **self.counts))
        self.meta.publish(msg)


class Probe(Node):
    def __init__(self):
        super().__init__('noisy_position_probe')
        self.truth, self.estimate, self.meta, self.diagnostics = [], [], None, {}
        self.stale_seen = set()
        self.both_stale_seen = False
        self.recovered_inputs = False
        self.handles = [
            self.create_subscription(Odometry, '/test/noisy_truth', lambda m: self.record(self.truth, m), 10),
            self.create_subscription(Odometry, '/odometry/local', lambda m: self.record(self.estimate, m), 10),
            self.create_subscription(String, '/test/noisy_scenario', self.metadata, 10),
            self.create_subscription(DiagnosticArray, '/diagnostics', self.health, 10),
        ]

    @staticmethod
    def record(buffer, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        stamp = seconds(msg)
        validate_stamp(stamp, stamp, buffer[-1][0] if buffer else None)
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            raise ValueError('nonfinite position output')
        orientation = normalize_quaternion((q.x,q.y,q.z,q.w))
        buffer.append((stamp, (p.x,p.y,p.z), orientation))
        del buffer[:-1500]

    def metadata(self, msg):
        self.meta = json.loads(msg.data)

    def health(self, msg):
        stale_now = set()
        for status in msg.status:
            if status.name in ('local_sensor_inputs/camera', 'local_sensor_inputs/imu'):
                self.diagnostics[status.name] = {v.key:v.value for v in status.values}
                if 'stale' in status.message:
                    self.stale_seen.add(status.name)
                    stale_now.add(status.name)
        if len(stale_now) == 2:
            self.both_stale_seen = True
        elif self.both_stale_seen and len(self.diagnostics) == 2:
            # Ages prove inputs resumed even if transport rejects a reordered packet.
            self.recovered_inputs = all(
                0 <= float(values['age_s']) <= .5 for values in self.diagnostics.values())

    def result(self):
        if not self.meta or not self.estimate or not self.truth:
            return None
        origin = self.meta['origin']
        if min(self.estimate[-1][0], self.truth[-1][0])-origin < 13.5:
            return None
        groups = dict(all=[], before_fault=[], fault=[], recovery=[])
        angles = []
        for stamp, p, q in self.estimate:
            truth = min(self.truth, key=lambda t: abs(t[0]-stamp))
            elapsed = stamp-origin
            if abs(truth[0]-stamp) > .1 or elapsed < 1.5:
                continue
            error = math.dist(p, truth[1])
            if not math.isfinite(error):
                raise RuntimeError('nonfinite position output')
            groups['all'].append(error)
            if elapsed < 5.: groups['before_fault'].append(error)
            if 5.4 <= elapsed < 6.8: groups['fault'].append(error)
            if elapsed >= 11.5: groups['recovery'].append(error)
            dot = min(1., abs(sum(a*b for a,b in zip(q, truth[2]))))
            angles.append(math.degrees(2.*math.acos(dot)))
        if any(len(samples) < 15 for samples in groups.values()):
            return None
        summary = {name: dict(samples=len(samples), rmse_m=math.sqrt(sum(v*v for v in samples)/len(samples)),
                              max_error_m=max(samples), median_error_m=sorted(samples)[len(samples)//2])
                   for name,samples in groups.items()}
        counts = {name: {k:int(v) for k,v in values.items() if k in ('accepted','rejected')}
                  for name, values in self.diagnostics.items()}
        if (self.meta['stale_stamp'] != 1 or self.meta['future_stamp'] != 1 or
                not all(self.meta[k] for k in ('camera_dropout', 'imu_dropout', 'both_dropout'))):
            raise RuntimeError('required timestamp/outage impairments were not generated')
        if 'local_sensor_inputs/imu' not in self.stale_seen:
            raise RuntimeError('IMU outage was not reported as stale')
        if not self.both_stale_seen or not self.recovered_inputs:
            raise RuntimeError('simultaneous input loss and subsequent fresh inputs were not observed')
        if sum(item.get('rejected',0) for item in counts.values()) < 2:
            raise RuntimeError('bad timestamps were not rejected')
        passed = summary['before_fault']['rmse_m'] < .5 and summary['recovery']['rmse_m'] < .5
        if self.meta['profile'] == 'noisy':
            passed = passed and summary['all']['rmse_m'] < .5 and summary['all']['max_error_m'] < 1.5
        else:
            passed = passed and summary['fault']['median_error_m'] > 10.
        result = dict(seed=self.meta['seed'], profile=self.meta['profile'],
                      passed_fixture_expectations=passed, position=summary,
                      orientation_rmse_deg=math.sqrt(sum(v*v for v in angles)/len(angles)),
                      input_counts=counts, stale_inputs_observed=sorted(self.stale_seen),
                      simultaneous_loss_reported=self.both_stale_seen,
                      inputs_resumed_after_loss=self.recovered_inputs,
                      generated_impairments=self.meta,
                      interpretation='Synthetic robustness/failure characterization; not hardware calibration')
        print('NOISY_FUSION_RESULT '+json.dumps(result), flush=True)
        if not passed:
            raise RuntimeError('noisy fixture exceeded its stated regression bounds; inspect reported errors')
        return result


def main():
    rclpy.init(args=[])
    mode = sys.argv[1]
    node = Sensors(int(sys.argv[3]), sys.argv[2]) if mode == 'sensors' else Probe()
    try:
        if mode == 'sensors':
            rclpy.spin(node)
        else:
            deadline = time.monotonic()+30.
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                if node.result(): return 0
            raise RuntimeError('timed out collecting noisy fixture measurements')
    except KeyboardInterrupt:
        return 0 if mode == 'sensors' else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
