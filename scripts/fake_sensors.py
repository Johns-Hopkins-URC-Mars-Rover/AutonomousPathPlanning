#!/usr/bin/env python3
"""Fake every rover sensor along a KNOWN PATH (no hardware, no Gazebo).

Publishes on the SAME topics and frames as the real drivers so the real
hardware_localization stack (validators + local/global EKF + navsat) can be
exercised: /gps/fix, /imu/data_raw (ICM), /zed/zed_node/imu/data (+_raw),
/zed/zed_node/odom and /diff_drive_controller/odom. Noise levels come from
what we measured on the real sensors on 2026-11-12.

The rover starts parked facing EAST at the origin (x east, y north, yaw 0 = east,
counter-clockwise positive). The path is a list of segments:
    wait:SECONDS            stand still
    drive:METRES@MPS        straight (negative metres = reverse)
    turn:DEGREES@DEG_PER_S  turn in place (positive = left/CCW)
Use ``path_name`` for a built-in path or ``path_spec`` for your own, e.g.
    path_spec:="wait:10;drive:5@0.5;turn:90@30;drive:5@0.5;wait:10"
Writes truth.csv and gps.csv to ``out_dir`` and a ``done`` file when finished.
"""
import csv
import math
import os
import random

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from tf2_ros import StaticTransformBroadcaster

R_EARTH = 6378137.0
D2R = math.pi / 180.0
CAMERA_FORWARD = 0.3429  # matches approximate_zed_camera_link_mount in the launch

PATHS = {
    'straight': 'wait:15;drive:20@0.5;wait:15',
    'turn_in_place': 'wait:10;turn:360@30;wait:5;turn:-360@30;wait:10',
    'square': 'wait:10;' + 'drive:5@0.5;turn:90@30;' * 4 + 'wait:10',
    'out_and_back': 'wait:10;drive:10@0.5;turn:180@30;drive:10@0.5;turn:180@30;wait:10',
}


def parse_path(spec):
    """Return [(duration_s, v_mps, w_rad_s)] for a path spec string."""
    segments = []
    for part in spec.split(';'):
        part = part.strip()
        if not part:
            continue
        kind, _, arg = part.partition(':')
        if kind == 'wait':
            segments.append((float(arg), 0.0, 0.0))
        elif kind == 'drive':
            dist, _, speed = arg.partition('@')
            dist, speed = float(dist), abs(float(speed))
            segments.append((abs(dist) / speed, math.copysign(speed, dist), 0.0))
        elif kind == 'turn':
            angle, _, rate = arg.partition('@')
            angle, rate = float(angle), abs(float(rate))
            segments.append((abs(angle) / rate, 0.0, math.copysign(rate * D2R, angle)))
        else:
            raise ValueError('unknown path segment: %r' % part)
    return segments


def cov3(a, b, c):
    return [a, 0., 0., 0., b, 0., 0., 0., c]


class FakeSensors(Node):
    def __init__(self):
        super().__init__('fake_sensors')
        p = self.declare_parameter
        self.out_dir = p('out_dir', '.').value
        name = p('path_name', 'straight').value
        spec = p('path_spec', '').value or PATHS[name]
        self.segments = parse_path(spec)
        self.t_end = sum(s[0] for s in self.segments)
        self.gps_sigma = p('gps_sigma_m', 3.0).value
        self.gps_tau = p('gps_tau_s', 30.0).value
        self.wheel_turn_scale = p('wheel_turn_scale', 0.9).value  # skid-steer slip in turns
        self.lat0 = p('lat0', 39.3249).value
        self.lon0 = p('lon0', -76.6229).value
        self.rng = random.Random(p('seed', 1).value)
        self.spec = spec

        self.pub = {
            'gps': self.create_publisher(NavSatFix, '/gps/fix', 10),
            'icm': self.create_publisher(Imu, '/imu/data_raw', 10),
            'zimu': self.create_publisher(Imu, '/zed/zed_node/imu/data', 10),
            'zraw': self.create_publisher(Imu, '/zed/zed_node/imu/data_raw', 10),
            'zodom': self.create_publisher(Odometry, '/zed/zed_node/odom', 10),
            'wheel': self.create_publisher(Odometry, '/diff_drive_controller/odom', 10),
        }
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = 'base_link', 'zed_imu_link'
        tf.transform.translation.x = CAMERA_FORWARD
        tf.transform.rotation.w = 1.0
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(tf)

        self.x = self.y = self.yaw = 0.0
        self.t0 = None
        self.last = None
        self.gps_err = [0.0, 0.0]
        self.zed_off = [0.0, 0.0]
        self.zed_yaw_drift = 0.0
        self.next = {k: 0.0 for k in ('gps', 'icm', 'zimu', 'zodom', 'wheel')}
        self.period = {'gps': 0.4, 'icm': 0.02, 'zimu': 0.01, 'zodom': 1 / 30., 'wheel': 1 / 30.}

        os.makedirs(self.out_dir, exist_ok=True)
        self.truth = open(os.path.join(self.out_dir, 'truth.csv'), 'w', newline='')
        self.gpslog = open(os.path.join(self.out_dir, 'gps.csv'), 'w', newline='')
        self.truth_w = csv.writer(self.truth)
        self.gps_w = csv.writer(self.gpslog)
        self.truth_w.writerow(['t', 'east_m', 'north_m', 'yaw_deg', 'speed_mps', 'yawrate_dps', 't_abs'])
        self.gps_w.writerow(['t', 'true_east_m', 'true_north_m', 'meas_east_m', 'meas_north_m'])
        with open(os.path.join(self.out_dir, 'path.txt'), 'w') as f:
            f.write('name: %s\nspec: %s\ntotal_s: %.1f\n' % (name, spec, self.t_end))
        self.create_timer(0.005, self.step)
        self.get_logger().info('FAKE SENSORS: path "%s" = %s (total %.0f s). GPS noise sigma %.1f m.'
                               % (name, spec, self.t_end, self.gps_sigma))

    def g(self, sigma):
        return self.rng.gauss(0.0, sigma)

    def command(self, t):
        for duration, v, w in self.segments:
            if t < duration:
                return v, w
            t -= duration
        return 0.0, 0.0

    def stamp(self, msg, frame):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame

    def step(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = self.last = now
        t = now - self.t0
        if t > self.t_end:
            self.get_logger().info('FAKE SENSORS: path finished.')
            self.truth.close()
            self.gpslog.close()
            open(os.path.join(self.out_dir, 'done'), 'w').close()
            raise SystemExit
        dt = now - self.last
        self.last = now
        self.v, self.w = self.command(t)
        self.yaw += self.w * dt
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        for kind in list(self.next):
            if t >= self.next[kind]:
                self.next[kind] += self.period[kind]
                getattr(self, 'pub_' + kind)(t)
        if int(t * 20) != int((t - dt) * 20):
            self.truth_w.writerow(['%.3f' % t, '%.4f' % self.x, '%.4f' % self.y,
                                   '%.3f' % (self.yaw / D2R), '%.3f' % self.v,
                                   '%.2f' % (self.w / D2R), '%.3f' % now])

    # ---- sensors -------------------------------------------------------
    def pub_gps(self, t):
        a = math.exp(-self.period['gps'] / self.gps_tau)
        for i in (0, 1):  # slow correlated wander, like multipath/atmosphere
            self.gps_err[i] = a * self.gps_err[i] + math.sqrt(1 - a * a) * self.g(self.gps_sigma)
        e, n = self.x + self.gps_err[0], self.y + self.gps_err[1]
        msg = NavSatFix()
        self.stamp(msg, 'gps_link')
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.latitude = self.lat0 + math.degrees(n / R_EARTH)
        msg.longitude = self.lon0 + math.degrees(e / (R_EARTH * math.cos(self.lat0 * D2R)))
        msg.altitude = 50.0 + self.g(5.0)
        var = self.gps_sigma ** 2
        msg.position_covariance = [var, 0., 0., 0., var, 0., 0., 0., 4 * var]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub['gps'].publish(msg)
        self.gps_w.writerow(['%.3f' % t, '%.3f' % self.x, '%.3f' % self.y, '%.3f' % e, '%.3f' % n])

    def imu_msg(self, frame, gyro_bias, gyro_sd, acc_z, acc_sd, cov):
        msg = Imu()
        self.stamp(msg, frame)
        msg.orientation.w = 1.0
        msg.orientation_covariance = [-1.0] + [0.0] * 8
        msg.angular_velocity.x = self.g(gyro_sd)
        msg.angular_velocity.y = self.g(gyro_sd)
        msg.angular_velocity.z = self.w + gyro_bias + self.g(gyro_sd)
        msg.linear_acceleration.x = self.g(acc_sd)
        msg.linear_acceleration.y = self.g(acc_sd)
        msg.linear_acceleration.z = acc_z + self.g(acc_sd)
        msg.angular_velocity_covariance = cov
        msg.linear_acceleration_covariance = cov3(acc_sd ** 2, acc_sd ** 2, acc_sd ** 2)
        return msg

    def pub_icm(self, t):
        # ICM after our bias calibration: ~0.05 deg/s residual, 1.3 deg/s noise.
        self.pub['icm'].publish(self.imu_msg(
            'imu_link', 0.05 * D2R, 1.3 * D2R, 10.0, 0.07, cov3(4.6e-4, 5.2e-4, 7.6e-4)))

    def pub_zimu(self, t):
        msg = self.imu_msg('zed_imu_link', 0.22 * D2R, 0.09 * D2R, 9.79, 0.05,
                           cov3(2.5e-6, 1.9e-6, 2.6e-6))
        self.pub['zimu'].publish(msg)
        self.pub['zraw'].publish(msg)

    def pub_zodom(self, t):
        # Visual odometry: 1% scale error plus a slow random walk in position and yaw.
        step = math.sqrt(self.period['zodom'])
        for i in (0, 1):
            self.zed_off[i] += self.g(0.002) * step
        self.zed_yaw_drift += self.g(0.05 * D2R) * step
        yaw = self.yaw + self.zed_yaw_drift
        msg = Odometry()
        self.stamp(msg, 'odom')
        msg.child_frame_id = 'zed_camera_link'
        msg.pose.pose.position.x = self.x * 1.01 + CAMERA_FORWARD * math.cos(yaw) + self.zed_off[0]
        msg.pose.pose.position.y = self.y * 1.01 + CAMERA_FORWARD * math.sin(yaw) + self.zed_off[1]
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        msg.pose.covariance = [0.0] * 36
        for i, c in enumerate((0.01, 0.01, 0.01, 0.01, 0.01, 0.02)):
            msg.pose.covariance[i * 7] = c
        self.pub['zodom'].publish(msg)

    def pub_wheel(self, t):
        msg = Odometry()
        self.stamp(msg, 'odom')
        msg.child_frame_id = 'base_link'
        msg.twist.twist.linear.x = (0.98 * self.v + self.g(0.02)) if self.v else self.g(0.005)
        msg.twist.twist.angular.z = self.wheel_turn_scale * self.w + self.g(0.02)
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance = [0.0] * 36
        msg.twist.covariance = [0.0] * 36
        for i, c in ((0, 0.01), (7, 0.01), (14, 0.01), (21, 0.01), (28, 0.01), (35, 0.02)):
            msg.pose.covariance[i] = c
            msg.twist.covariance[i] = c
        self.pub['wheel'].publish(msg)


def main():
    rclpy.init()
    node = FakeSensors()
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
