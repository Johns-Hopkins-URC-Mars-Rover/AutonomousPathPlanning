#!/usr/bin/env python3
"""Print and log what each IMU publishes next to the fused output.

Subscribes to the ICM-20948 (/imu/data_raw), the ZED IMU
(/zed/zed_node/imu/data) and the fusion output (/imu/data). Once per second
it prints the latest accel, gyro and covariance diagonals of each, plus the
fused roll/pitch/yaw. Every message is also written to a CSV (one row per
message, a `source` column says which topic) for later analysis.

Raw values are in each sensor's own frame (imu_link, zed_imu_link). The fused
output is in base_link, so with a non-identity mount the axes will differ.

Works live or while replaying a bag (add --sim-time with `ros2 bag play --clock`).

Usage:
    python3 scripts/imu_compare.py
    python3 scripts/imu_compare.py --output results/imu_compare.csv --period 2
"""

import argparse
import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

SOURCES = {
    'icm': '/imu/data_raw',
    'zed': '/zed/zed_node/imu/data',
    'fused': '/imu/data',
}

COLUMNS = [
    'source', 'stamp', 'frame_id',
    'ax', 'ay', 'az', 'gx', 'gy', 'gz',
    'roll_deg', 'pitch_deg', 'yaw_deg',
    'qx', 'qy', 'qz', 'qw',
    'acc_cov_xx', 'acc_cov_yy', 'acc_cov_zz',
    'gyro_cov_xx', 'gyro_cov_yy', 'gyro_cov_zz',
    'ori_cov_xx', 'ori_cov_yy', 'ori_cov_zz',
]


def euler_deg(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return tuple(math.degrees(a) for a in (roll, pitch, yaw))


def row_from(source, msg):
    q = msg.orientation
    has_orientation = msg.orientation_covariance[0] >= 0.0 and (
        q.x or q.y or q.z or q.w)
    if has_orientation:
        roll, pitch, yaw = euler_deg(q.x, q.y, q.z, q.w)
        quat = [q.x, q.y, q.z, q.w]
        ori_cov = [msg.orientation_covariance[i] for i in (0, 4, 8)]
    else:
        roll = pitch = yaw = float('nan')
        quat = [float('nan')] * 4
        ori_cov = [float('nan')] * 3
    a, g = msg.linear_acceleration, msg.angular_velocity
    return [
        source,
        f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}',
        msg.header.frame_id,
        a.x, a.y, a.z, g.x, g.y, g.z,
        roll, pitch, yaw,
        *quat,
        *[msg.linear_acceleration_covariance[i] for i in (0, 4, 8)],
        *[msg.angular_velocity_covariance[i] for i in (0, 4, 8)],
        *ori_cov,
    ]


class ImuCompare(Node):
    def __init__(self, output, period):
        super().__init__('imu_compare')
        self.latest = {}
        self.counts = dict.fromkeys(SOURCES, 0)
        Path(output).parent.mkdir(parents=True, exist_ok=True)
        self.file = open(output, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(COLUMNS)
        for name, topic in SOURCES.items():
            qos = 10 if name == 'fused' else qos_profile_sensor_data
            self.create_subscription(
                Imu, topic, lambda m, n=name: self.on_msg(n, m), qos)
        self.create_timer(period, self.report)
        self.get_logger().info(f'Logging to {output}')

    def on_msg(self, name, msg):
        row = row_from(name, msg)
        self.writer.writerow(row)
        self.latest[name] = row
        self.counts[name] += 1

    def report(self):
        self.file.flush()
        print('-' * 100)
        for name in SOURCES:
            row = self.latest.get(name)
            if row is None:
                print(f'{name:>5}: no data on {SOURCES[name]}')
                continue
            (_, _, frame, ax, ay, az, gx, gy, gz, roll, pitch, yaw,
             _, _, _, _, ca_x, ca_y, ca_z, cg_x, cg_y, cg_z, *_) = row
            norm = math.sqrt(ax * ax + ay * ay + az * az)
            print(f'{name:>5} [{frame}] n={self.counts[name]}')
            print(f'       accel  {ax:8.3f} {ay:8.3f} {az:8.3f}  |a|={norm:7.3f} m/s^2'
                  f'   cov diag {ca_x:.2e} {ca_y:.2e} {ca_z:.2e}')
            print(f'       gyro   {gx:8.4f} {gy:8.4f} {gz:8.4f} rad/s'
                  f'              cov diag {cg_x:.2e} {cg_y:.2e} {cg_z:.2e}')
            if not math.isnan(yaw):
                print(f'       r/p/y  {roll:8.2f} {pitch:8.2f} {yaw:8.2f} deg'
                      f'   sigma r/p/y {math.degrees(math.sqrt(row[22])):.2f} '
                      f'{math.degrees(math.sqrt(row[23])):.2f} '
                      f'{math.degrees(math.sqrt(row[24])):.2f} deg')

    def close(self):
        self.file.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--output', default='results/imu_compare.csv')
    parser.add_argument('--period', type=float, default=1.0, help='print period [s]')
    parser.add_argument('--sim-time', action='store_true',
                        help='use /clock (bag replay with --clock)')
    args = parser.parse_args()

    rclpy.init()
    node = ImuCompare(args.output, args.period)
    if args.sim_time:
        node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
