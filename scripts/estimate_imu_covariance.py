#!/usr/bin/env python3
"""Collect raw IMU samples and estimate gyro and accelerometer covariance.

Run this while the IMU is completely stationary. The script subscribes to a
sensor_msgs/Imu topic, stores the collected measurements in a CSV file, and
prints unbiased sample covariance matrices suitable for ROS Imu messages.
"""

import argparse
import csv
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


Vector3 = tuple[float, float, float]
Matrix3 = list[list[float]]


def covariance(samples: list[Vector3]) -> Matrix3:
    """Return the unbiased 3x3 sample covariance of three-axis samples."""
    if len(samples) < 2:
        raise ValueError('At least two samples are required for covariance.')

    count = len(samples)
    means = [sum(sample[axis] for sample in samples) / count for axis in range(3)]

    return [
        [
            sum(
                (sample[row] - means[row]) * (sample[column] - means[column])
                for sample in samples
            ) / (count - 1)
            for column in range(3)
        ]
        for row in range(3)
    ]


def flatten(matrix: Matrix3) -> list[float]:
    """Flatten a 3x3 matrix in the row-major order used by sensor_msgs/Imu."""
    return [value for row in matrix for value in row]


def format_matrix(matrix: Matrix3) -> str:
    """Format a 3x3 matrix for terminal output."""
    return '\n'.join(
        '[  ' + '  '.join(f'{value: .10e}' for value in row) + '  ]'
        for row in matrix
    )


class ImuSampleCollector(Node):
    """Collect a fixed number of angular velocity and acceleration samples."""

    def __init__(self, topic: str, sample_count: int, discard_count: int) -> None:
        super().__init__('imu_covariance_estimator')
        self.sample_count = sample_count
        self.discard_remaining = discard_count
        self.angular_velocity: list[Vector3] = []
        self.linear_acceleration: list[Vector3] = []
        self.timestamps: list[tuple[int, int]] = []
        self.subscription = self.create_subscription(
            Imu,
            topic,
            self._imu_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'Waiting for {discard_count + sample_count} messages on {topic}; '
            f'the first {discard_count} will be discarded.'
        )

    @property
    def complete(self) -> bool:
        """Return whether the requested number of retained samples arrived."""
        return len(self.angular_velocity) >= self.sample_count

    def _imu_callback(self, message: Imu) -> None:
        if self.discard_remaining > 0:
            self.discard_remaining -= 1
            if self.discard_remaining == 0:
                self.get_logger().info('Warm-up messages discarded; collecting samples.')
            return

        if self.complete:
            return

        self.timestamps.append((message.header.stamp.sec, message.header.stamp.nanosec))
        self.angular_velocity.append((
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        ))
        self.linear_acceleration.append((
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        ))

        received = len(self.angular_velocity)
        if received % 100 == 0 or self.complete:
            self.get_logger().info(f'Collected {received}/{self.sample_count} samples.')


def write_csv(path: Path, collector: ImuSampleCollector) -> None:
    """Write all retained measurements to a CSV file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as output_file:
        writer = csv.writer(output_file)
        writer.writerow([
            'stamp_sec',
            'stamp_nanosec',
            'angular_velocity_x_rad_s',
            'angular_velocity_y_rad_s',
            'angular_velocity_z_rad_s',
            'linear_acceleration_x_m_s2',
            'linear_acceleration_y_m_s2',
            'linear_acceleration_z_m_s2',
        ])
        for timestamp, angular, linear in zip(
                collector.timestamps,
                collector.angular_velocity,
                collector.linear_acceleration):
            writer.writerow((*timestamp, *angular, *linear))


def parse_arguments() -> argparse.Namespace:
    """Parse arguments not consumed by ROS 2."""
    parser = argparse.ArgumentParser(
        description='Estimate covariance from stationary sensor_msgs/Imu samples.',
    )
    parser.add_argument('--topic', default='/imu/data_raw', help='IMU topic name')
    parser.add_argument('--samples', type=int, default=1000, help='samples to retain')
    parser.add_argument(
        '--discard',
        type=int,
        default=100,
        help='initial warm-up messages to discard',
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=60.0,
        help='maximum collection time in seconds; 0 disables the timeout',
    )
    parser.add_argument(
        '--output',
        type=Path,
        default=Path('imu_covariance_samples.csv'),
        help='destination CSV path',
    )
    arguments = parser.parse_args()
    if arguments.samples < 2:
        parser.error('--samples must be at least 2')
    if arguments.discard < 0:
        parser.error('--discard cannot be negative')
    if arguments.timeout < 0:
        parser.error('--timeout cannot be negative')
    return arguments


def main() -> int:
    """Collect measurements, save them, and print covariance estimates."""
    arguments = parse_arguments()
    # The script arguments were handled above, so do not pass them to ROS.
    rclpy.init(args=[])
    collector = ImuSampleCollector(
        topic=arguments.topic,
        sample_count=arguments.samples,
        discard_count=arguments.discard,
    )
    started_at = time.monotonic()

    try:
        while rclpy.ok() and not collector.complete:
            rclpy.spin_once(collector, timeout_sec=0.25)
            if arguments.timeout and time.monotonic() - started_at >= arguments.timeout:
                collector.get_logger().error(
                    f'Timed out after {arguments.timeout:g} seconds with '
                    f'{len(collector.angular_velocity)}/{arguments.samples} samples.'
                )
                return 1

        if not collector.complete:
            return 1

        write_csv(arguments.output, collector)
        gyro_covariance = covariance(collector.angular_velocity)
        acceleration_covariance = covariance(collector.linear_acceleration)

        print(f'\nSaved {arguments.samples} samples to: {arguments.output.resolve()}')
        print('\nAngular velocity covariance [(rad/s)^2]:')
        print(format_matrix(gyro_covariance))
        print('\nROS angular_velocity_covariance (row-major):')
        print(flatten(gyro_covariance))
        print('\nLinear acceleration covariance [(m/s^2)^2]:')
        print(format_matrix(acceleration_covariance))
        print('\nROS linear_acceleration_covariance (row-major):')
        print(flatten(acceleration_covariance))
        return 0
    except KeyboardInterrupt:
        collector.get_logger().warning('Collection interrupted; no results written.')
        return 130
    finally:
        collector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
