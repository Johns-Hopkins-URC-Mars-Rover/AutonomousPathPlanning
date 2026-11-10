#!/usr/bin/env python3
"""Publish a stationary joint-IMU fixture for the real-ZED bench test."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class FakeImuPublisher(Node):
    """Publish zero angular velocity with fresh timestamps on a configurable topic."""

    def __init__(self):
        super().__init__('fake_imu_publisher')
        topic = self.declare_parameter('topic', '/imu/fused').value
        frame_id = self.declare_parameter('frame_id', 'base_link').value
        rate_hz = self.declare_parameter('rate_hz', 50.0).value

        if rate_hz <= 0.0:
            raise ValueError('rate_hz must be positive')

        self._publisher = self.create_publisher(Imu, topic, 10)
        self._message = Imu()
        self._message.header.frame_id = frame_id
        self._message.orientation.w = 1.0
        self._message.orientation_covariance[0] = -1.0
        self._message.angular_velocity_covariance[0] = 0.01
        self._message.angular_velocity_covariance[4] = 0.01
        self._message.angular_velocity_covariance[8] = 0.01
        self._message.linear_acceleration_covariance[0] = -1.0
        self._timer = self.create_timer(1.0 / rate_hz, self._publish)

    def _publish(self):
        self._message.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(self._message)


def main():
    rclpy.init()
    node = FakeImuPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
