#!/usr/bin/env python3
"""Align robot_localization's relative yaw using a measured startup heading.

This provides navsat's initial coordinate alignment, not a compass observation
for either EKF. The rover must stay parked until navsat publishes /odometry/gps.
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

from sensor_contracts import compass_to_enu, normalize_quaternion


class StartupHeading(Node):
    def __init__(self):
        super().__init__('startup_heading')
        self.initial_yaw = compass_to_enu(
            self.declare_parameter('initial_heading_deg', float('nan')).value)
        stddev = self.declare_parameter('heading_stddev_deg', 10.0).value
        if not math.isfinite(stddev) or stddev <= 0.0:
            raise ValueError('heading_stddev_deg must be finite and positive')
        self.variance = math.radians(stddev) ** 2
        self.reference = None
        self.publisher = self.create_publisher(Imu, '/localization/startup_heading', 10)
        self.subscription = self.create_subscription(
            Odometry, '/odometry/local', self.receive, 10)
        self.get_logger().warning(
            'Using a manually measured startup heading. Keep the rover parked until '
            '/odometry/gps arrives. This is coordinate alignment, not a compass.')

    def receive(self, msg):
        q = msg.pose.pose.orientation
        try:
            x, y, z, w = normalize_quaternion((q.x, q.y, q.z, q.w))
        except ValueError:
            return
        yaw = math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))
        if self.reference is None:
            self.reference = yaw
        aligned = self.initial_yaw + yaw - self.reference
        out = Imu()
        out.header = msg.header
        out.header.frame_id = msg.child_frame_id
        out.orientation.z = math.sin(aligned / 2.0)
        out.orientation.w = math.cos(aligned / 2.0)
        out.orientation_covariance = [self.variance if i in (0, 4, 8) else 0.0
                                      for i in range(9)]
        out.angular_velocity_covariance[0] = -1.0
        out.linear_acceleration_covariance[0] = -1.0
        self.publisher.publish(out)


def main():
    rclpy.init()
    node = None
    try:
        node = StartupHeading()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
