"""Bounded readiness gate for physical-rover Nav2 startup."""
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener


def valid_odometry(message):
    p, q, t = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w,
              t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z)
    covariances = (message.pose.covariance, message.twist.covariance)
    return (all(math.isfinite(value) for value in values)
            and abs(sum(value*value for value in (q.x, q.y, q.z, q.w))-1.0) < 0.05
            and all(all(math.isfinite(v) for v in cov)
                    and all(cov[i] >= 0 for i in (0, 7, 14, 21, 28, 35))
                    for cov in covariances))


class LocalizationReadiness(Node):
    def __init__(self):
        super().__init__('wait_for_localization')
        self.declare_parameter('timeout', 120.0)
        self.declare_parameter('max_message_age', 1.0)
        self.received = {}
        self.previous_clock = None
        self.clock_advanced = False
        self.clock_updated_at = time.monotonic()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.subscriptions_ = [self.create_subscription(
            Odometry, '/odometry/' + name,
            lambda msg, name=name: self.receive(name, msg), qos_profile_sensor_data)
            for name in ('local', 'global', 'gps')]

    def receive(self, name, message):
        if valid_odometry(message):
            self.received[name] = (time.monotonic(), message.header.stamp)
        else:
            self.received.pop(name, None)

    def ready(self):
        now = time.monotonic()
        clock = self.get_clock().now().nanoseconds
        if self.previous_clock is not None and clock > self.previous_clock:
            self.clock_advanced = True
            self.clock_updated_at = now
        self.previous_clock = clock
        if len(self.received) != 3 or not self.clock_advanced:
            return False
        age = float(self.get_parameter('max_message_age').value)
        if not math.isfinite(age) or age <= 0:
            raise ValueError('max_message_age must be positive and finite')
        if now-self.clock_updated_at > age:
            return False
        for received, stamp in self.received.values():
            source_age = (clock - (stamp.sec*1_000_000_000+stamp.nanosec))*1e-9
            if now-received > age or not -0.2 <= source_age <= age:
                return False
        for parent, child in (('map', 'odom'), ('odom', 'base_link')):
            if not self.buffer.can_transform(parent, child, rclpy.time.Time()):
                return False
            stamp = self.buffer.lookup_transform(parent, child, rclpy.time.Time()).header.stamp
            transform_age = (clock-(stamp.sec*1_000_000_000+stamp.nanosec))*1e-9
            if not -0.2 <= transform_age <= age:
                return False
        return True


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationReadiness()
    status = 1
    try:
        timeout = float(node.get_parameter('timeout').value)
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be positive and finite')
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.ready():
                node.get_logger().info('Localization ready: fresh local/global/GPS odometry and map -> odom -> base_link')
                status = 0
                break
        if status:
            node.get_logger().error('Localization readiness timed out')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if status:
        raise SystemExit(status)


if __name__ == '__main__':
    main()
