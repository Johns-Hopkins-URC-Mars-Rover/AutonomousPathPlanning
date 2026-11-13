"""Correct controller twist signs for the original simulation wheel axes."""
import copy

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class SimWheelOdometry(Node):
    def __init__(self):
        super().__init__('sim_wheel_odometry')
        self.publisher = self.create_publisher(Odometry, '/simulation/wheel_odometry', 10)
        self.subscription = self.create_subscription(
            Odometry, '/diff_drive_controller/odom', self.receive, qos_profile_sensor_data)

    def receive(self, message):
        result = copy.deepcopy(message)
        result.twist.twist.linear.x *= -1
        result.twist.twist.angular.z *= -1
        signs = (-1, 1, 1, 1, 1, -1)
        result.twist.covariance = [value * signs[i // 6] * signs[i % 6]
                                   for i, value in enumerate(message.twist.covariance)]
        self.publisher.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = SimWheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
