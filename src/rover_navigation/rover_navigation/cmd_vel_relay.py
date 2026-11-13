import copy
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.publisher = self.create_publisher(
            Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.relay, 10)

    def relay(self, message):
        output = copy.deepcopy(message)
        output.linear.x = -output.linear.x
        output.angular.z = -output.angular.z
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
