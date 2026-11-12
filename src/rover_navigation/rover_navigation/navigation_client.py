import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from robot_localization.srv import FromLL


class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')

        self.declare_parameter('lat', 1.0)
        self.declare_parameter('lon', 1.0)
        self.declare_parameter('heading', 0.0)

        # convert from gps lat/lon to coord from map
        self.converter = self.create_client(FromLL, '/fromLL')
        self.navigator = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def wait_for_result(self, future, timeout=None):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        if not future.done():
            raise TimeoutError('Timed out waiting for ROS response')

        result = future.result()
        if result is None:
            raise RuntimeError('ROS request returned no response')
        return result

    def navigate(self):
        lat = self.get_parameter('lat').value
        lon = self.get_parameter('lon').value
        heading = self.get_parameter('heading').value

        if not self.converter.wait_for_service(timeout_sec=30.0):
            raise RuntimeError('Timed out waiting for /fromLL')
        if not self.navigator.wait_for_server(timeout_sec=30.0):
            raise RuntimeError('Timed out waiting for /navigate_to_pose')

        request = FromLL.Request()
        request.ll_point.latitude = lat
        request.ll_point.longitude = lon
        request.ll_point.altitude = 0.0
        self.get_logger().info(f'Sending goal: lat={lat}, lon={lon}')
        

        response = self.wait_for_result(self.converter.call_async(request), timeout=10.0)

        point = response.map_point # the converted point

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = rclpy.time.Time().to_msg()
        goal.pose.pose.position.x = point.x
        goal.pose.pose.position.y = point.y
        goal.pose.pose.position.z = 0.0  # we don't need a target altitude
        rad_heading = math.radians(heading)
        # convert nice happy angle into quaternion
        goal.pose.pose.orientation.z = math.sin(rad_heading / 2.0)
        goal.pose.pose.orientation.w = math.cos(rad_heading / 2.0)

        self.get_logger().info(f'Sending goal: x={point.x:.2f}, y={point.y:.2f}')

        self.goal_handle = self.wait_for_result(
            self.navigator.send_goal_async(goal, feedback_callback=self.feedback_callback)
        )

        if not self.goal_handle.accepted:
            raise RuntimeError('Goal rejected :(')
        self.get_logger().info('Goal accepted :)')

        result = self.wait_for_result(self.goal_handle.get_result_async())
        self.get_logger().info(f'{result.status}')


    def feedback_callback(self, message):
        remaining = message.feedback.distance_remaining
        self.get_logger().info(f'{remaining:.1f} m left')

    def abort_action(self):
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationClient()

    try:
        node.navigate()
    except KeyboardInterrupt:
        node.abort_action() # stop currently running action
    except Exception as error:
        node.get_logger().error(str(error))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()