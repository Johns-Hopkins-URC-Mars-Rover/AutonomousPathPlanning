import math

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from robot_localization.srv import FromLL


class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')

        self.declare_parameter('lat', 39.323178)
        self.declare_parameter('lon', -76.627894)
        self.declare_parameter('heading', 0.0)
        self.declare_parameter('max_goal_distance', 70.0)

        # Convert GPS latitude/longitude to a point in the map frame.
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
        max_goal_distance = self.get_parameter('max_goal_distance').value

        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            raise ValueError(f'Invalid GPS target: latitude={lat}, longitude={lon}')
        if not math.isfinite(heading):
            raise ValueError(f'Invalid target heading: {heading}')
        if not math.isfinite(max_goal_distance) or max_goal_distance <= 0.0:
            raise ValueError(f'Invalid maximum goal distance: {max_goal_distance}')

        if not self.converter.wait_for_service(timeout_sec=30.0):
            raise RuntimeError('Timed out waiting for /fromLL')
        if not self.navigator.wait_for_server(timeout_sec=30.0):
            raise RuntimeError('Timed out waiting for /navigate_to_pose')

        request = FromLL.Request()
        request.ll_point.latitude = lat
        request.ll_point.longitude = lon
        request.ll_point.altitude = 0.0
        self.get_logger().info(f'Converting target: lat={lat}, lon={lon}')

        response = self.wait_for_result(self.converter.call_async(request), timeout=10.0)

        point = response.map_point
        distance_from_origin = math.hypot(point.x, point.y)
        if not all(math.isfinite(value) for value in (point.x, point.y)):
            raise RuntimeError('The GPS conversion returned a non-finite map point')
        if distance_from_origin > max_goal_distance:
            raise RuntimeError(
                f'Converted goal ({point.x:.2f}, {point.y:.2f}) is '
                f'{distance_from_origin:.1f} m from the map origin, outside the '
                f'{max_goal_distance:.1f} m simulation navigation area')

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = point.x
        goal.pose.pose.position.y = point.y
        goal.pose.pose.position.z = 0.0
        rad_heading = math.radians(heading)
        goal.pose.pose.orientation.z = math.sin(rad_heading / 2.0)
        goal.pose.pose.orientation.w = math.cos(rad_heading / 2.0)

        self.get_logger().info(
            f'Sending map goal: x={point.x:.2f}, y={point.y:.2f}, heading={heading:.1f} deg')

        self.goal_handle = self.wait_for_result(
            self.navigator.send_goal_async(goal, feedback_callback=self.feedback_callback)
        )

        if not self.goal_handle.accepted:
            raise RuntimeError('Goal rejected :(')
        self.get_logger().info('Goal accepted :)')

        result = self.wait_for_result(self.goal_handle.get_result_async())
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'Navigation finished with action status {result.status}')
        self.get_logger().info('Navigation succeeded')

    def feedback_callback(self, message):
        remaining = message.feedback.distance_remaining
        self.get_logger().info(f'{remaining:.1f} m left')

    def abort_action(self):
        if getattr(self, 'goal_handle', None) is not None:
            self.goal_handle.cancel_goal_async()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationClient()

    try:
        node.navigate()
    except KeyboardInterrupt:
        node.abort_action()  # Stop the currently running action.
    except Exception as error:
        node.get_logger().error(str(error))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
