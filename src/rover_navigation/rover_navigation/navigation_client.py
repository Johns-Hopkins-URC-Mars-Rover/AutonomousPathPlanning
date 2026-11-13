import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from robot_localization.srv import FromLL
from lifecycle_msgs.srv import GetState


def wait_for_navigation_active(node, timeout=60.0):
    """An action service can exist while Nav2 still rejects all goals."""
    deadline = time.monotonic() + timeout
    for name in ('bt_navigator', 'velocity_smoother'):
        service = node.create_client(GetState, '/' + name + '/get_state')
        try:
            remaining = deadline-time.monotonic()
            if remaining <= 0 or not service.wait_for_service(timeout_sec=remaining):
                raise TimeoutError('Timed out waiting for Nav2 lifecycle services')
            while time.monotonic() < deadline:
                future = service.call_async(GetState.Request())
                rclpy.spin_until_future_complete(
                    node, future, timeout_sec=max(0., min(5., deadline-time.monotonic())))
                if not future.done():
                    future.cancel()
                    raise TimeoutError('Timed out reading Nav2 lifecycle state')
                response = future.result()
                if response is not None and response.current_state.id == 3:
                    break
                rclpy.spin_once(node, timeout_sec=min(.1, max(0., deadline-time.monotonic())))
            else:
                raise TimeoutError('Timed out waiting for Nav2 activation')
        finally:
            node.destroy_client(service)


class NavigationClient(Node):
    def wait_for_navigation_active(self):
        wait_for_navigation_active(self)

    def __init__(self):
        super().__init__('navigation_client')

        self.declare_parameter('lat', 39.323178)
        self.declare_parameter('lon', -76.627894)
        self.declare_parameter('heading', 0.0)
        self.declare_parameter('max_goal_distance', 70.0)
        # Heading is a map-frame yaw in degrees: +X=0, +Y=90.
        self.declare_parameter('navigation_timeout', 180.0)

        # Convert GPS latitude/longitude to a point in the map frame.
        self.converter = self.create_client(FromLL, '/fromLL')
        self.navigator = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def wait_for_result(self, future, timeout=None, cancel_on_timeout=True):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        if not future.done():
            if cancel_on_timeout:
                future.cancel()
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
        navigation_timeout = self.get_parameter('navigation_timeout').value

        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            raise ValueError(f'Invalid GPS target: latitude={lat}, longitude={lon}')
        if not math.isfinite(heading):
            raise ValueError(f'Invalid target heading: {heading}')
        if not math.isfinite(max_goal_distance) or max_goal_distance <= 0.0:
            raise ValueError(f'Invalid maximum goal distance: {max_goal_distance}')
        if not math.isfinite(navigation_timeout) or navigation_timeout <= 0.0:
            raise ValueError('navigation_timeout must be positive and finite')

        if not self.converter.wait_for_service(timeout_sec=30.0):
            raise RuntimeError('Timed out waiting for /fromLL')
        if not self.navigator.wait_for_server(timeout_sec=30.0):
            raise RuntimeError('Timed out waiting for /navigate_to_pose')
        self.wait_for_navigation_active()

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

        # Preserve a pending acceptance future: canceling this local future
        # cannot cancel a goal that the server may accept after the deadline.
        self.pending_goal = self.navigator.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self.goal_handle = self.wait_for_result(
            self.pending_goal, timeout=10.0, cancel_on_timeout=False)

        if not self.goal_handle.accepted:
            raise RuntimeError('Goal rejected :(')
        self.get_logger().info('Goal accepted :)')

        result = self.wait_for_result(
            self.goal_handle.get_result_async(), timeout=navigation_timeout)
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'Navigation finished with action status {result.status}')
        self.get_logger().info('Navigation succeeded')

    def feedback_callback(self, message):
        remaining = message.feedback.distance_remaining
        self.get_logger().info(f'{remaining:.1f} m left')

    def abort_action(self):
        if getattr(self, 'goal_handle', None) is None and getattr(self, 'pending_goal', None) is not None:
            self.goal_handle = self.wait_for_result(
                self.pending_goal, timeout=5.0, cancel_on_timeout=False)
        if getattr(self, 'goal_handle', None) is not None and self.goal_handle.accepted:
            response = self.wait_for_result(
                self.goal_handle.cancel_goal_async(), timeout=5.0)
            if not response.goals_canceling:
                self.get_logger().warning('Goal was already terminal or cancellation was rejected')


def main(args=None):
    rclpy.init(args=args)
    node = NavigationClient()
    status = 0
    try:
        node.navigate()
    except KeyboardInterrupt:
        status = 130
    except Exception as error:
        node.get_logger().error(str(error))
        status = 1
    finally:
        if status:
            try:
                node.abort_action()
            except Exception as error:
                node.get_logger().error(f'Could not confirm cancellation: {error}')
        node.destroy_node()
        rclpy.try_shutdown()
    if status:
        raise SystemExit(status)


if __name__ == '__main__':
    main()
