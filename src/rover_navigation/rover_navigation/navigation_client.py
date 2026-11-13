import math
import time
from typing import NamedTuple

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import GetCostmap
from robot_localization.srv import FromLL
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer, TransformException, TransformListener


class Waypoint(NamedTuple):
    x: float
    y: float
    is_final: bool


class CostmapNotReady(RuntimeError):
    """The rolling window has not caught up with the rover yet."""


def select_waypoint(position, target, costmap, max_distance, margin):
    """Keep the final target and select an in-window goal toward it."""
    if costmap.header.frame_id != 'map':
        raise RuntimeError(
            f'Expected the global costmap in map, received {costmap.header.frame_id!r}')
    metadata = costmap.metadata
    resolution = metadata.resolution
    origin = metadata.origin.position
    if (not all(math.isfinite(value) for value in
                (position.x, position.y, target.x, target.y,
                 origin.x, origin.y, resolution, max_distance, margin)) or
            resolution <= 0.0 or metadata.size_x <= 2 or metadata.size_y <= 2):
        raise RuntimeError('The global costmap has invalid or uninitialized bounds')
    if max_distance <= 0.0 or margin <= 0.0:
        raise ValueError('Waypoint distance and margin must be positive')

    padding = max(margin, 2.0 * resolution)
    min_x, min_y = origin.x + padding, origin.y + padding
    max_x = origin.x + metadata.size_x * resolution - padding
    max_y = origin.y + metadata.size_y * resolution - padding
    if min_x >= max_x or min_y >= max_y:
        raise RuntimeError('The global costmap is too small for the waypoint margin')
    if not (min_x <= position.x <= max_x and min_y <= position.y <= max_y):
        raise CostmapNotReady('Waiting for the rolling costmap to recenter on the rover')

    dx, dy = target.x - position.x, target.y - position.y
    distance = math.hypot(dx, dy)
    if distance == 0.0:
        return Waypoint(target.x, target.y, True)
    scale = min(1.0, max_distance / distance)
    for delta, current, lower, upper in (
            (dx, position.x, min_x, max_x), (dy, position.y, min_y, max_y)):
        if delta > 0.0:
            scale = min(scale, (upper - current) / delta)
        elif delta < 0.0:
            scale = min(scale, (lower - current) / delta)
    if scale <= 0.0:
        raise CostmapNotReady('Waiting for the rolling costmap to provide forward space')
    return Waypoint(position.x + scale * dx, position.y + scale * dy, scale >= 1.0)


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
        self.declare_parameter('waypoint_margin', 1.0)
        # Heading is a map-frame yaw in degrees: +X=0, +Y=90.
        self.declare_parameter('navigation_timeout', 600.0)

        # Convert GPS latitude/longitude to a point in the map frame.
        self.converter = self.create_client(FromLL, '/fromLL')
        self.costmap = self.create_client(GetCostmap, '/global_costmap/get_costmap')
        self.navigator = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

    def get_current_position(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
            try:
                transform = self.tf_buffer.lookup_transform('map', 'base_link', Time())
                return transform.transform.translation
            except TransformException:
                continue
        raise TimeoutError('Timed out waiting for the rover pose in map')

    def next_waypoint(self, target, max_distance, margin):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            position = self.get_current_position(timeout=max(0.0, deadline - time.monotonic()))
            response = self.wait_for_result(
                self.costmap.call_async(GetCostmap.Request()),
                timeout=max(0.0, deadline - time.monotonic()))
            try:
                return select_waypoint(position, target, response.map, max_distance, margin)
            except CostmapNotReady:
                # Re-read the map after its next rolling-window update.
                rclpy.spin_once(self, timeout_sec=min(0.2, max(0.0, deadline - time.monotonic())))
        raise TimeoutError('The rolling global costmap did not catch up with the rover')

    def navigate(self):
        lat = self.get_parameter('lat').value
        lon = self.get_parameter('lon').value
        heading = self.get_parameter('heading').value
        max_goal_distance = self.get_parameter('max_goal_distance').value
        waypoint_margin = self.get_parameter('waypoint_margin').value
        navigation_timeout = self.get_parameter('navigation_timeout').value

        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            raise ValueError(f'Invalid GPS target: latitude={lat}, longitude={lon}')
        if not math.isfinite(heading):
            raise ValueError(f'Invalid target heading: {heading}')
        if not math.isfinite(max_goal_distance) or max_goal_distance <= 0.0:
            raise ValueError(f'Invalid maximum waypoint distance: {max_goal_distance}')
        if not math.isfinite(waypoint_margin) or waypoint_margin <= 0.0:
            raise ValueError('waypoint_margin must be positive and finite')
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
        if not all(math.isfinite(value) for value in (point.x, point.y)):
            raise RuntimeError('The GPS conversion returned a non-finite map point')

        if not self.costmap.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('Timed out waiting for /global_costmap/get_costmap')

        self.get_logger().info(f'Final GPS target: map x={point.x:.2f}, y={point.y:.2f}')
        while True:
            waypoint = self.next_waypoint(point, max_goal_distance, waypoint_margin)
            yaw = (math.radians(heading) if waypoint.is_final else
                   math.atan2(point.y - waypoint.y, point.x - waypoint.x))
            self.navigate_waypoint(waypoint, yaw, navigation_timeout)
            if waypoint.is_final:
                self.get_logger().info('Navigation succeeded at the final GPS target')
                return

    def navigate_waypoint(self, waypoint, yaw, timeout):

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = waypoint.x
        goal.pose.pose.position.y = waypoint.y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        kind = 'final goal' if waypoint.is_final else 'intermediate waypoint'
        self.get_logger().info(
            f'Sending {kind}: x={waypoint.x:.2f}, y={waypoint.y:.2f}, '
            f'heading={math.degrees(yaw):.1f} deg')

        # Preserve a pending acceptance future: canceling this local future
        # cannot cancel a goal that the server may accept after the deadline.
        self.goal_handle = None
        self.pending_goal = self.navigator.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self.goal_handle = self.wait_for_result(
            self.pending_goal, timeout=10.0, cancel_on_timeout=False)

        if not self.goal_handle.accepted:
            raise RuntimeError('Goal rejected :(')
        self.get_logger().info('Goal accepted :)')

        result = self.wait_for_result(
            self.goal_handle.get_result_async(), timeout=timeout)
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'Navigation finished with action status {result.status}')
        self.goal_handle = None
        self.pending_goal = None

    def feedback_callback(self, message):
        remaining = message.feedback.distance_remaining
        self.get_logger().info(f'{remaining:.1f} m left to current waypoint')

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
