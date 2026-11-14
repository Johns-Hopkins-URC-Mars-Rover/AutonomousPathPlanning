"""Last autonomous command filter for stale inputs and localization jumps.

Humble Collision Monitor ignores expired clouds. This guard supplies the missing
freshness stop and a command watchdog. It does not infer terrain traversability
or repair heading calibration; hardware emergency stopping remains independent.
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from .wait_for_localization import valid_odometry


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def planar_yaw(q):
    return math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                      1.0 - 2.0 * (q.y*q.y + q.z*q.z))


def finite_cloud(message):
    """Reject empty/all-invalid depth without a Python loop over every pixel."""
    fields = {field.name: field for field in message.fields}
    if not message.width or not message.height or not {'x', 'y', 'z'} <= fields.keys():
        return False
    try:
        formats = []
        endian = '>' if message.is_bigendian else '<'
        for key in ('x', 'y', 'z'):
            field = fields[key]
            if field.count != 1 or field.datatype not in (7, 8):
                return False
            formats.append(endian + ('f4' if field.datatype == 7 else 'f8'))
        dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': formats,
                          'offsets': [fields[k].offset for k in ('x', 'y', 'z')],
                          'itemsize': message.point_step})
        points = np.ndarray((message.height, message.width), dtype=dtype,
                            buffer=message.data,
                            strides=(message.row_step, message.point_step))
        return bool((np.isfinite(points['x']) & np.isfinite(points['y'])
                     & np.isfinite(points['z'])).any())
    except (TypeError, ValueError):
        return False


def pose_jump(previous, current, elapsed, distance_limit, angle_limit):
    """Allow bounded real motion; detect sudden map-frame corrections."""
    distance = math.hypot(current[0]-previous[0], current[1]-previous[1])
    angle = abs(math.atan2(math.sin(current[2]-previous[2]),
                          math.cos(current[2]-previous[2])))
    return (distance > distance_limit + 0.5 * max(0.0, elapsed)
            or angle > angle_limit + 0.7 * max(0.0, elapsed))


def bounded_command(message, max_linear, max_angular):
    """Never invert motor signs or allow autonomous reverse/sideways commands."""
    components = (message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z)
    if not all(math.isfinite(v) for v in components):
        raise ValueError('non-finite velocity command')
    if message.linear.x < -1e-6 or abs(message.linear.y) > 1e-6:
        raise ValueError('reverse/sideways autonomous command rejected')
    result = Twist()
    result.linear.x = min(max_linear, max(0.0, message.linear.x))
    result.angular.z = max(-max_angular, min(max_angular, message.angular.z))
    return result


class VelocityGuard(Node):
    def __init__(self):
        super().__init__('velocity_guard')
        self.limits = {}
        for name, default in {
            'command_timeout': 0.3, 'sensor_timeout': 0.75,
            'odometry_timeout': 0.75, 'gps_timeout': 2.0,
            'transform_timeout': 0.75, 'startup_hold': 2.0,
            'jump_distance': 1.0, 'jump_angle': 0.7,
            'max_linear': 0.25, 'max_angular': 0.35,
        }.items():
            value = float(self.declare_parameter(name, default).value)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite')
            self.limits[name] = value
        input_topic = self.declare_parameter('input_topic', '/cmd_vel_checked').value
        output_topic = self.declare_parameter(
            'output_topic', '/diff_drive_controller/cmd_vel_unstamped').value
        cloud_topic = self.declare_parameter(
            'cloud_topic', '/zed/zed_node/point_cloud/cloud_registered').value
        if input_topic == output_topic:
            raise ValueError('Guard input and output topics must differ')
        self.output = self.create_publisher(Twist, output_topic, 10)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ready_pub = self.create_publisher(Bool, '/navigation_guard/ready', status_qos)
        self.status_pub = self.create_publisher(String, '/navigation_guard/status', status_qos)
        self.received = {}
        self.source_frames = {}
        self.last_global = None
        self.latched_fault = ''
        self.armed = False
        self.healthy_since = None
        self.last_reason = None
        self.command = Twist()
        self.command_received = None
        self.command_error = ''
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.subscriptions_ = [
            self.create_subscription(Twist, input_topic, self.receive_command, 10),
            self.create_subscription(PointCloud2, cloud_topic, self.receive_cloud,
                                     qos_profile_sensor_data),
            self.create_subscription(Imu, '/imu/data', self.receive_imu,
                                     qos_profile_sensor_data),
        ]
        for name in ('local', 'global', 'gps'):
            self.subscriptions_.append(self.create_subscription(
                Odometry, '/odometry/' + name,
                lambda msg, name=name: self.receive_odometry(name, msg),
                qos_profile_sensor_data))
        # Keep watchdogs running when simulation time pauses or jumps.
        self.timer = self.create_timer(
            0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def record(self, name, message):
        self.received[name] = (time.monotonic(), stamp_seconds(message.header.stamp))

    def receive_cloud(self, message):
        if message.header.frame_id and finite_cloud(message):
            self.record('cloud', message)
            self.source_frames['cloud'] = message.header.frame_id
        else:
            self.received.pop('cloud', None)

    def receive_imu(self, message):
        q, w = message.orientation, message.angular_velocity
        values = (q.x, q.y, q.z, q.w, w.x, w.y, w.z)
        if (message.header.frame_id and all(math.isfinite(v) for v in values)
                and abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.0) < 0.05):
            self.record('imu', message)
            self.source_frames['imu'] = message.header.frame_id
        else:
            self.received.pop('imu', None)

    def receive_odometry(self, name, message):
        expected_frame = 'odom' if name == 'local' else 'map'
        valid_gps_covariance = name != 'gps' or (
            message.pose.covariance[0] > 0.0 and message.pose.covariance[7] > 0.0)
        if (not valid_odometry(message) or message.header.frame_id != expected_frame
                or not valid_gps_covariance):
            self.received.pop(name, None)
            return
        if name == 'global':
            p = message.pose.pose.position
            current = (p.x, p.y, planar_yaw(message.pose.pose.orientation))
            stamp = stamp_seconds(message.header.stamp)
            if self.armed and self.last_global is not None:
                old_stamp, previous = self.last_global
                if stamp < old_stamp or pose_jump(
                        previous, current, stamp-old_stamp,
                        self.limits['jump_distance'], self.limits['jump_angle']):
                    self.latched_fault = 'localization jumped; stop and restart after checking GPS/heading'
            self.last_global = (stamp, current)
        self.record(name, message)

    def receive_command(self, message):
        try:
            self.command = bounded_command(
                message, self.limits['max_linear'], self.limits['max_angular'])
            self.command_error = ''
            self.command_received = time.monotonic()
        except ValueError as error:
            self.command = Twist()
            self.command_error = str(error)
            self.command_received = None

    def unavailable_reason(self, now):
        if self.latched_fault:
            return self.latched_fault
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        for name in ('cloud', 'imu', 'local', 'global', 'gps'):
            if name not in self.received:
                return f'waiting for valid {name} input'
            receipt, stamp = self.received[name]
            timeout = self.limits['gps_timeout' if name == 'gps' else
                                  'odometry_timeout' if name in ('local', 'global') else
                                  'sensor_timeout']
            if now-receipt > timeout or not -0.1 <= ros_now-stamp <= timeout:
                return f'{name} input is stale or from a different clock'
        for parent, child in (('map', 'odom'), ('odom', 'base_link')):
            try:
                transform = self.buffer.lookup_transform(parent, child, Time())
            except TransformException:
                return f'missing transform {parent} -> {child}'
            age = ros_now - stamp_seconds(transform.header.stamp)
            if not -0.1 <= age <= self.limits['transform_timeout']:
                return f'stale transform {parent} -> {child}'
        for name in ('cloud', 'imu'):
            if not self.buffer.can_transform('base_link', self.source_frames[name], Time()):
                return f'missing {name} mount transform to base_link'
        return self.command_error

    def tick(self):
        now = time.monotonic()
        reason = self.unavailable_reason(now)
        if reason:
            self.healthy_since = None
        elif self.healthy_since is None:
            self.healthy_since = now
        ready = not reason and now-self.healthy_since >= self.limits['startup_hold']
        if not reason and not ready:
            reason = 'holding stationary while inputs stabilize'
        if ready:
            self.armed = True
        output = Twist()
        if ready and self.command_received is not None:
            if now-self.command_received <= self.limits['command_timeout']:
                output = self.command
        self.output.publish(output)
        status = reason or 'ready'
        self.ready_pub.publish(Bool(data=ready))
        self.status_pub.publish(String(data=status))
        if status != self.last_reason:
            self.get_logger().info('Navigation guard: ' + status)
            self.last_reason = status


def main(args=None):
    rclpy.init(args=args)
    node = VelocityGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.output.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
