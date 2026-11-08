#!/usr/bin/env python3
"""Validate sensor contracts before robot_localization; report acceptance and age."""
import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tf2_ros import Buffer, TransformException, TransformListener

from sensor_contracts import base_pose, validate_covariance, validate_fix, validate_stamp, normalize_quaternion


class SensorInputs(Node):
    def __init__(self):
        super().__init__('sensor_inputs')
        self.base = self.declare_parameter('base_frame', 'base_link').value
        self.world = self.declare_parameter('odom_frame', 'odom').value
        self.max_age = self.declare_parameter('max_age', .5).value
        if not math.isfinite(self.max_age) or self.max_age <= 0:
            raise ValueError('max_age must be finite and positive')
        mode = self.declare_parameter('mode', 'local').value
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.states = {}
        self.handles = []
        self.diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        if mode == 'local':
            self.add('camera', Odometry, 'zed_odom_topic', '/zed/zed_node/odom',
                     '/localization/zed_odom_base', self.camera)
            self.add('imu', Imu, 'fused_imu_topic', '/imu/data',
                     '/localization/imu', self.imu)
        elif mode == 'gps':
            self.add('heading', Imu, 'heading_topic', '/imu/heading',
                     '/localization/heading', self.heading)
            self.add('gps', NavSatFix, 'gps_topic', '/gps/fix', '/gps/validated', self.gps)
        else:
            raise ValueError('mode must be local or gps')
        self.timer = self.create_timer(1., self.report)

    def add(self, key, message_type, parameter, default_topic, output, convert):
        topic = self.declare_parameter(parameter, default_topic).value
        if topic == output:
            raise ValueError('input and validated output topics must differ')
        state = {'topic': topic, 'stamp': None, 'accepted': 0, 'rejected': 0,
                 'reason': 'waiting for first accepted measurement'}
        self.states[key] = state
        publisher = self.create_publisher(message_type, output, 10)

        def callback(msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            try:
                validate_stamp(stamp, self.get_clock().now().nanoseconds * 1e-9,
                               state['stamp'], self.max_age)
                if not msg.header.frame_id:
                    raise ValueError('measurement frame_id is empty')
                if self.count_publishers(topic) != 1:
                    raise ValueError('expected exactly one input publisher; stop duplicate hardware/simulation launches')
                result = convert(msg)
            except (ValueError, TransformException) as error:
                state['rejected'] += 1
                state['reason'] = str(error)
                self.get_logger().warning(f'{key}: rejected input: {error}', throttle_duration_sec=2.)
                return
            state['stamp'] = stamp
            state['accepted'] += 1
            state['reason'] = 'accepted'
            publisher.publish(result)
        self.handles.append(self.create_subscription(message_type, topic, callback, qos_profile_sensor_data))
        self.handles.append(publisher)

    def mount(self, frame):
        # Mounts are rigid; the latest calibrated static TF is sufficient.
        return self.buffer.lookup_transform(frame, self.base, Time()).transform

    def camera(self, msg):
        if msg.header.frame_id != self.world:
            raise ValueError(f'camera pose must use {self.world}; got {msg.header.frame_id}')
        if not msg.child_frame_id:
            raise ValueError('camera child_frame_id is empty')
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if msg.child_frame_id == self.base:
            t, rotation = (0., 0., 0.), (0., 0., 0., 1.)
        else:
            mount = self.mount(msg.child_frame_id)
            t = (mount.translation.x, mount.translation.y, mount.translation.z)
            rotation = (mount.rotation.x, mount.rotation.y, mount.rotation.z, mount.rotation.w)
        position, orientation, covariance = base_pose(
            (p.x, p.y, p.z), (q.x, q.y, q.z, q.w), t, rotation, msg.pose.covariance)
        out = copy.deepcopy(msg)
        out.child_frame_id = self.base
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = position
        out.pose.pose.orientation.x, out.pose.pose.orientation.y, out.pose.pose.orientation.z, out.pose.pose.orientation.w = orientation
        out.pose.covariance = covariance
        # Do not mislabel camera-frame twist as base-frame data. The EKFs use pose only.
        out.twist = type(out.twist)()
        return out

    def imu(self, msg):
        if not all(math.isfinite(v) for v in
                   (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)):
            raise ValueError('IMU angular velocity is nonfinite')
        validate_covariance(msg.angular_velocity_covariance, 3)
        if msg.header.frame_id != self.base:
            self.mount(msg.header.frame_id)
        return msg

    def heading(self, msg):
        q = msg.orientation
        normalize_quaternion((q.x, q.y, q.z, q.w))
        validate_covariance(msg.orientation_covariance, 3)
        if msg.header.frame_id != self.base:
            self.mount(msg.header.frame_id)
        return msg

    def gps(self, msg):
        validate_fix(msg.status.status, msg.latitude, msg.longitude, msg.altitude,
                     msg.position_covariance_type, msg.position_covariance)
        if msg.header.frame_id != self.base:
            self.mount(msg.header.frame_id)
        return msg

    def report(self):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        now = self.get_clock().now().nanoseconds * 1e-9
        for key, state in self.states.items():
            status = DiagnosticStatus()
            status.name = self.get_name() + '/' + key
            status.hardware_id = state['topic']
            age = None if state['stamp'] is None else now - state['stamp']
            publishers = self.count_publishers(state['topic'])
            healthy = (publishers == 1 and age is not None and
                       0 <= age <= self.max_age and state['reason'] == 'accepted')
            status.level = DiagnosticStatus.OK if healthy else DiagnosticStatus.ERROR
            status.message = 'accepting fresh measurements' if healthy else state['reason']
            if age is not None and age > self.max_age:
                status.message = 'accepted input is stale; EKF output may be prediction only'
            if publishers != 1:
                status.message = f'expected exactly one input publisher; found {publishers}'
            status.values = [KeyValue(key=k, value=str(v)) for k, v in
                             dict(accepted=state['accepted'], rejected=state['rejected'],
                                  age_s=age, publishers=publishers).items()]
            array.status.append(status)
        self.diag.publish(array)


def main():
    rclpy.init()
    node = None
    try:
        node = SensorInputs()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
