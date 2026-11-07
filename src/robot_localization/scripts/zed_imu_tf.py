#!/usr/bin/env python3
"""Publish the ZED SDK's calibrated camera/IMU transform without motion TF."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from sensor_contracts import normalize_quaternion


class CalibrationTf(Node):
    def __init__(self):
        super().__init__('zed_imu_calibration_tf')
        topic = self.declare_parameter('calibration_topic', '/zed/zed_node/left_cam_imu_transform').value
        self.child = self.declare_parameter('imu_frame', 'zed_imu_link').value
        self.sent = False
        self.broadcaster = StaticTransformBroadcaster(self)
        self.subscription = self.create_subscription(TransformStamped, topic, self.receive, qos_profile_sensor_data)
        self.timer = self.create_timer(5., self.check)

    def receive(self, msg):
        if self.sent:
            return
        q, t = msg.transform.rotation, msg.transform.translation
        try:
            normalize_quaternion((q.x, q.y, q.z, q.w))
            if (not msg.header.frame_id or msg.header.frame_id == msg.child_frame_id or
                    msg.child_frame_id != self.child or
                    not all(math.isfinite(v) for v in (t.x, t.y, t.z))):
                raise ValueError('invalid calibration frames/translation; inspect the actual ZED calibration message')
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=5.)
            return
        self.broadcaster.sendTransform(msg)
        self.sent = True
        self.get_logger().info(f'Published calibrated static TF {msg.header.frame_id} -> {msg.child_frame_id}')

    def check(self):
        if not self.sent:
            self.get_logger().warning('Waiting for ZED camera/IMU calibration. Verify sensors.publish_cam_imu_transf=true and calibration_topic; no identity TF will be invented.')


def main():
    rclpy.init()
    node = CalibrationTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
