"""Publish an idealized body-frame magnetic field from Gazebo IMU attitude."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField


def world_to_body(quaternion, vector):
    """Rotate an ENU world vector into the body frame."""
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1.0e-9:
        raise ValueError('IMU orientation is not a valid quaternion')
    x, y, z, w = (value / norm for value in (x, y, z, w))

    # Rotation matrix body -> world, transposed for world -> body.
    rotation = (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
         2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
         2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w),
         1 - 2 * (x * x + y * y)),
    )
    return tuple(sum(rotation[row][column] * vector[row]
                     for row in range(3)) for column in range(3))


class MagneticFieldSimulator(Node):
    """Create `/imu/mag` with the same message contract as the real ICM."""

    def __init__(self):
        super().__init__('magnetic_field_simulator')
        self.declare_parameter(
            'field_enu_tesla', [0.0, 25.0e-6, -40.0e-6])
        field = self.get_parameter('field_enu_tesla').value
        if len(field) != 3 or not all(math.isfinite(value) for value in field):
            raise ValueError('field_enu_tesla must contain three finite values')
        self.field = tuple(float(value) for value in field)
        self.publisher = self.create_publisher(
            MagneticField, '/imu/mag', qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            Imu, '/imu/data_raw', self.receive, qos_profile_sensor_data)

    def receive(self, message):
        q = message.orientation
        try:
            x, y, z = world_to_body((q.x, q.y, q.z, q.w), self.field)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)
            return
        output = MagneticField()
        output.header = message.header
        output.magnetic_field.x = x
        output.magnetic_field.y = y
        output.magnetic_field.z = z
        output.magnetic_field_covariance = [
            1.0e-12, 0.0, 0.0,
            0.0, 1.0e-12, 0.0,
            0.0, 0.0, 1.0e-12,
        ]
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = MagneticFieldSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
