import qwiic_icm20948
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class ImuPublisher(Node):
    def __init__(self):
        super().__init__("ImuPublisher")

        self.declare_parameter("i2c_address", 0x69)
        self.i2c_address = self.get_parameter("i2c_address").get_parameter_value().integer_value

        self.declare_parameter("frame_id", "imu_link")
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        self.declare_parameter("pub_rate", 50)
        self.pub_rate = self.get_parameter("pub_rate").get_parameter_value().integer_value

        self.imu_publisher = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.mag_publisher = self.create_publisher(MagneticField, '/imu/mag', 10)
        self.diag_publisher = self.create_publisher(DiagnosticArray, '/imu/diagnostics', 10)

        self.IMU = qwiic_icm20948.QwiicIcm20948(address=self.i2c_address)

        if not self.IMU.connected:
            self.get_logger().error("The Qwiic ICM20948 device isn't connected to the system. Please check your connection.")

        self.IMU.begin()
        self.IMU.setFullScaleRangeGyro(qwiic_icm20948.dps2000)
        self.IMU.setFullScaleRangeAccel(qwiic_icm20948.gpm16)

        self.timer = self.create_timer(1.0 / self.pub_rate, self.timer_callback)

    def timer_callback(self):
        if (not self.IMU.dataReady()):
            self.get_logger().info("Waiting to Receive Data from the IMU.")
            return

        time_stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = "icm20948-imu-interface/icm20948"
        status.hardware_id = "ICM-20948"
        status.values = [KeyValue(key="frame_id", value=self.frame_id)]

        diagnostics_msg = DiagnosticArray()
        diagnostics_msg.header.stamp = time_stamp

        try:
            self.IMU.getAgmt()
        except Exception as e:
            self.get_logger().error(str(e))
            status.level = DiagnosticStatus.ERROR
            status.message = "Status Read Failed."
            diagnostics_msg.status = [status]
            self.diag_publisher.publish(diagnostics_msg)
            return

        acceleration_scale = 9.80665 / 2048.0
        gyro_scale = math.pi / (180 * 16.4)
        mag_scale = 0.15e-6

        imu_msg = Imu()
        imu_msg.header.stamp = time_stamp
        imu_msg.header.frame_id = self.frame_id

        imu_msg.linear_acceleration.x = self.IMU.axRaw * acceleration_scale
        imu_msg.linear_acceleration.y = self.IMU.ayRaw * acceleration_scale
        imu_msg.linear_acceleration.z = self.IMU.azRaw * acceleration_scale

        imu_msg.angular_velocity.x = self.IMU.gxRaw * gyro_scale
        imu_msg.angular_velocity.y = self.IMU.gyRaw * gyro_scale
        imu_msg.angular_velocity.z = self.IMU.gzRaw * gyro_scale

        imu_msg.orientation_covariance[0] = -1.0

        mag_msg = MagneticField()
        mag_msg.header.stamp = time_stamp
        mag_msg.header.frame_id = self.frame_id

        mag_msg.magnetic_field.x = self.IMU.mxRaw * mag_scale
        mag_msg.magnetic_field.y = self.IMU.myRaw * mag_scale
        mag_msg.magnetic_field.z = self.IMU.mzRaw * mag_scale

        status.level = DiagnosticStatus.OK
        status.message = "Status Read Successful."
        diagnostics_msg.status = [status]

        self.imu_publisher.publish(imu_msg)
        self.mag_publisher.publish(mag_msg)
        self.diag_publisher.publish(diagnostics_msg)

        self.get_logger().info("Published IMU, magnetometer, and diagnostics data.")


def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()