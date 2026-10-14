import qwicc_icm20948
import time
import sys
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class ImuPublisher(Node):
    def __init__(self):
        super().__init("ImuPublisher")
        self.imu_publisher = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.mag_publisher = self.create_publisher(MagneticField, '/imu/mag', 10)
        self.diag_publisher = self.create_publisher(DiagnosticArray, '/imu/diagnostics', 10)
        self.IMU = qwiic_icm20948.QwiicIcm20948()

        assert(self.IMU.connected == True)
        self.IMU.begin()

    def timer_callback(self):
        if (not self.IMU.dataReady()):
            self.get_logger().info("Waiting to Receive Data from the IMU.")
            return
        
        self.IMU.getAgmt()
        time_stamp = self.get_clock().now().to_msg()
        acceleration_scale = 9.80665 / 2048.0
        gyro_scale = math.pi / (180 * 16.4)
        mag_scale = 0.15e-16

        imu_msg = Imu()
        imu_msg.header.stamp = time_stamp
        imu_msg.header.frame_id = "imu_link"

        imu_msg.linear_acceleration.x = self.imu.axRaw * acceleration_scale
        imu_msg.linear_acceleration.y = self.imu.ayRaw * acceleration_scale
        imu_msg.linear_acceleration.z = self.imu.azRaw * acceleration_scale

        imu_msg.angular_velocity.x = self.imu.gxRaw * gyro_scale
        imu_msg.angular_velocity.y = self.imu.gyRaw * gyro_scale
        imu_msg.angular_velocity.z = self.imu.gzRaw * gyro_scale

        imu_msg.orientation_covariance[0] = -1.0

        mag_msg = MagneticField()
        mag_msg.header.stamp = stamp
        mag_msg.header.frame_id = "imu_link"

        mag_msg.magnetic_field.x = self.imu.mxRaw * mag_scale
        mag_msg.magnetic_field.y = self.imu.myRaw * mag_scale
        mag_msg.magnetic_field.z = self.imu.mzRaw * mag_scale

        diagnostics_msg = DiagnosticArray()
        diagnostics_msg.header.stamp = stamp

        status = DiagnosticStatus()
        status.name = "icm20948-imu-interface/icm20948"
        status.hardware_id = "ICM-20948"
        status.level = DiagnosticStatus.OK
        status.message = "Statis Read Successfull."
        status.values = [KeyValue(key="frame_id", value = "imu_link")]

        diagnostics_msg.status = [status]

        imu_msg.data = f"IMU Data"
        mag_msg.data = f"Magnetic_Data"
        diag_msg.data = f"Diagnostics_Data" 
        self.imu_publisher.publish(imu_msg)
        self.mag_publisher.publish(mag_msg)
        self.diag_publisher.publish(diagnostics_msg)

        self.get_logger().info("Publishing: %s\n%s\n%s", imu_msg.data, mag_msg.data, diagnostics_msg.data)


def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        

        
