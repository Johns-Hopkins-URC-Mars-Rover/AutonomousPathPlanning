import qwiic_icm20948
import math
import sys
import smbus2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class SMBusWrapper:
    """
    A direct wrapper that maps the SparkFun library's expected 
    API calls straight to a raw smbus2 file descriptor on the correct bus.
    """
    def __init__(self, bus_number):
        self.bus = smbus2.SMBus(bus_number)
        
    def readByte(self, address, commandCode):
        return self.bus.read_byte_data(address, commandCode)
        
    def writeByte(self, address, commandCode, value):
        return self.bus.write_byte_data(address, commandCode, value)

    def readBlock(self, address, commandCode, length):
        return self.bus.read_i2c_block_data(address, commandCode, length)

    def writeBlock(self, address, commandCode, value_list):
        return self.bus.write_i2c_block_data(address, commandCode, value_list)

    def isDeviceConnected(self, address):
        try:
            self.bus.read_byte_data(address, 0x00)
            return True
        except Exception:
            return False


class ImuPublisher(Node):
    def __init__(self):
        super().__init__("ImuPublisher")

        self.declare_parameter("i2c_address", 0x69)
        self.declare_parameter("i2c_bus", 7)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("pub_rate", 50)

        self.i2c_address = self.get_parameter("i2c_address").get_parameter_value().integer_value
        self.i2c_bus = self.get_parameter("i2c_bus").get_parameter_value().integer_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.pub_rate = self.get_parameter("pub_rate").get_parameter_value().integer_value

        self.imu_publisher = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.mag_publisher = self.create_publisher(MagneticField, '/imu/mag', 10)
        self.diag_publisher = self.create_publisher(DiagnosticArray, '/imu/diagnostics', 10)

        # Construct the custom SMBus wrapper targeting Bus 7 directly
        try:
            # Create a wrapper explicitly locked to the parameter bus (7)
            direct_i2c_driver = SMBusWrapper(self.i2c_bus)            

            # Pass the driver straight to the sensor class constructor
            self.IMU = qwiic_icm20948.QwiicIcm20948(address=self.i2c_address, i2c_driver=direct_i2c_driver)
            
        except Exception as e:
            self.get_logger().error(f"Failed to open smbus2 descriptor on Bus {self.i2c_bus}: {str(e)}")
            sys.exit(1)

        if not self.IMU.connected:
            self.get_logger().error(f"Hardware ping failed. Device not responding at address {hex(self.i2c_address)} on Bus {self.i2c_bus}.")
            raise RuntimeError("IMU physical hardware target missing or unreachable.")

        self.IMU.begin()
        self.IMU.setFullScaleRangeGyro(qwiic_icm20948.dps2000)
        self.IMU.setFullScaleRangeAccel(qwiic_icm20948.gpm16)
        self.get_logger().info(f"ICM20948 successfully initialized on Bus {self.i2c_bus} via direct SMBus descriptor!")
        
        self.timer = self.create_timer(1.0 / self.pub_rate, self.timer_callback)

    def timer_callback(self):
        if not self.IMU.dataReady():
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
            self.get_logger().error(f"Data fetch runtime crash: {str(e)}")
            status.level = DiagnosticStatus.ERROR
            status.message = "Status Read Failed."
            diagnostics_msg.status = [status]
            self.diag_publisher.publish(diagnostics_msg)
            return

        acceleration_scale = 9.80665 / 2048.0
        gyro_scale = math.pi / (180 * 16.4)
        mag_scale = 0.15e-6

        acceleration_bias = 0.0
        gyro_bias = 0.0
        mag_bias = 0.0
        
        imu_msg = Imu()
        imu_msg.header.stamp = time_stamp
        imu_msg.header.frame_id = self.frame_id

        imu_msg.linear_acceleration.x = self.IMU.axRaw * acceleration_scale - acceleration_bias
        imu_msg.linear_acceleration.y = self.IMU.ayRaw * acceleration_scale - acceleration_bias
        imu_msg.linear_acceleration.z = self.IMU.azRaw * acceleration_scale - acceleration_bias

        imu_msg.angular_velocity.x = self.IMU.gxRaw * gyro_scale - gyro_bias
        imu_msg.angular_velocity.y = self.IMU.gyRaw * gyro_scale - gyro_bias
        imu_msg.angular_velocity.z = self.IMU.gzRaw * gyro_scale - gyro_bias

        # An orientation covariance matrix with -1 in the first element indicates 
        # that orientation data is not covered by this raw topic.
        # Create an array of 9 floats, setting the first element to -1.0 to flag "no orientation data"
        imu_msg.orientation_covariance = [-1.0, 0.0, 0.0, 
                                           0.0, 0.0, 0.0, 
                                           0.0, 0.0, 0.0]


        mag_msg = MagneticField()
        mag_msg.header.stamp = time_stamp
        mag_msg.header.frame_id = self.frame_id

        mag_msg.magnetic_field.x = self.IMU.mxRaw * mag_scale - mag_bias
        mag_msg.magnetic_field.y = self.IMU.myRaw * mag_scale - mag_bias
        mag_msg.magnetic_field.z = self.IMU.mzRaw * mag_scale - mag_bias

        status.level = DiagnosticStatus.OK
        status.message = "Status Read Successful."
        diagnostics_msg.status = [status]

        self.imu_publisher.publish(imu_msg)
        self.mag_publisher.publish(mag_msg)
        self.diag_publisher.publish(diagnostics_msg)

        
def main(args=None):
    rclpy.init(args=args)
    imu_publisher = ImuPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

