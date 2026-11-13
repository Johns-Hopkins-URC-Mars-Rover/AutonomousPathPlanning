import qwiic_icm20948
import math
import sys
import time
import smbus2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from icm20948_imu_interface.gyro_bias import GyroBiasCalibrator

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
        # Row-major 3x3 covariances reported in the Imu message, from a
        # stationary recording (scripts/estimate_imu_covariance.py).
        # angular_velocity_covariance in (rad/s)^2, linear_acceleration_covariance
        # in (m/s^2)^2. All zeros means "not measured": consumers read the
        # message as having no covariance.
        self.declare_parameter("angular_velocity_covariance", [0.0] * 9)
        self.declare_parameter("linear_acceleration_covariance", [0.0] * 9)
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        # Gyro bias removal. The rover must sit still for gyro_bias_samples
        # samples (5 s at 50 Hz) after launch while the bias is measured; no
        # IMU data is published until then. A fixed gyro_bias [x, y, z] in
        # rad/s skips calibration (empty list = calibrate at startup).
        self.declare_parameter("gyro_bias_samples", 250)
        self.declare_parameter("gyro_bias", [0.0])

        self.i2c_address = self.get_parameter("i2c_address").get_parameter_value().integer_value
        self.i2c_bus = self.get_parameter("i2c_bus").get_parameter_value().integer_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.pub_rate = self.get_parameter("pub_rate").get_parameter_value().integer_value
        self.gyro_cov = self._covariance("angular_velocity_covariance")
        self.accel_cov = self._covariance("linear_acceleration_covariance")
        diagnostics_topic = self.get_parameter("diagnostics_topic").value
        self.gyro_bias = None
        self.calibrator = None
        self._init_gyro_bias()

        self.imu_publisher = self.create_publisher(Imu, '/imu/data_raw', qos_profile_sensor_data)
        self.mag_publisher = self.create_publisher(
            MagneticField, '/imu/mag', qos_profile_sensor_data)
        self.diag_publisher = self.create_publisher(DiagnosticArray, diagnostics_topic, 10)

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

    def _init_gyro_bias(self):
        fixed = [float(v) for v in self.get_parameter("gyro_bias").value]
        if len(fixed) == 3:
            self.gyro_bias = tuple(fixed)
            self.get_logger().info(
                "Using fixed gyro bias [%.5f, %.5f, %.5f] rad/s, skipping calibration"
                % self.gyro_bias)
            return
        if len(fixed) > 1:
            raise ValueError(f"gyro_bias must have 3 values (x, y, z), got {len(fixed)}")
        samples = self.get_parameter("gyro_bias_samples").value
        self.calibrator = GyroBiasCalibrator(num_samples=samples)
        self._cal_start = None
        self._cal_last_log = 0.0
        self.get_logger().info(
            "CALIBRATING gyro bias: keep the rover completely still, about %.0f s. "
            "No IMU data is published until this finishes." % (samples / self.pub_rate))

    def _calibrate(self, gyro_scale):
        """Feed one raw sample to the calibrator. Returns True once bias is known."""
        cal = self.calibrator
        retries = cal.retries
        now = time.monotonic()
        if self._cal_start is None:
            self._cal_start = now
        finished = cal.add_sample(self.IMU.gxRaw * gyro_scale,
                                  self.IMU.gyRaw * gyro_scale,
                                  self.IMU.gzRaw * gyro_scale)
        if cal.retries != retries:
            self._cal_start = now
            self.get_logger().warn(
                "CALIBRATING: rover moved (spread %.2f deg/s, mean %.2f deg/s), "
                "restart #%d. Hold still, ~%.0f s from now."
                % (math.degrees(max(cal.last_std)),
                   math.degrees(max(abs(m) for m in cal.last_mean)),
                   cal.retries, cal.num_samples / self.pub_rate))
        elif finished:
            self.gyro_bias = cal.bias
            self.get_logger().info(
                "CALIBRATION DONE in %.1f s. Gyro bias (deg/s): x=%.3f y=%.3f z=%.3f "
                "| noise (std) %.3f deg/s. Publishing IMU data now."
                % (now - self._cal_start, *(math.degrees(b) for b in cal.bias),
                   math.degrees(max(cal.last_std))))
        elif now - self._cal_last_log >= 1.0 and cal.count > 0:
            self._cal_last_log = now
            n = cal.count
            elapsed = now - self._cal_start
            if n >= 10:
                remaining = elapsed / n * (cal.num_samples - n)
            else:  # too few samples to measure the rate yet: use the nominal one
                remaining = (cal.num_samples - n) / self.pub_rate
            mean, std = cal.running_stats()
            self.get_logger().info(
                "CALIBRATING %3d%% | ~%.1f s left | bias so far (deg/s) "
                "x=%.2f y=%.2f z=%.2f | motion %s"
                % (100 * n // cal.num_samples, remaining,
                   *(math.degrees(m) for m in mean),
                   "OK" if max(std) <= cal.max_std else "TOO HIGH"))
        return finished

    def _covariance(self, name):
        covariance = [float(v) for v in self.get_parameter(name).value]
        if len(covariance) != 9:
            raise ValueError(f"{name} must have 9 values (row-major 3x3), got {len(covariance)}")
        if any(covariance[i] < 0.0 for i in (0, 4, 8)):
            raise ValueError(f"{name} has a negative variance on the diagonal")
        return covariance

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
        mag_bias = 0.0

        if self.gyro_bias is None:
            self._calibrate(gyro_scale)
            if self.gyro_bias is None:
                status.level = DiagnosticStatus.WARN
                status.message = "Calibrating gyro bias, keep the rover still."
                status.values.append(KeyValue(
                    key="calibration_retries", value=str(self.calibrator.retries)))
                diagnostics_msg.status = [status]
                self.diag_publisher.publish(diagnostics_msg)
                return
            # The sample that completed calibration is dropped, not published.
            return

        gyro_bias = self.gyro_bias
        
        imu_msg = Imu()
        imu_msg.header.stamp = time_stamp
        imu_msg.header.frame_id = self.frame_id

        imu_msg.linear_acceleration.x = self.IMU.axRaw * acceleration_scale - acceleration_bias
        imu_msg.linear_acceleration.y = self.IMU.ayRaw * acceleration_scale - acceleration_bias
        imu_msg.linear_acceleration.z = self.IMU.azRaw * acceleration_scale - acceleration_bias

        imu_msg.angular_velocity.x = self.IMU.gxRaw * gyro_scale - gyro_bias[0]
        imu_msg.angular_velocity.y = self.IMU.gyRaw * gyro_scale - gyro_bias[1]
        imu_msg.angular_velocity.z = self.IMU.gzRaw * gyro_scale - gyro_bias[2]

        imu_msg.angular_velocity_covariance = self.gyro_cov
        imu_msg.linear_acceleration_covariance = self.accel_cov

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

        status.values += [KeyValue(key="gyro_bias_%s_rad_s" % axis, value="%.6f" % b)
                          for axis, b in zip("xyz", gyro_bias)]
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

