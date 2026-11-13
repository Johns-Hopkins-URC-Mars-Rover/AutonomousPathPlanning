import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from serial import SerialException

from .L76X import L76X


from .nmea import decimal_degrees, parse_rmc, parse_gga, ellipsoid_altitude, utc_timestamp


class GPSPublisher(Node):
    def __init__(self):
        super().__init__("l76x_gps_publisher")

        self.declare_parameter("baudrate", 9600)
        self.declare_parameter("frame_id", "gps_link")

        baudrate = int(self.get_parameter("baudrate").value)
        self.frame_id = self.get_parameter("frame_id").value

        self.variance = self.declare_parameter('position_variance', [0.0, 0.0, 0.0]).value
        if (len(self.variance) != 3 or not all(math.isfinite(v) and v >= 0 for v in self.variance)
                or (any(self.variance) and not all(v > 0 for v in self.variance))):
            raise ValueError('position_variance must be all zero (unknown) or 3 positive measured variances')
        self.add_on_set_parameters_callback(self.on_parameters)
        self.pending = {}
        self.gga = {}
        self.last_epoch = None
        self.publisher = self.create_publisher(NavSatFix, "/gps/fix", 10)

        self.gps = L76X()
        self.gps.L76X_Set_Baudrate(baudrate)

        self.gps.L76X_Exit_BackupMode()
        self.gps.L76X_Send_Command(self.gps.SET_NORMAL_MODE)

        self.gps.L76X_Send_Command("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        self.gps.L76X_Send_Command(self.gps.SET_POS_FIX_400MS)

        self.serial = self.gps.config.serial
        self.serial.timeout = 0 
        self.buffer = bytearray()

        self.last_rmc = time.monotonic()
        self.warned_silent = False

        self.timer = self.create_timer(0.05, self.read_serial)

        self.get_logger().info(f"Publishing /gps/fix at receiver baudrate {baudrate}")

    def on_parameters(self, parameters):
        """Let the startup calibrator replace position_variance while running."""
        from rcl_interfaces.msg import SetParametersResult
        for parameter in parameters:
            if parameter.name != 'position_variance':
                continue
            value = list(parameter.value)
            if (len(value) != 3 or not all(math.isfinite(v) and v > 0 for v in value)):
                return SetParametersResult(
                    successful=False, reason='position_variance needs 3 positive finite values')
            self.variance = value
            self.get_logger().info('position_variance set to %s' % value)
        return SetParametersResult(successful=True)

    def read_serial(self):
        try:
            available = self.serial.in_waiting

            if available:
                self.buffer.extend(self.serial.read(min(available, 4096)))

            while (b"\n" in self.buffer):
                line, _, remainder = self.buffer.partition(b"\n")
                self.buffer = bytearray(remainder)

                fields = parse_rmc(line)
                if fields is not None:
                    self.last_rmc = time.monotonic()
                    self.warned_silent = False
                    try:
                        epoch = float(fields[1])
                        if math.isfinite(epoch) and epoch != self.last_epoch:
                            self.pending[epoch] = (fields, time.monotonic())
                    except ValueError:
                        pass
                gga = parse_gga(line)
                if gga is not None:
                    try:
                        epoch = float(gga[1])
                        if math.isfinite(epoch):
                            self.gga[epoch] = (gga, time.monotonic())
                    except ValueError:
                        pass

            for epoch, (fields, received) in list(self.pending.items()):
                pair = self.gga.get(epoch)
                if pair is not None and time.monotonic() - pair[1] > 1.0:
                    pair = None
                if pair is not None or fields[2] == 'V' or time.monotonic() - received > 1.0:
                    self.publish_fix(fields, pair[0] if pair else None)
                    self.last_epoch = epoch
                    del self.pending[epoch]
            # Bound caches even with malformed or missing counterpart sentences.
            for cache in (self.pending, self.gga):
                for epoch in list(cache):
                    if time.monotonic() - cache[epoch][1] > 2.0:
                        del cache[epoch]

            if len(self.buffer) > 8192:
                self.buffer.clear()

            if (time.monotonic() - self.last_rmc > 5.0 and not self.warned_silent):
                self.get_logger().warning("No valid RMC sentences for 5 seconds. "
                    "Check receiver baudrate, wiring, and power.")
                self.warned_silent = True

        except (SerialException, OSError) as error:
            self.get_logger().error(f"Serial connection failed: {error}")
            self.timer.cancel()

    def publish_fix(self, fields, gga=None):
        msg = NavSatFix()

        try:
            stamp = utc_timestamp(fields)
        except ValueError as error:
            self.get_logger().warning(f'GPS acquisition time invalid: {error}', throttle_duration_sec=2.)
            return
        msg.header.stamp.sec = int(stamp)
        msg.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
        msg.header.frame_id = self.frame_id
        msg.status.service = 0

        msg.status.status = NavSatStatus.STATUS_NO_FIX
        msg.latitude = float("nan")
        msg.longitude = float("nan")
        msg.altitude = float("nan")

        msg.position_covariance_type = (NavSatFix.COVARIANCE_TYPE_UNKNOWN)

        if fields[2] == "A":
            if len(fields) > 12 and fields[12] == "N":
                self.publisher.publish(msg)
                return

            try:
                msg.latitude = decimal_degrees(fields[3], fields[4], is_latitude=True)
                msg.longitude = decimal_degrees(fields[5], fields[6], is_latitude=False)

            except ValueError:
                return

            msg.status.status = NavSatStatus.STATUS_FIX
            if gga is not None:
                try:
                    msg.altitude = ellipsoid_altitude(gga)
                except ValueError:
                    pass  # Raw horizontal fix remains available; fusion gate rejects unknown altitude.
            if all(v > 0 for v in self.variance):
                msg.position_covariance = [v if r == c else 0.
                                           for r, v in enumerate(self.variance) for c in range(3)]
                msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.publisher.publish(msg)

    def destroy_node(self):
        self.timer.cancel()
        if self.serial.is_open:
            self.serial.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = GPSPublisher()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()