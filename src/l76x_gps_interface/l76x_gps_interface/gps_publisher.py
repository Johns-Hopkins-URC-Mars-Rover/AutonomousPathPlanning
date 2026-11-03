import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from serial import SerialException

from .L76X import L76X


def decimal_degrees(value, hemisphere, is_latitude):
    """Convert NMEA ddmm.mmmm / dddmm.mmmm to signed degrees."""
    allowed = ("N", "S") if is_latitude else ("E", "W")
    if hemisphere not in allowed:
        raise ValueError("Invalid hemisphere")

    raw = float(value)
    if not math.isfinite(raw) or raw < 0:
        raise ValueError("Invalid coordinate")

    degrees = int(raw // 100)
    minutes = raw - degrees * 100

    if not 0 <= minutes < 60:
        raise ValueError("Invalid minutes")

    result = degrees + minutes / 60.0
    limit = 90 if is_latitude else 180

    if result > limit:
        raise ValueError("Coordinate out of range")

    return -result if hemisphere in ("S", "W") else result


def parse_rmc(line):
    """Return RMC fields only for a checksum-valid GP/GN sentence."""
    try:
        text = line.decode("ascii").strip()

        if not text.startswith("$"):
            return None

        body, checksum = text[1:].split("*")
        if len(checksum) != 2:
            return None

        calculated = 0
        for character in body:
            calculated ^= ord(character)

        if calculated != int(checksum, 16):
            return None

        fields = body.split(",")
        if fields[0] not in ("GPRMC", "GNRMC"):
            return None

        if len(fields) < 10 or fields[2] not in ("A", "V"):
            return None

        return fields

    except (UnicodeDecodeError, ValueError):
        return None


class GPSPublisher(Node):
    def __init__(self):
        super().__init__("l76x_gps_publisher")

        self.declare_parameter("baudrate", 9600)
        self.declare_parameter("frame_id", "gps_link")

        baudrate = int(self.get_parameter("baudrate").value)
        self.frame_id = self.get_parameter("frame_id").value

        self.publisher = self.create_publisher(NavSatFix, "/gps/fix", 10)

        self.gps = L76X()
        self.gps.L76X_Set_Baudrate(baudrate)

        self.gps.L76X_Exit_BackupMode()
        self.gps.L76X_Send_Command(self.gps.SET_NORMAL_MODE)

        self.gps.L76X_Send_Command("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        self.gps.L76X_Send_Command(self.gps.SET_POS_FIX_400MS)

        self.serial = self.gps.config.serial
        self.serial.timeout = 0 
        self.buffer = bytearray()

        self.last_rmc = time.monotonic()
        self.warned_silent = False

        self.timer = self.create_timer(0.05, self.read_serial)

        self.get_logger().info(f"Publishing /gps/fix at receiver baudrate {baudrate}")

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
                    self.publish_fix(fields)

            if len(self.buffer) > 8192:
                self.buffer.clear()

            if (time.monotonic() - self.last_rmc > 5.0 and not self.warned_silent):
                self.get_logger().warning("No valid RMC sentences for 5 seconds. "
                    "Check receiver baudrate, wiring, and power.")
                self.warned_silent = True

        except (SerialException, OSError) as error:
            self.get_logger().error(f"Serial connection failed: {error}")
            self.timer.cancel()

    def publish_fix(self, fields):
        msg = NavSatFix()

        msg.header.stamp = self.get_clock().now().to_msg()
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

        self.publisher.publish(msg)

    def destroy_node(self):
        self.timer.cancel()
        if self.serial.is_open:
            self.serial.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

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