
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

import numpy as np
import math
from collections import deque


class GPSCovariance(Node):

    def __init__(self):
        super().__init__('gps_covariance')

        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )

        self.samples = []
        self.count = 0

        self.reference = None
        self.R = 6378137.0  # Earth radius in meters

        self.get_logger().info("Collecting GPS data...")

    def gps_callback(self, msg):

        if msg.status.status < 0:
            return

        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude

        if not all(math.isfinite(v) for v in [lat, lon, alt]):
            return

        # Set initial reference position
        if self.reference is None:
            self.reference = (lat, lon, alt)

        lat0, lon0, alt0 = self.reference

        # Convert GPS coordinates to local meters (East, North, Up)
        x = self.R * math.cos(math.radians(lat0)) * math.radians(lon - lon0)
        y = self.R * math.radians(lat - lat0)
        z = alt - alt0

        self.samples.append([x, y, z])
        self.count += 1

        # Calculate covariance every 10 messages
        # after collecting at least 100 samples
        if self.count % 300 == 0 and self.count >= 100:
            self.calculate_covariance()
    

    def calculate_covariance(self):
        data = np.array(self.samples)
        
        covariance = np.cov(data, rowvar=False)

        self.get_logger().info(
            f"\nGPS Covariance Matrix (m^2):\n{covariance}"
        )

        std_dev = np.sqrt(np.diag(covariance))

        self.get_logger().info(
            f"Standard deviations (m): {std_dev}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = GPSCovariance()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
