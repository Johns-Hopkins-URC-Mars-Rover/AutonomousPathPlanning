"""Check Gazebo Classic ROS clock publication without any rover assets."""
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data


def main():
    rclpy.init()
    node = rclpy.create_node('gazebo_environment_smoke')
    stamps = []
    subscription = node.create_subscription(
        Clock, '/clock',
        lambda msg: stamps.append(msg.clock.sec + msg.clock.nanosec * 1e-9),
        qos_profile_sensor_data)
    with open('/tmp/gazebo-environment.log', 'w', encoding='utf-8') as log:
        process = subprocess.Popen(
            ['xvfb-run', '-a', 'gzserver', '--verbose',
             '/usr/share/gazebo-11/worlds/empty.world',
             '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 90
            while time.monotonic() < deadline and process.poll() is None:
                rclpy.spin_once(node, timeout_sec=0.2)
                if len(stamps) >= 10 and stamps[-1] > stamps[0]:
                    print(f'PASS Gazebo clock: {len(stamps)} messages, '
                          f'{stamps[0]:.3f} -> {stamps[-1]:.3f} seconds', flush=True)
                    return
            log.flush()
            print(Path('/tmp/gazebo-environment.log').read_text()[-12000:], flush=True)
            raise RuntimeError('No advancing Gazebo ROS clock')
        finally:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
