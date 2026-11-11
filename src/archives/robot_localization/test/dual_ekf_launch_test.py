"""Real EKFs/navsat: full inputs, ZED-only odometry, and dropout/recovery."""
import math
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import TransformStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster


def generate_test_description():
    bringup = os.environ.get('ROVER_TEST_BRINGUP') == '1'
    package = 'rover_bringup' if bringup else 'robot_localization'
    file = 'rover.launch.py' if bringup else 'dual_ekf.launch.py'
    path = os.path.join(get_package_share_directory(package), 'launch', file)
    arguments = {
        'enable_global': 'true', 'heading_mode': 'startup',
        'initial_heading_deg': '90', 'publish_approximate_mounts': 'true',
    }
    if os.environ.get('ROVER_TEST_PROFILE') == 'zed_only':
        arguments.update(use_wheel_odom='false', use_zed_imu='false', use_icm_imu='false')
    if bringup:
        # Synthetic sensor publishers replace drivers; no legacy packages or
        # hardware drivers are installed in this independence test environment.
        arguments.update(enable_localization='true', enable_zed='false',
                         enable_imu='false', enable_gps='false')
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', os.environ.get('ROVER_TEST_DOMAIN_ID', '198')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                 launch_arguments=arguments.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestDualEkf(unittest.TestCase):
    def test_gps_correction_and_independent_local_motion(self):
        # launch's environment substitutions affect child processes; initialize
        # this in-process test node on the same explicit domain as those nodes.
        rclpy.init(domain_id=int(os.environ.get('ROVER_TEST_DOMAIN_ID', '198')))
        node = rclpy.create_node('dual_ekf_test_inputs')
        profile = os.environ.get('ROVER_TEST_PROFILE', 'full')
        self.assertIn(profile, ('full', 'missing_wheel', 'zed_only', 'dropouts'))
        duration = 28.0 if profile == 'dropouts' else 19.0
        messages = {key: [] for key in ('local', 'global', 'gps', 'filtered', 'tf', 'diag')}
        diagnostics = []
        handles = []
        for key, topic in (('local', '/odometry/local'), ('global', '/odometry/global'),
                           ('gps', '/odometry/gps')):
            handles.append(node.create_subscription(
                Odometry, topic, lambda msg, key=key: messages[key].append(msg), 100))
        handles.append(node.create_subscription(
            NavSatFix, '/gps/filtered', messages['filtered'].append, 10))
        handles.append(node.create_subscription(
            TFMessage, '/tf', lambda msg: messages['tf'].extend(msg.transforms), 100))
        def record_diagnostics(msg):
            messages['diag'].extend(msg.status)
            diagnostics.extend((time.monotonic() - started, status) for status in msg.status)

        handles.append(node.create_subscription(
            DiagnosticArray, '/diagnostics', record_diagnostics, 100))
        publishers = {
            'camera': node.create_publisher(Odometry, '/zed/zed_node/odom', qos_profile_sensor_data),
            'wheel': node.create_publisher(Odometry, '/diff_drive_controller/odom', qos_profile_sensor_data),
            'zed': node.create_publisher(Imu, '/zed/zed_node/imu/data', qos_profile_sensor_data),
            'icm': node.create_publisher(Imu, '/imu/data_raw', qos_profile_sensor_data),
            'fix': node.create_publisher(NavSatFix, '/gps/fix', 10),
        }
        absent = set()
        if profile == 'missing_wheel':
            absent = {'wheel'}
        elif profile == 'zed_only':
            absent = {'wheel', 'zed', 'icm'}
        for key in absent:
            node.destroy_publisher(publishers.pop(key))

        def active(key, elapsed):
            if key in absent or (profile == 'zed_only' and elapsed < 2.0):
                return False
            if profile == 'dropouts':
                if 20.0 <= elapsed < 23.0:
                    return False
                pause = {'wheel': (5., 8.), 'camera': (8., 11.),
                         'zed': (11., 14.), 'icm': (14., 17.), 'fix': (17., 20.)}[key]
                if pause[0] <= elapsed < pause[1]:
                    return False
            return True
        # Only synthetic internal camera/IMU TF here. The live driver bridge
        # supplies this from SDK calibration; never fabricate it on hardware.
        broadcaster = StaticTransformBroadcaster(node)
        mount = TransformStamped()
        mount.header.frame_id = 'zed_camera_link'
        mount.child_frame_id = 'zed_imu_link'
        mount.transform.rotation.w = 1.0
        broadcaster.sendTransform(mount)
        started = time.monotonic()
        start_stamp = node.get_clock().now().nanoseconds * 1e-9
        iteration = 0
        checked_waiting = False
        try:
            while time.monotonic() - started < duration:
                elapsed = time.monotonic() - started
                if profile == 'zed_only' and elapsed >= 1.5 and not checked_waiting:
                    self.assertEqual(messages['local'], [], 'local EKF should wait for a valid measurement')
                    self.assertEqual(messages['global'], [], 'global EKF should wait for a valid measurement')
                    checked_waiting = True
                distance = max(0.0, elapsed - 6.0) * .2
                velocity = .2 if elapsed > 6.0 else 0.0
                stamp = node.get_clock().now().to_msg()
                camera = Odometry()
                camera.header.stamp = stamp
                camera.header.frame_id = 'odom'
                camera.child_frame_id = 'zed_camera_link'
                camera.pose.pose.position.x = distance + .3429
                camera.pose.pose.orientation.w = 1.0
                camera.pose.covariance = [.02 if i % 7 == 0 else 0.0 for i in range(36)]
                if active('camera', elapsed):
                    publishers['camera'].publish(camera)
                wheel = Odometry()
                wheel.header.stamp = stamp
                wheel.header.frame_id = 'odom'
                wheel.child_frame_id = 'base_link'
                wheel.pose.pose.orientation.w = 1.0
                wheel.twist.twist.linear.x = velocity
                wheel.twist.covariance[0] = .02
                wheel.twist.covariance[35] = .01
                if profile == 'dropouts':
                    # Simulate a publisher disappearing and reappearing, not
                    # only an advertised topic becoming silent.
                    if 5.0 <= elapsed < 8.0 and 'wheel' in publishers:
                        node.destroy_publisher(publishers.pop('wheel'))
                    elif elapsed >= 8.0 and 'wheel' not in publishers:
                        publishers['wheel'] = node.create_publisher(
                            Odometry, '/diff_drive_controller/odom', qos_profile_sensor_data)
                if active('wheel', elapsed):
                    publishers['wheel'].publish(wheel)
                for key, frame in (('zed', 'zed_imu_link'), ('icm', 'imu_link')):
                    imu = Imu()
                    imu.header.stamp = stamp
                    imu.header.frame_id = frame
                    imu.orientation_covariance[0] = -1.0
                    imu.angular_velocity_covariance = [
                        .001 if i % 4 == 0 else 0.0 for i in range(9)]
                    if active(key, elapsed):
                        publishers[key].publish(imu)
                if iteration % 5 == 0 and active('fix', elapsed):
                    fix = NavSatFix()
                    fix.header.stamp = stamp
                    fix.header.frame_id = 'gps_link'
                    fix.status.status = 0
                    fix.status.service = 1
                    fix.latitude = 40.0
                    # Inject a 2 m eastward GPS step after 12 seconds.
                    east = distance + (2.0 if elapsed > 12.0 else 0.0)
                    fix.longitude = -76.0 + math.degrees(east / (6378137.0 * math.cos(math.radians(40))))
                    fix.altitude = 70.0
                    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                    fix.position_covariance = [.04, 0., 0., 0., .04, 0., 0., 0., 1.]
                    publishers['fix'].publish(fix)
                iteration += 1
                # Service all output streams, not one callback per input tick:
                # local/global odometry + TF exceed the input loop frequency.
                for _ in range(12):
                    rclpy.spin_once(node, timeout_sec=0.0)
                time.sleep(.025)

            for key in ('local', 'global', 'gps', 'filtered'):
                self.assertGreater(len(messages[key]), 15, f'{key} output missing')
            local, global_msg, gps = [messages[key][-1] for key in ('local', 'global', 'gps')]
            local_stamp = local.header.stamp.sec + local.header.stamp.nanosec * 1e-9
            self.assertLess(node.get_clock().now().nanoseconds * 1e-9 - local_stamp, .5,
                            'test subscriber fell behind the live output')
            self.assertEqual((local.header.frame_id, local.child_frame_id), ('odom', 'base_link'))
            self.assertEqual((global_msg.header.frame_id, global_msg.child_frame_id), ('map', 'base_link'))
            self.assertEqual(gps.header.frame_id, 'map')
            self.assertAlmostEqual(local.pose.pose.position.x, .2 * (duration - 6.0), delta=.35)
            self.assertAlmostEqual(local.twist.twist.linear.x, .2, delta=.05)
            self.assertGreater(global_msg.pose.pose.position.x - local.pose.pose.position.x, 1.0)
            jumps = [abs(b.pose.pose.position.x-a.pose.pose.position.x)
                     for a, b in zip(messages['local'], messages['local'][1:])]
            self.assertLess(max(jumps), .15, 'GPS discontinuity leaked into local odometry')
            pairs = {(msg.header.frame_id, msg.child_frame_id) for msg in messages['tf']}
            self.assertIn(('map', 'odom'), pairs)
            self.assertIn(('odom', 'base_link'), pairs)
            for key in ('camera', 'wheel', 'zed_imu', 'icm_imu'):
                reports = [s for s in messages['diag'] if s.name == 'dual_sensor_inputs/' + key]
                if profile == 'missing_wheel' and key == 'wheel':
                    self.assertTrue(any(s.level == DiagnosticStatus.WARN for s in reports))
                    self.assertTrue(all(int(next(v.value for v in s.values if v.key == 'accepted')) == 0
                                        for s in reports), 'missing wheels must not produce fake measurements')
                    continue
                if profile == 'zed_only' and key != 'camera':
                    self.assertTrue(any(s.message == 'disabled by configuration' for s in reports))
                    continue
                self.assertTrue(any(s.level == DiagnosticStatus.OK for s in reports),
                                f'{key} was never accepted: '
                                f'{[(s.name, s.message) for s in reports[-3:]]}')
            if profile == 'dropouts':
                windows = {'wheel': (5.8, 7.9), 'camera': (8.8, 10.9),
                           'zed_imu': (11.8, 13.9), 'icm_imu': (14.8, 16.9)}
                for key, (begin, end) in windows.items():
                    self.assertTrue(any(begin <= t <= end and
                                        s.name == 'dual_sensor_inputs/' + key and
                                        s.level == DiagnosticStatus.WARN for t, s in diagnostics),
                                    f'{key} dropout was not reported')
                    self.assertTrue(any(t > 24.0 and s.name == 'dual_sensor_inputs/' + key and
                                        s.level == DiagnosticStatus.OK for t, s in diagnostics),
                                    f'{key} did not recover automatically')
                self.assertTrue(any(19.0 <= t <= 22.9 and
                                    s.name == 'global_sensor_inputs/gps' and
                                    s.level == DiagnosticStatus.WARN for t, s in diagnostics))
                self.assertTrue(any(t > 24.0 and s.name == 'global_sensor_inputs/gps' and
                                    s.level == DiagnosticStatus.OK for t, s in diagnostics))
                self.assertTrue(any(21.0 <= t <= 22.9 and
                                    s.name == 'dual_sensor_inputs/availability' and
                                    s.level == DiagnosticStatus.ERROR for t, s in diagnostics),
                                'total raw input loss must be explicit in diagnostics')
                for key in ('local', 'global'):
                    predicted = [m for m in messages[key] if 21.0 < (
                        m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 - start_stamp) < 22.9]
                    self.assertGreater(len(predicted), 15, f'{key} stopped during complete input loss')
                    self.assertGreater(predicted[-1].pose.covariance[0],
                                       predicted[0].pose.covariance[0],
                                       f'{key} uncertainty should grow during prediction-only operation')
            for msg in (local, global_msg):
                self.assertTrue(all(math.isfinite(v) for v in msg.pose.covariance))
                self.assertGreater(msg.pose.covariance[0], 0)
            print(f'PASS dual EKF [{profile}]: local x={local.pose.pose.position.x:.3f}, '
                  f'global x={global_msg.pose.pose.position.x:.3f}; '
                  f'max local step={max(jumps):.4f} m', flush=True)
        finally:
            node.destroy_node()
            rclpy.shutdown()
