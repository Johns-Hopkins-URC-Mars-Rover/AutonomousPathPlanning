"""Run real sensor callbacks against message/transport doubles (no ROS graph)."""
import copy
import importlib.util
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1] / 'scripts'


def stamp():
    return NS(sec=10, nanosec=0)


def header(frame='base_link'):
    return NS(stamp=stamp(), frame_id=frame)


def vec():
    return NS(x=0., y=0., z=0.)


def quat():
    return NS(x=0., y=0., z=0., w=1.)


class Imu:
    def __init__(self):
        self.header = header()
        self.angular_velocity, self.linear_acceleration = vec(), vec()
        self.orientation = quat()
        self.angular_velocity_covariance = [1.,0.,0.,0.,1.,0.,0.,0.,1.]
        self.orientation_covariance = self.angular_velocity_covariance[:]
        self.linear_acceleration_covariance = [0.] * 9


class Twist:
    pass


class Odom:
    def __init__(self):
        self.header = header('odom')
        self.child_frame_id = 'base_link'
        self.pose = NS(pose=NS(position=vec(), orientation=quat()),
                       covariance=[float(r == c) for r in range(6) for c in range(6)])
        self.twist = Twist()


class Fix:
    def __init__(self):
        self.header = header('gps_link')
        self.status = NS(status=0)
        self.latitude, self.longitude, self.altitude = 39., -76., 20.
        self.position_covariance_type = 2
        self.position_covariance = [4.,0.,0.,0.,4.,0.,0.,0.,9.]


class DiagnosticStatus:
    OK, WARN, ERROR = 0, 1, 2


class DiagnosticArray:
    def __init__(self):
        self.header, self.status = header(), []


class Transform:
    def __init__(self):
        self.header = header('zed_left_camera_frame')
        self.child_frame_id = 'zed_imu_link'
        self.transform = NS(translation=vec(), rotation=quat())


class MissingTransform(Exception):
    pass


class Buffer:
    def __init__(self, **kw):
        self.missing = False
    def lookup_transform(self, *args):
        if self.missing:
            raise MissingTransform('mount missing')
        return Transform()


class FakeNode:
    overrides = {}
    def __init__(self, name):
        self.name, self.messages, self.callbacks, self.logs = name, {}, {}, []
        self.publisher_count = 1
        self.publisher_counts = {}
        self.now_s = 10.1
    @property
    def timers(self):
        return ()  # rclpy exposes this as a read-only property.
    def declare_parameter(self, name, default):
        return NS(value=self.overrides.get(name, default))
    def create_publisher(self, kind, topic, qos):
        self.messages[topic] = []
        return NS(publish=lambda msg: self.messages[topic].append(copy.deepcopy(msg)))
    def create_subscription(self, kind, topic, callback, qos):
        self.callbacks[topic] = callback
        return callback
    def create_timer(self, *args):
        return NS()
    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=int(self.now_s*1e9), to_msg=stamp))
    def count_publishers(self, topic):
        return self.publisher_counts.get(topic, self.publisher_count)
    def get_logger(self):
        return NS(warning=lambda msg, **kw: self.logs.append(msg),
                  error=lambda msg, **kw: self.logs.append(msg),
                  info=lambda msg, **kw: self.logs.append(msg))
    def get_name(self):
        return self.name
    def destroy_node(self):
        pass


class Broadcaster:
    def __init__(self, node):
        self.messages = []
    def sendTransform(self, msg):
        self.messages.append(msg)


def load(name, folder=ROOT):
    mapping = {
        'rclpy': dict(init=lambda: None, spin=lambda node: None, try_shutdown=lambda: None),
        'rclpy.node': dict(Node=FakeNode),
        'rclpy.qos': dict(qos_profile_sensor_data=1),
        'rclpy.time': dict(Time=lambda: None),
        'nav_msgs.msg': dict(Odometry=Odom),
        'sensor_msgs.msg': dict(Imu=Imu, NavSatFix=Fix),
        'std_msgs.msg': dict(String=lambda: NS(data='')),
        'geometry_msgs.msg': dict(TransformStamped=Transform),
        'diagnostic_msgs.msg': dict(DiagnosticArray=DiagnosticArray,
                                    DiagnosticStatus=DiagnosticStatus, KeyValue=lambda **kw: NS(**kw)),
        'tf2_ros': dict(Buffer=Buffer, TransformException=MissingTransform,
                        TransformListener=lambda *args: None, StaticTransformBroadcaster=Broadcaster),
    }
    modules = {}
    for key, contents in mapping.items():
        modules[key] = ModuleType(key)
        modules[key].__dict__.update(contents)
    spec = importlib.util.spec_from_file_location('sensor_contracts', ROOT / 'sensor_contracts.py')
    contracts = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(contracts)
    modules['sensor_contracts'] = contracts
    with patch.dict(sys.modules, modules):
        spec = importlib.util.spec_from_file_location(name, folder / (name+'.py'))
        result = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(result)
    return result



class RoverInputsTest(unittest.TestCase):
    def test_shutdown_conversion_error_only_ignored_when_context_stopped(self):
        mod = load('sensor_inputs')
        failure = RuntimeError('Unable to convert call argument to Python object (compile in debug mode for details)')
        with patch.object(mod.rclpy, 'spin', side_effect=failure), \
             patch.object(mod.rclpy, 'ok', return_value=False, create=True):
            mod.main()
        with patch.object(mod.rclpy, 'spin', side_effect=failure), \
             patch.object(mod.rclpy, 'ok', return_value=True, create=True):
            with self.assertRaises(RuntimeError):
                mod.main()
    def setUp(self):
        FakeNode.overrides = {}
        self.module = load('sensor_inputs')
        self.node = self.module.SensorInputs()

    @staticmethod
    def wheel():
        msg = Odom()
        msg.twist.twist = NS(linear=vec(), angular=vec())
        msg.twist.covariance = [0.] * 36
        msg.twist.covariance[0] = .04
        msg.twist.covariance[35] = .01
        return msg

    def test_four_direct_inputs_publish_and_report_healthy(self):
        self.node.callbacks['/zed/zed_node/odom'](Odom())
        self.node.callbacks['/diff_drive_controller/odom'](self.wheel())
        self.node.callbacks['/zed/zed_node/imu/data'](Imu())
        self.node.callbacks['/imu/data_raw'](Imu())
        self.assertNotIn('/imu/data', self.node.callbacks)
        self.node.report()
        self.assertEqual(len(self.node.messages['/diagnostics'][-1].status), 5)
        self.assertTrue(all(s.level == 0 for s in self.node.messages['/diagnostics'][-1].status))
        self.node.now_s = 12.
        self.node.report()
        self.assertTrue(all(s.level == DiagnosticStatus.WARN
                            for s in self.node.messages['/diagnostics'][-1].status[:-1]))
        self.assertEqual(self.node.messages['/diagnostics'][-1].status[-1].level,
                         DiagnosticStatus.ERROR)

    def test_missing_wheels_leave_camera_and_both_imus_independent(self):
        self.node.publisher_counts['/diff_drive_controller/odom'] = 0
        self.node.callbacks['/zed/zed_node/odom'](Odom())
        self.node.callbacks['/zed/zed_node/imu/data'](Imu())
        self.node.callbacks['/imu/data_raw'](Imu())
        self.node.report()
        reports = {s.name.rsplit('/', 1)[-1]: s
                   for s in self.node.messages['/diagnostics'][-1].status}
        self.assertEqual(reports['wheel'].level, DiagnosticStatus.WARN)
        self.assertEqual(reports['availability'].level, DiagnosticStatus.WARN)
        for key in ('camera', 'zed_imu', 'icm_imu'):
            self.assertEqual(reports[key].level, DiagnosticStatus.OK)
            self.assertEqual(self.node.states[key]['accepted'], 1)
        self.assertEqual(self.node.messages['/localization/wheel_odom'], [])

    def test_explicitly_disabled_input_has_no_subscription_or_fake_output(self):
        FakeNode.overrides = {'use_wheel_odom': False}
        node = self.module.SensorInputs()
        self.assertNotIn('/diff_drive_controller/odom', node.callbacks)
        self.assertNotIn('/localization/wheel_odom', node.messages)
        node.report()
        wheel = next(s for s in node.messages['/diagnostics'][-1].status
                     if s.name.endswith('/wheel'))
        self.assertEqual(wheel.level, DiagnosticStatus.OK)
        self.assertEqual(wheel.message, 'disabled by configuration')

    def test_input_recovers_after_publisher_returns(self):
        self.node.publisher_counts['/diff_drive_controller/odom'] = 0
        self.node.report()
        self.node.publisher_counts['/diff_drive_controller/odom'] = 1
        self.node.callbacks['/diff_drive_controller/odom'](self.wheel())
        self.node.report()
        wheel = next(s for s in self.node.messages['/diagnostics'][-1].status
                     if s.name.endswith('/wheel'))
        self.assertEqual(wheel.level, DiagnosticStatus.OK)
        self.assertEqual(self.node.states['wheel']['accepted'], 1)

    def test_wheel_uses_selected_covariance_and_rejects_invalid_velocity(self):
        msg = self.wheel()
        self.assertIs(self.node.wheel(msg), msg)
        msg.twist.covariance[35] = 0.
        with self.assertRaises(ValueError):
            self.node.wheel(msg)
        msg = self.wheel()
        msg.twist.twist.linear.x = math.nan
        with self.assertRaises(ValueError):
            self.node.wheel(msg)
        msg = self.wheel()
        msg.child_frame_id = ''
        with self.assertRaises(ValueError):
            self.node.wheel(msg)

    def test_callbacks_reject_duplicates_and_stale_samples(self):
        callback = self.node.callbacks['/imu/data_raw']
        self.node.publisher_count = 2
        callback(Imu())
        self.node.publisher_count = 1
        callback(Imu())
        callback(Imu())
        self.node.now_s = 12.
        callback(Imu())
        self.assertEqual(self.node.states['icm_imu']['accepted'], 1)
        self.assertEqual(self.node.states['icm_imu']['rejected'], 3)

    def test_clock_rewind_allows_new_epoch_but_still_rejects_duplicate_samples(self):
        callback = self.node.callbacks['/imu/data_raw']
        callback(Imu())
        self.node.now_s = 2.1
        message = Imu()
        message.header.stamp.sec = 2
        callback(message)
        self.assertEqual(self.node.states['icm_imu']['accepted'], 2)
        callback(message)
        self.assertEqual(self.node.states['icm_imu']['accepted'], 2)
        self.assertEqual(self.node.states['icm_imu']['rejected'], 1)

    def test_gps_requires_known_covariance_and_heading(self):
        FakeNode.overrides = {'mode': 'gps'}
        node = self.module.SensorInputs()
        node.callbacks['/gps/fix'](Fix())
        node.callbacks['/imu/heading'](Imu())
        self.assertEqual(node.states['gps']['accepted'], 1)
        msg = Fix()
        msg.position_covariance_type = 0
        with self.assertRaises(ValueError):
            node.gps(msg)

    def test_calibration_bridge_uses_sdk_transform_once(self):
        mod = load('zed_imu_tf')
        node = mod.CalibrationTf()
        bad = Transform()
        bad.child_frame_id = 'wrong'
        node.receive(bad)
        self.assertFalse(node.sent)
        good = Transform()
        good.transform.translation.x = .02
        node.receive(good)
        node.receive(good)
        self.assertEqual(node.broadcaster.messages, [good])

    def test_startup_heading_publishes_enu_without_fusing_orientation(self):
        FakeNode.overrides = {'initial_heading_deg': 90.}
        mod = load('startup_heading')
        node = mod.StartupHeading()
        node.receive(Odom())
        out = node.messages['/localization/startup_heading'][0]
        self.assertAlmostEqual(out.orientation.z, 0.)
        self.assertAlmostEqual(out.orientation.w, 1.)
        self.assertEqual(out.header.frame_id, 'base_link')
        self.assertGreater(out.orientation_covariance[8], 0.)
        self.assertEqual(out.angular_velocity_covariance[0], -1.)


if __name__ == '__main__':
    unittest.main()
