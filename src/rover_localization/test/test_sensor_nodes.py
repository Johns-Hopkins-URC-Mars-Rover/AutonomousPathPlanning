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
    OK, ERROR = 0, 2


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
        self.now_s = 10.1
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
        return self.publisher_count
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


def load(name):
    mapping = {
        'rclpy': dict(init=lambda: None, spin=lambda node: None, try_shutdown=lambda: None),
        'rclpy.node': dict(Node=FakeNode),
        'rclpy.qos': dict(qos_profile_sensor_data=1),
        'rclpy.time': dict(Time=lambda: None),
        'nav_msgs.msg': dict(Odometry=Odom),
        'sensor_msgs.msg': dict(Imu=Imu, NavSatFix=Fix),
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
        spec = importlib.util.spec_from_file_location(name, ROOT / (name+'.py'))
        result = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(result)
    return result


class SensorNodesTest(unittest.TestCase):
    def setUp(self):
        FakeNode.overrides = {}
        self.module = load('sensor_inputs')
        self.node = self.module.SensorInputs()

    def test_camera_and_imu_callbacks_publish_then_report_health(self):
        self.node.callbacks['/zed/zed_node/odom'](Odom())
        self.node.callbacks['/imu/data'](Imu())
        self.assertEqual(len(self.node.messages['/localization/zed_odom_base']), 1)
        self.node.report()
        self.assertTrue(all(status.level == 0 for status in self.node.messages['/diagnostics'][-1].status))
        self.node.now_s = 12.
        self.node.report()
        self.assertTrue(all(status.level == 2 for status in self.node.messages['/diagnostics'][-1].status))

    def test_rejects_duplicate_publisher_bad_frame_and_repeated_stamp(self):
        callback = self.node.callbacks['/zed/zed_node/odom']
        self.node.publisher_count = 2
        callback(Odom())
        self.node.publisher_count = 1
        msg = Odom(); msg.header.frame_id = 'map'; callback(msg)
        msg.header.frame_id = ''; callback(msg)
        msg = Odom(); msg.child_frame_id = ''; callback(msg)
        callback(Odom()); callback(Odom())
        self.assertEqual(self.node.states['camera']['accepted'], 1)
        self.assertEqual(self.node.states['camera']['rejected'], 5)
        self.node.report()
        self.assertEqual(self.node.messages['/diagnostics'][-1].status[0].level, 2)

    def test_real_camera_mount_and_missing_imu_tf(self):
        msg = Odom(); msg.child_frame_id = 'zed_camera_link'
        self.assertEqual(self.node.camera(msg).child_frame_id, 'base_link')
        msg = Imu(); msg.header.frame_id = 'zed_imu_link'
        self.node.buffer.missing = True
        self.node.callbacks['/imu/data'](msg)
        self.assertEqual(self.node.states['imu']['rejected'], 1)
        self.node.buffer.missing = False
        self.assertIs(self.node.imu(msg), msg)
        msg.angular_velocity.x = math.nan
        with self.assertRaises(ValueError): self.node.imu(msg)

    def test_gps_and_heading_validity(self):
        FakeNode.overrides = {'mode': 'gps'}
        node = self.module.SensorInputs()
        node.callbacks['/gps/fix'](Fix())
        node.callbacks['/imu/heading'](Imu())
        self.assertEqual(node.states['gps']['accepted'], 1)
        msg = Imu(); msg.header.frame_id = 'heading_sensor'
        self.assertIs(node.heading(msg), msg)
        fix = Fix(); fix.position_covariance_type = 0
        with self.assertRaises(ValueError): node.gps(fix)
        fix = Fix(); fix.header.frame_id = 'base_link'
        self.assertIs(node.gps(fix), fix)

    def test_invalid_configuration_fails(self):
        for overrides in ({'mode': 'bad'}, {'max_age': 0.}, {'zed_odom_topic': '/localization/zed_odom_base'}):
            FakeNode.overrides = overrides
            with self.assertRaises(ValueError): self.module.SensorInputs()

    def test_main_closes_cleanly(self):
        self.module.main()

    def test_calibration_bridge_uses_source_transform_once(self):
        mod = load('zed_imu_tf')
        node = mod.CalibrationTf()
        node.check()
        bad = Transform(); bad.transform.rotation.w = 0.
        node.receive(bad)
        bad = Transform(); bad.child_frame_id = 'wrong'
        node.receive(bad)
        self.assertFalse(node.sent)
        good = Transform(); good.transform.translation.x = .02
        node.receive(good); node.receive(good); node.check()
        self.assertEqual(node.broadcaster.messages, [good])
        mod.main()


if __name__ == '__main__':
    unittest.main()
