"""Critical stop behavior, independent of the motor hardware and ROS graph."""
from types import SimpleNamespace as NS
from unittest.mock import Mock
import time

import pytest
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header

from rover_navigation.velocity_guard import (
    VelocityGuard, bounded_command, finite_cloud, pose_jump,
)


def test_command_conventions_are_preserved_and_bounded():
    cmd = Twist(); cmd.linear.x = .2; cmd.angular.z = -.3
    result = bounded_command(cmd, .25, .35)
    assert result.linear.x == .2 and result.angular.z == -.3
    cmd.linear.x = .8; cmd.angular.z = .9
    result = bounded_command(cmd, .25, .35)
    assert result.linear.x == .25 and result.angular.z == .35


@pytest.mark.parametrize('field,value', [('linear.x', -.1), ('linear.y', .1),
                                        ('angular.z', float('nan'))])
def test_unusable_commands_are_rejected(field, value):
    cmd = Twist(); parent, key = field.split('.')
    setattr(getattr(cmd, parent), key, value)
    with pytest.raises(ValueError): bounded_command(cmd, .25, .35)


def test_yaw_wrap_is_not_a_localization_jump():
    assert not pose_jump((0, 0, 3.13), (.01, 0, -3.13), .05, 1., .7)
    assert pose_jump((0, 0, 0), (3, 0, 0), .05, 1., .7)
    assert pose_jump((0, 0, 0), (0, 0, 2), .05, 1., .7)


def test_invalid_sky_does_not_hide_valid_depth():
    points = [(float('inf'),)*3]*3000 + [(1., 0., .1)]
    assert finite_cloud(create_cloud_xyz32(Header(), points))
    assert not finite_cloud(create_cloud_xyz32(Header(), points[:-1]))
    assert not finite_cloud(create_cloud_xyz32(Header(), []))


def guard():
    node = VelocityGuard.__new__(VelocityGuard)
    node.limits = dict(command_timeout=.3, sensor_timeout=.75, odometry_timeout=.75,
                       gps_timeout=2., transform_timeout=.75, startup_hold=2.,
                       jump_distance=1., jump_angle=.7, max_linear=.25, max_angular=.35)
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000))
    node.received = {key: (time.monotonic(), 10.) for key in ('cloud','imu','local','global','gps')}
    node.source_frames = dict(cloud='camera', imu='base_link')
    node.buffer = Mock()
    node.buffer.lookup_transform.return_value = NS(header=NS(stamp=NS(sec=10, nanosec=0)))
    node.buffer.can_transform.return_value = True
    node.output = Mock(); node.ready_pub = Mock(); node.status_pub = Mock()
    node.get_logger = Mock(return_value=Mock())
    node.command = Twist(); node.command.linear.x = .2
    node.command_received = time.monotonic(); node.command_error = ''
    node.healthy_since = time.monotonic()-3.; node.armed = True
    node.latched_fault = ''; node.last_reason = None; node.last_global = None
    return node


def test_ready_guard_forwards_positive_commands_without_inversion():
    node = guard(); node.tick()
    assert node.output.publish.call_args.args[0].linear.x == .2
    assert node.ready_pub.publish.call_args.args[0].data is True


@pytest.mark.parametrize('source', ['cloud', 'imu', 'local', 'global', 'gps'])
def test_stale_input_stops_motion_and_withdraws_readiness(source):
    node = guard(); node.received[source] = (time.monotonic()-3., 10.)
    node.tick()
    assert node.output.publish.call_args.args[0].linear.x == 0.
    assert node.ready_pub.publish.call_args.args[0].data is False


def test_stale_transform_stops_motion():
    node = guard()
    node.buffer.lookup_transform.return_value.header.stamp.sec = 8
    node.tick()
    assert node.output.publish.call_args.args[0].linear.x == 0.
    assert 'stale transform' in node.status_pub.publish.call_args.args[0].data


def test_lost_command_stops_even_if_localization_is_fresh():
    node = guard(); node.command_received = time.monotonic()-1.
    node.tick()
    assert node.output.publish.call_args.args[0].linear.x == 0.


def test_global_jump_latches_stop_even_after_subsequent_good_messages():
    node = guard(); node.last_global = (9.95, (0., 0., 0.))
    msg = Odometry(); msg.header.frame_id='map';msg.header.stamp.sec=10
    msg.pose.pose.orientation.w=1.;msg.pose.pose.position.x=3.
    node.receive_odometry('global',msg)
    assert 'localization jumped' in node.latched_fault
    node.receive_odometry('global',msg)
    node.tick()
    assert node.output.publish.call_args.args[0].linear.x == 0.
    assert node.ready_pub.publish.call_args.args[0].data is False


def test_unknown_gps_uncertainty_is_not_accepted():
    node = guard()
    msg = Odometry(); msg.header.frame_id = 'map'; msg.header.stamp.sec = 10
    msg.pose.pose.orientation.w = 1.
    node.receive_odometry('gps', msg)
    assert 'gps' not in node.received
    msg.pose.covariance[0] = .5; msg.pose.covariance[7] = .5
    node.receive_odometry('gps', msg)
    assert 'gps' in node.received
