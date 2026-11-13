"""Reject corrupt/stale localization before Nav2 activation."""
from nav_msgs.msg import Odometry
import pytest

from rover_navigation.wait_for_localization import valid_odometry


def valid_message():
    message = Odometry()
    message.pose.pose.orientation.w = 1.0
    return message


def test_finite_normalized_odometry_is_valid():
    assert valid_odometry(valid_message())


@pytest.mark.parametrize('corruption', ['position', 'quaternion', 'twist', 'pose_cov', 'twist_cov'])
def test_corrupt_odometry_is_rejected(corruption):
    message = valid_message()
    if corruption == 'position':
        message.pose.pose.position.x = float('nan')
    elif corruption == 'quaternion':
        message.pose.pose.orientation.w = 0.0
    elif corruption == 'twist':
        message.twist.twist.angular.z = float('inf')
    elif corruption == 'pose_cov':
        message.pose.covariance[0] = -1.0
    else:
        message.twist.covariance[35] = float('nan')
    assert not valid_odometry(message)


def gate():
    import time
    from types import SimpleNamespace as NS
    from unittest.mock import Mock
    from rover_navigation.wait_for_localization import LocalizationReadiness
    node = LocalizationReadiness.__new__(LocalizationReadiness)
    node.previous_clock = 4_000_000_000
    node.clock_advanced = False
    node.clock_updated_at = time.monotonic()
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=5_000_000_000))
    node.get_parameter = lambda name: NS(value=1.)
    node.received = {name: (time.monotonic(), NS(sec=5,nanosec=0)) for name in ('local','global','gps')}
    node.buffer = Mock()
    node.buffer.can_transform.return_value = True
    node.buffer.lookup_transform.return_value = NS(header=NS(stamp=NS(sec=5,nanosec=0)))
    return node


def test_fresh_valid_localization_with_advancing_clock_is_ready():
    assert gate().ready()


def test_stale_source_stamp_is_not_ready():
    from types import SimpleNamespace as NS
    node = gate()
    received, _ = node.received['local']
    node.received['local'] = (received, NS(sec=1,nanosec=0))
    assert not node.ready()


def test_stale_transform_is_not_ready():
    from types import SimpleNamespace as NS
    node = gate()
    node.buffer.lookup_transform.return_value = NS(header=NS(stamp=NS(sec=1,nanosec=0)))
    assert not node.ready()


def test_nonadvancing_clock_is_not_ready():
    node = gate()
    node.previous_clock = 5_000_000_000
    assert not node.ready()


def test_without_gps_odometry_is_not_ready():
    node = gate()
    del node.received['gps']
    assert not node.ready()
