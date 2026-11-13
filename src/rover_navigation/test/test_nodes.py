"""Small node lifecycle and relay checks using real ROS node construction."""
from unittest.mock import Mock, patch

import pytest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from rover_navigation import cmd_vel_relay as relay
from rover_navigation import wait_for_localization as readiness


def test_relay_forwards_same_twist():
    rclpy.init()
    node = relay.CmdVelRelay()
    try:
        message = Twist()
        message.linear.x = .2
        with patch.object(node.publisher, 'publish') as publish:
            node.relay(message)
        publish.assert_called_once_with(message)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_readiness_receives_valid_and_removes_invalid_source():
    rclpy.init()
    node = readiness.LocalizationReadiness()
    try:
        message = Odometry()
        message.pose.pose.orientation.w = 1.
        node.receive('local', message)
        assert 'local' in node.received
        assert not node.ready()
        message.pose.pose.position.x = float('nan')
        node.receive('local', message)
        assert 'local' not in node.received
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_relay_main_shuts_down_after_keyboard_interrupt():
    node = Mock()
    with patch.object(relay.rclpy,'init'), patch.object(relay.rclpy,'try_shutdown'), \
            patch.object(relay.rclpy,'spin',side_effect=KeyboardInterrupt), \
            patch.object(relay,'CmdVelRelay',return_value=node):
        relay.main()
    node.destroy_node.assert_called_once()


def test_readiness_main_success():
    node = Mock()
    node.get_parameter.return_value.value = 1.
    node.ready.return_value = True
    with patch.object(readiness.rclpy,'init'), patch.object(readiness.rclpy,'try_shutdown'), \
            patch.object(readiness.rclpy,'spin_once'), patch.object(readiness.rclpy,'ok',return_value=True), \
            patch.object(readiness,'LocalizationReadiness',return_value=node):
        readiness.main()
    node.destroy_node.assert_called_once()
