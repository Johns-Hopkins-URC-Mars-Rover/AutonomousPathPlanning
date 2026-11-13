"""Small node lifecycle and relay checks using real ROS node construction."""
from unittest.mock import Mock, patch

import pytest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from rover_navigation import cmd_vel_relay as relay
from rover_navigation import wait_for_localization as readiness
from rover_navigation import sim_wheel_odometry


def test_simulated_wheel_odometry_converts_twist_and_covariance():
    rclpy.init()
    node = sim_wheel_odometry.SimWheelOdometry()
    try:
        message = Odometry()
        message.twist.twist.linear.x = .2
        message.twist.twist.angular.z = .3
        message.twist.covariance[1] = .04
        with patch.object(node.publisher, 'publish') as publish:
            node.receive(message)
        output = publish.call_args.args[0]
        assert output.twist.twist.linear.x == pytest.approx(-.2)
        assert output.twist.twist.angular.z == pytest.approx(-.3)
        assert output.twist.covariance[1] == pytest.approx(-.04)
        assert message.twist.twist.linear.x == pytest.approx(.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_relay_corrects_simulation_drive_direction():
    rclpy.init()
    node = relay.CmdVelRelay()
    try:
        message = Twist()
        message.linear.x = .2
        message.angular.z = .3
        with patch.object(node.publisher, 'publish') as publish:
            node.relay(message)
        output = publish.call_args.args[0]
        assert output.linear.x == pytest.approx(-.2)
        assert output.angular.z == pytest.approx(-.3)
        assert message.linear.x == pytest.approx(.2)
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
