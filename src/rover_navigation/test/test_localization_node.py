"""ROS node lifecycle checks for the physical localization readiness gate."""
from unittest.mock import Mock, patch

import rclpy
from nav_msgs.msg import Odometry

from rover_navigation import wait_for_localization as readiness


def test_readiness_receives_valid_and_removes_invalid_source():
    rclpy.init()
    node = readiness.LocalizationReadiness()
    try:
        message = Odometry()
        message.pose.pose.orientation.w = 1.0
        node.receive('local', message)
        assert 'local' in node.received
        assert not node.ready()
        message.pose.pose.position.x = float('nan')
        node.receive('local', message)
        assert 'local' not in node.received
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_readiness_main_success():
    node = Mock()
    node.get_parameter.return_value.value = 1.0
    node.ready.return_value = True
    with patch.object(readiness.rclpy, 'init'), \
            patch.object(readiness.rclpy, 'try_shutdown'), \
            patch.object(readiness.rclpy, 'spin_once'), \
            patch.object(readiness.rclpy, 'ok', return_value=True), \
            patch.object(readiness, 'LocalizationReadiness', return_value=node):
        readiness.main()
    node.destroy_node.assert_called_once()
