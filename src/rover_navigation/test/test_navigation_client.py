"""ROS client failure handling without a running ROS graph."""
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from rover_navigation import navigation_client as client


def bare_client():
    return client.NavigationClient.__new__(client.NavigationClient)


def test_wait_timeout_cancels_pending_future():
    node = bare_client()
    future = Future()
    with patch.object(client.rclpy, 'spin_until_future_complete'):
        with pytest.raises(TimeoutError):
            node.wait_for_result(future, timeout=0.01)
    assert future.cancelled()


def test_abort_waits_for_cancellation_acknowledgement():
    node = bare_client()
    future = Future()
    future.set_result(SimpleNamespace(goals_canceling=[object()]))
    node.goal_handle = Mock()
    node.goal_handle.cancel_goal_async.return_value = future
    with patch.object(client.rclpy, 'spin_until_future_complete') as spin:
        node.abort_action()
    spin.assert_called_once()
    assert spin.call_args.kwargs['timeout_sec'] > 0


def test_main_propagates_navigation_failure_as_nonzero_exit():
    node = Mock()
    node.navigate.side_effect = RuntimeError('goal failed')
    with patch.object(client.rclpy, 'init'), patch.object(client.rclpy, 'try_shutdown'), \
            patch.object(client, 'NavigationClient', return_value=node):
        with pytest.raises(SystemExit) as error:
            client.main()
    assert error.value.code != 0
    node.destroy_node.assert_called_once()


def test_main_cancels_on_navigation_failure():
    node = Mock()
    node.navigate.side_effect = TimeoutError('navigation timed out')
    with patch.object(client.rclpy, 'init'), patch.object(client.rclpy, 'try_shutdown'), \
            patch.object(client, 'NavigationClient', return_value=node):
        with pytest.raises(SystemExit):
            client.main()
    node.abort_action.assert_called_once()


def configured_client(**overrides):
    node = bare_client()
    parameters = dict(lat=39.0, lon=-76.0, heading=90.0,
                      max_goal_distance=70.0, navigation_timeout=180.0)
    parameters.update(overrides)
    node.get_parameter = lambda key: SimpleNamespace(value=parameters[key])
    node.get_clock = Mock(return_value=SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None)))
    node.get_logger = Mock(return_value=Mock())
    node.converter = Mock()
    node.navigator = Mock()
    node.wait_for_navigation_active = Mock()
    return node


@pytest.mark.parametrize('overrides', [dict(lat=float('nan')), dict(lon=181.0),
                         dict(heading=float('inf')), dict(max_goal_distance=0.0),
                         dict(navigation_timeout=float('nan'))])
def test_invalid_parameters_never_send_goal(overrides):
    node = configured_client(**overrides)
    with pytest.raises(ValueError):
        node.navigate()
    node.navigator.send_goal_async.assert_not_called()


def test_valid_goal_uses_map_yaw_and_bounded_action_waits():
    node = configured_client()
    handle = Mock(accepted=True)
    node.wait_for_result = Mock(side_effect=[
        SimpleNamespace(map_point=SimpleNamespace(x=1., y=2.)), handle,
        SimpleNamespace(status=client.GoalStatus.STATUS_SUCCEEDED)])
    with patch.object(client.NavigateToPose, 'Goal') as goal_type:
        node.navigate()
    goal = goal_type.return_value
    assert goal.pose.header.frame_id == 'map'
    assert goal.pose.pose.orientation.z == pytest.approx(2**-0.5)
    assert goal.pose.pose.orientation.w == pytest.approx(2**-0.5)
    assert [call.kwargs['timeout'] for call in node.wait_for_result.call_args_list] == [10., 10., 180.]


@pytest.mark.parametrize('point', [(float('nan'), 0.), (71., 0.)])
def test_invalid_converted_goal_never_sent(point):
    node = configured_client()
    node.wait_for_result = Mock(return_value=SimpleNamespace(map_point=SimpleNamespace(x=point[0], y=point[1])))
    with pytest.raises(RuntimeError):
        node.navigate()
    node.navigator.send_goal_async.assert_not_called()


def test_pending_goal_acceptance_future_preserved_on_timeout():
    node = bare_client()
    future = Future()
    with patch.object(client.rclpy, 'spin_until_future_complete'):
        with pytest.raises(TimeoutError):
            node.wait_for_result(future, timeout=.01, cancel_on_timeout=False)
    assert not future.cancelled()


def test_late_goal_acceptance_is_resolved_then_cancelled():
    node = bare_client()
    handle = Mock(accepted=True)
    node.pending_goal = Future()
    node.pending_goal.set_result(handle)
    cancellation = Future()
    cancellation.set_result(SimpleNamespace(goals_canceling=[object()]))
    handle.cancel_goal_async.return_value = cancellation
    with patch.object(client.rclpy, 'spin_until_future_complete'):
        node.abort_action()
    handle.cancel_goal_async.assert_called_once()


def test_navigation_readiness_waits_for_lifecycle_activation():
    node = Mock()
    service = node.create_client.return_value
    def response(state):
        future = Future()
        future.set_result(SimpleNamespace(current_state=SimpleNamespace(id=state)))
        return future
    service.call_async.side_effect = [response(2), response(3), response(3)]
    with patch.object(client.rclpy, 'spin_until_future_complete'), \
            patch.object(client.rclpy, 'spin_once'):
        client.wait_for_navigation_active(node, timeout=1.)
    assert service.call_async.call_count == 3
    assert node.destroy_client.call_count == 2


def test_navigation_readiness_has_bounded_missing_service_failure():
    node = Mock()
    node.create_client.return_value.wait_for_service.return_value = False
    with pytest.raises(TimeoutError, match='Nav2'):
        client.wait_for_navigation_active(node, timeout=.01)
    node.destroy_client.assert_called_once()
