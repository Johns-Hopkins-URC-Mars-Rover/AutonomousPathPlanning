"""ROS client failure handling without a running ROS graph."""
from concurrent.futures import Future
import ast
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
import yaml
from builtin_interfaces.msg import Time as TimeMessage

from rover_navigation import navigation_client as client


def bare_client():
    return client.NavigationClient.__new__(client.NavigationClient)


def costmap_response(width=10, height=10, resolution=0.05, origin=(-5., -5.)):
    response = client.GetCostmap.Response()
    response.map.header.frame_id = 'map'
    metadata = response.map.metadata
    metadata.resolution = resolution
    metadata.size_x = round(width / resolution)
    metadata.size_y = round(height / resolution)
    metadata.origin.position.x, metadata.origin.position.y = origin
    metadata.origin.orientation.w = 1.0
    response.map.data = [0] * (metadata.size_x * metadata.size_y)
    return response


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
                      max_goal_distance=70.0, waypoint_margin=1.0,
                      navigation_timeout=600.0, arrival_tolerance=1.0)
    parameters.update(overrides)
    node.get_parameter = lambda key: SimpleNamespace(value=parameters[key])
    node.get_clock = Mock(return_value=SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=TimeMessage)))
    node.get_logger = Mock(return_value=Mock())
    node.converter = Mock()
    node.costmap = Mock()
    node.navigator = Mock()
    node.wait_for_navigation_active = Mock()
    node.wait_for_guard = Mock()
    node.assert_guard_ready = Mock()
    node.confirm_arrival = Mock()
    node.get_current_position = Mock(return_value=SimpleNamespace(x=0., y=0.))
    return node


@pytest.mark.parametrize('overrides', [dict(lat=float('nan')), dict(lon=181.0),
                         dict(heading=float('inf')), dict(max_goal_distance=0.0),
                         dict(waypoint_margin=0.0),
                         dict(navigation_timeout=float('nan'))])
def test_invalid_parameters_never_send_goal(overrides):
    node = configured_client(**overrides)
    with pytest.raises(ValueError):
        node.navigate()
    node.navigator.send_goal_async.assert_not_called()


def test_valid_goal_uses_map_yaw_and_bounded_action_waits():
    node = configured_client()
    node.next_waypoint = Mock(return_value=client.Waypoint(1., 2., True))
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
    waits = [call.kwargs['timeout'] for call in node.wait_for_result.call_args_list]
    assert waits[:2] == [10., 10.]
    assert 0 < waits[2] <= 600.
    node.confirm_arrival.assert_called_once()


@pytest.mark.parametrize('target', [(15.65, 43.69), (100., 0.)])
def test_far_gps_goal_advances_with_rolling_costmap_until_final_goal(target):
    node = configured_client()
    position = SimpleNamespace(x=0., y=0.)
    node.get_current_position = Mock(side_effect=lambda **kwargs: position)
    sent, origins = [], []

    def completed(value):
        future = Future()
        future.set_result(value)
        return future

    node.converter.call_async.side_effect = lambda request: completed(SimpleNamespace(
        map_point=SimpleNamespace(x=target[0], y=target[1])))

    def current_map(request):
        origin = (position.x - 5., position.y - 5.)
        origins.append(origin)
        return completed(costmap_response(origin=origin))

    node.costmap.call_async.side_effect = current_map

    def navigate(goal, **kwargs):
        endpoint = goal.pose.pose.position
        assert abs(endpoint.x - position.x) <= 4.000001
        assert abs(endpoint.y - position.y) <= 4.000001
        sent.append(goal)
        handle = Mock(accepted=True)

        def arrive():
            position.x, position.y = endpoint.x, endpoint.y
            return completed(SimpleNamespace(status=client.GoalStatus.STATUS_SUCCEEDED))

        handle.get_result_async.side_effect = arrive
        return completed(handle)

    node.navigator.send_goal_async.side_effect = navigate
    with patch.object(client.rclpy, 'spin_until_future_complete'):
        node.navigate()

    assert len(sent) > 1
    assert len(origins) == len(sent)
    assert origins[0] != origins[-1]
    assert (position.x, position.y) == pytest.approx(target)
    final_orientation = sent[-1].pose.pose.orientation
    assert final_orientation.z == pytest.approx(2**-0.5)
    assert final_orientation.w == pytest.approx(2**-0.5)
    first_orientation = sent[0].pose.pose.orientation
    first_yaw = 2 * math.atan2(first_orientation.z, first_orientation.w)
    assert first_yaw == pytest.approx(math.atan2(target[1], target[0]))


def test_missing_costmap_service_never_sends_goal():
    node = configured_client()
    node.costmap.wait_for_service.return_value = False
    node.wait_for_result = Mock(return_value=SimpleNamespace(
        map_point=SimpleNamespace(x=15.65, y=43.69)))
    with pytest.raises(RuntimeError, match='/global_costmap/get_costmap'):
        node.navigate()
    node.costmap.wait_for_service.assert_called_once_with(timeout_sec=10.0)
    node.navigator.send_goal_async.assert_not_called()


def test_reported_goal_selects_intermediate_goal_inside_small_window():
    waypoint = client.select_waypoint(
        SimpleNamespace(x=0., y=0.), SimpleNamespace(x=15.65, y=43.69),
        costmap_response().map, 70., 1.)
    assert waypoint.y == pytest.approx(4., abs=1e-6)
    assert waypoint.x == pytest.approx(4. * 15.65 / 43.69, abs=1e-6)
    assert not waypoint.is_final


@pytest.mark.parametrize('target', [(5., 0.), (-5., 0.), (0., 5.), (500., -500.)])
def test_outside_and_boundary_targets_create_interior_waypoints(target):
    waypoint = client.select_waypoint(
        SimpleNamespace(x=0., y=0.), SimpleNamespace(x=target[0], y=target[1]),
        costmap_response().map, 70., 1.)
    assert -4.000001 <= waypoint.x <= 4.000001
    assert -4.000001 <= waypoint.y <= 4.000001
    assert not waypoint.is_final


def test_rolling_costmap_uses_live_origin():
    grid = costmap_response(origin=(100., 200.)).map
    waypoint = client.select_waypoint(
        SimpleNamespace(x=105., y=205.), SimpleNamespace(x=150., y=250.), grid, 70., 1.)
    assert waypoint.x == pytest.approx(109., abs=1e-6)
    assert waypoint.y == pytest.approx(209., abs=1e-6)
    assert not waypoint.is_final


def test_near_target_is_the_final_waypoint():
    waypoint = client.select_waypoint(
        SimpleNamespace(x=0., y=0.), SimpleNamespace(x=1., y=2.),
        costmap_response().map, 70., 1.)
    assert waypoint == client.Waypoint(1., 2., True)


def test_maximum_goal_distance_caps_each_waypoint_not_total_route():
    waypoint = client.select_waypoint(
        SimpleNamespace(x=0., y=0.), SimpleNamespace(x=1000., y=0.),
        costmap_response(width=160, height=160, resolution=0.2, origin=(-80., -80.)).map,
        70., 1.)
    assert waypoint == client.Waypoint(70., 0., False)


def test_waits_for_costmap_to_recenter_after_rover_moves():
    node = configured_client()
    node.get_current_position = Mock(return_value=SimpleNamespace(x=10., y=0.))
    node.wait_for_result = Mock(side_effect=[
        costmap_response(), costmap_response(origin=(5., -5.))])
    with patch.object(client.rclpy, 'spin_once'):
        waypoint = node.next_waypoint(SimpleNamespace(x=100., y=0.), 70., 1.)
    assert waypoint.x == pytest.approx(14., abs=1e-6)
    assert waypoint.y == pytest.approx(0.)
    assert not waypoint.is_final
    assert node.costmap.call_async.call_count == 2


def test_failed_intermediate_waypoint_stops_navigation():
    node = configured_client()
    node.next_waypoint = Mock(return_value=client.Waypoint(4., 0., False))
    node.wait_for_result = Mock(side_effect=[
        SimpleNamespace(map_point=SimpleNamespace(x=100., y=0.)),
        Mock(accepted=True), SimpleNamespace(status=client.GoalStatus.STATUS_ABORTED)])
    with pytest.raises(RuntimeError, match='action status'):
        node.navigate()
    node.next_waypoint.assert_called_once()
    node.navigator.send_goal_async.assert_called_once()


def test_new_waypoint_acceptance_timeout_keeps_new_goal_cancelable():
    node = configured_client()
    node.goal_handle = Mock(accepted=True)
    pending = Future()
    node.navigator.send_goal_async.return_value = pending
    node.wait_for_result = Mock(side_effect=TimeoutError('acceptance timed out'))
    with pytest.raises(TimeoutError):
        node.navigate_waypoint(client.Waypoint(4., 0., False), 0., 180.)
    assert node.goal_handle is None
    assert node.pending_goal is pending


def test_missing_rover_transform_has_bounded_failure():
    node = configured_client()
    node.get_current_position = client.NavigationClient.get_current_position.__get__(node)
    node.tf_buffer = Mock()
    node.tf_buffer.lookup_transform.side_effect = client.TransformException('No map TF')
    with patch.object(client.rclpy, 'spin_once'):
        with pytest.raises(TimeoutError, match='rover pose'):
            node.get_current_position(timeout=.01)


def test_wrong_costmap_frame_is_rejected():
    grid = costmap_response().map
    grid.header.frame_id = 'odom'
    with pytest.raises(RuntimeError, match='Expected the global costmap in map'):
        client.select_waypoint(SimpleNamespace(x=0., y=0.),
                               SimpleNamespace(x=1., y=2.), grid, 70., 1.)


@pytest.mark.parametrize('field,value', [('resolution', 0.), ('size_x', 0)])
def test_uninitialized_costmap_is_rejected(field, value):
    grid = costmap_response().map
    setattr(grid.metadata, field, value)
    with pytest.raises(RuntimeError, match='invalid or uninitialized bounds'):
        client.select_waypoint(SimpleNamespace(x=0., y=0.),
                               SimpleNamespace(x=1., y=2.), grid, 70., 1.)


def test_default_rolling_map_supports_waypoints_at_all_target_bearings():
    source = Path(__file__).resolve().parents[2]
    params = yaml.safe_load((source / 'rover_description/config/nav2_params.yaml').read_text())
    distance = 100.0
    global_params = params['global_costmap']['global_costmap']['ros__parameters']
    local_params = params['local_costmap']['local_costmap']['ros__parameters']
    metadata = costmap_response(
        width=global_params['width'], height=global_params['height'],
        resolution=global_params['resolution'],
        origin=(-global_params['width'] / 2, -global_params['height'] / 2)).map
    for heading in range(0, 360, 15):
        angle = math.radians(heading)
        waypoint = client.select_waypoint(SimpleNamespace(x=0., y=0.), SimpleNamespace(
            x=distance * math.cos(angle), y=distance * math.sin(angle)), metadata, 10.0, 1.)
        assert abs(waypoint.x) < global_params['width'] / 2
        assert abs(waypoint.y) < global_params['height'] / 2
        assert not waypoint.is_final
    assert global_params['rolling_window'] is True
    assert local_params['resolution'] == 0.05
    assert global_params['width'] / global_params['resolution'] <= 1000
    assert global_params['height'] / global_params['resolution'] <= 1000


@pytest.mark.parametrize('point', [(float('nan'), 0.), (float('inf'), 0.)])
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
    service.call_async.side_effect = [response(2), response(3), response(3), response(3)]
    with patch.object(client.rclpy, 'spin_until_future_complete'), \
            patch.object(client.rclpy, 'spin_once'):
        client.wait_for_navigation_active(node, timeout=1.)
    assert service.call_async.call_count == 4
    assert node.destroy_client.call_count == 3


def test_navigation_readiness_has_bounded_missing_service_failure():
    node = Mock()
    node.create_client.return_value.wait_for_service.return_value = False
    with pytest.raises(TimeoutError, match='Nav2'):
        client.wait_for_navigation_active(node, timeout=.01)
    node.destroy_client.assert_called_once()


def test_occupied_final_target_is_not_sent():
    grid = costmap_response().map
    grid.data[100 * grid.metadata.size_x + 120] = 254  # (1.0, 0.0)
    with pytest.raises(RuntimeError, match='Final GPS target footprint is occupied'):
        client.select_waypoint(SimpleNamespace(x=0., y=0.),
                               SimpleNamespace(x=1., y=0.), grid, 10., 1.)


def test_projected_intermediate_target_is_shortened_before_obstacle():
    grid = costmap_response().map
    grid.data[100 * grid.metadata.size_x + 175] = 254
    waypoint = client.select_waypoint(SimpleNamespace(x=0., y=0.),
                                     SimpleNamespace(x=20., y=0.), grid, 10., 1.)
    assert 1.5 < waypoint.x < 3.
    assert not waypoint.is_final


def test_zero_path_feedback_does_not_report_arrival():
    node = configured_client()
    node.final_target = SimpleNamespace(x=10., y=0.)
    node.tf_buffer = Mock()
    node.tf_buffer.lookup_transform.return_value.transform.translation = SimpleNamespace(x=0., y=0.)
    node.feedback_callback(SimpleNamespace(feedback=SimpleNamespace(distance_remaining=0.)))
    text = node.get_logger().info.call_args.args[0]
    assert '10.0 m' in text and 'awaiting action result' in text
    node.confirm_arrival.assert_not_called()


def test_success_is_not_reported_if_arrival_confirmation_fails():
    node = configured_client()
    node.next_waypoint = Mock(return_value=client.Waypoint(1., 2., True))
    node.navigate_waypoint = Mock()
    node.wait_for_result = Mock(return_value=SimpleNamespace(map_point=SimpleNamespace(x=1., y=2.)))
    node.confirm_arrival.side_effect = RuntimeError('localization jumped')
    with pytest.raises(RuntimeError, match='localization jumped'):
        node.navigate()
    assert not any('Navigation succeeded' in call.args[0]
                   for call in node.get_logger().info.call_args_list)
