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
                      navigation_timeout=600.0,
                      final_position_tolerance=5.0,
                      final_position_samples=5,
                      final_position_sample_period=1.0)
    parameters.update(overrides)
    node.get_parameter = lambda key: SimpleNamespace(value=parameters[key])
    node.get_clock = Mock(return_value=SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=TimeMessage)))
    node.get_logger = Mock(return_value=Mock())
    node.converter = Mock()
    node.costmap = Mock()
    node.navigator = Mock()
    node.wait_for_navigation_active = Mock()
    node.verify_final_position = Mock(return_value=True)
    node.last_recovery_count = 0
    node.last_feedback_log_time = 0.0
    node.current_cmd = None
    node.current_cmd_time = None
    node.active_waypoint = client.Waypoint(3.0, 4.0, True)
    node.final_target = SimpleNamespace(x=3.0, y=4.0)
    node.active_goal_yaw = 0.0
    node.action_state = 'RUNNING'
    node.tf_buffer = Mock()
    node.tf_buffer.lookup_transform.side_effect = client.TransformException('No map TF')
    return node


def test_feedback_logs_each_new_recovery_once():
    node = configured_client()
    logger = node.get_logger.return_value
    feedback = SimpleNamespace(feedback=SimpleNamespace(
        distance_remaining=10.0, number_of_recoveries=1))

    node.feedback_callback(feedback)
    node.feedback_callback(feedback)

    logger.warning.assert_called_once()
    assert 'recovery #1' in logger.warning.call_args.args[0]


def test_feedback_distinguishes_tf_path_state_heading_command_and_recoveries():
    node = configured_client()
    node.final_target = SimpleNamespace(x=6.0, y=8.0)
    transform = SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=0.0, y=0.0),
        rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))
    node.tf_buffer.lookup_transform.side_effect = None
    node.tf_buffer.lookup_transform.return_value = transform
    command = SimpleNamespace(
        linear=SimpleNamespace(x=0.2), angular=SimpleNamespace(z=-0.1))

    with patch.object(client.time, 'monotonic', side_effect=[100.0, 100.0]):
        node.cmd_vel_callback(command)
        node.feedback_callback(SimpleNamespace(feedback=SimpleNamespace(
            distance_remaining=7.5, number_of_recoveries=2)))

    log = node.get_logger.return_value.info.call_args.args[0]
    assert 'state=RUNNING' in log
    assert 'tf_to_final_gps_target=10.00 m' in log
    assert 'tf_to_active_waypoint=5.00 m' in log
    assert 'nav2_path_to_active_waypoint=7.50 m (advisory)' in log
    assert 'heading_error=+0.0 deg' in log
    assert 'linear.x=+0.200 m/s' in log
    assert 'angular.z=-0.100 rad/s' in log
    assert 'recoveries=2' in log


def test_zero_path_feedback_does_not_report_success():
    node = configured_client()
    original_state = node.action_state
    with patch.object(client.time, 'monotonic', return_value=100.0):
        node.feedback_callback(SimpleNamespace(feedback=SimpleNamespace(
            distance_remaining=0.0, number_of_recoveries=0)))

    log = node.get_logger.return_value.info.call_args.args[0]
    assert 'state=RUNNING' in log
    assert 'nav2_path_to_active_waypoint=0.00 m (advisory)' in log
    assert 'succeeded' not in log.lower()
    assert node.action_state == original_state
    node.navigator.send_goal_async.assert_not_called()
    node.navigator.cancel_goal_async.assert_not_called()


@pytest.mark.parametrize('overrides', [dict(lat=float('nan')), dict(lon=181.0),
                         dict(heading=float('inf')), dict(max_goal_distance=0.0),
                         dict(waypoint_margin=0.0),
                         dict(navigation_timeout=float('nan')),
                         dict(final_position_tolerance=0.0),
                         dict(final_position_samples=2),
                         dict(final_position_sample_period=0.0)])
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
    assert [call.kwargs['timeout'] for call in node.wait_for_result.call_args_list] == [10., 10., 600.]
    node.verify_final_position.assert_called_once()


def test_final_position_requires_four_of_five_localization_samples():
    node = configured_client()
    node.verify_final_position = client.NavigationClient.verify_final_position.__get__(node)
    node.wait_for_localization_sample = Mock()
    target = SimpleNamespace(x=0.0, y=0.0)
    node.get_current_position = Mock(side_effect=[
        SimpleNamespace(x=0.0, y=0.0),
        SimpleNamespace(x=20.0, y=0.0),
        SimpleNamespace(x=20.0, y=0.0),
        SimpleNamespace(x=20.0, y=0.0),
        SimpleNamespace(x=20.0, y=0.0),
    ])

    assert not node.verify_final_position(target, 5.0, 5, 1.0)
    assert node.wait_for_localization_sample.call_count == 4


def test_final_position_accepts_one_noisy_outlier():
    node = configured_client()
    node.verify_final_position = client.NavigationClient.verify_final_position.__get__(node)
    node.wait_for_localization_sample = Mock()
    target = SimpleNamespace(x=0.0, y=0.0)
    node.get_current_position = Mock(side_effect=[
        SimpleNamespace(x=1.0, y=0.0),
        SimpleNamespace(x=2.0, y=0.0),
        SimpleNamespace(x=30.0, y=0.0),
        SimpleNamespace(x=3.0, y=0.0),
        SimpleNamespace(x=4.0, y=0.0),
    ])

    assert node.verify_final_position(target, 5.0, 5, 1.0)


def test_unstable_final_position_replans_instead_of_reporting_arrival():
    node = configured_client()
    final_waypoint = client.Waypoint(1.0, 2.0, True)
    node.next_waypoint = Mock(return_value=final_waypoint)
    node.navigate_waypoint = Mock()
    node.verify_final_position.side_effect = [False, True]
    node.wait_for_result = Mock(return_value=SimpleNamespace(
        map_point=SimpleNamespace(x=1.0, y=2.0)))

    node.navigate()

    assert node.navigate_waypoint.call_count == 2
    assert node.verify_final_position.call_count == 2
    warning = node.get_logger.return_value.warning
    assert 'not stable' in warning.call_args_list[0].args[0]


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
    with pytest.raises(RuntimeError, match='action state ABORTED'):
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
    params_path = source / 'rover_description/config/nav2_params.yaml'
    params = yaml.safe_load(params_path.read_text(encoding='utf-8'))
    launch = ast.parse((source / 'rover_navigation/launch/navigation_launch.py').read_text())
    distance = next(float(keyword.value.value)
                    for node in ast.walk(launch)
                    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                    and node.func.id == 'DeclareLaunchArgument' and node.args
                    and isinstance(node.args[0], ast.Constant)
                    and node.args[0].value == 'max_goal_distance'
                    for keyword in node.keywords if keyword.arg == 'default_value')
    global_params = params['global_costmap']['global_costmap']['ros__parameters']
    local_params = params['local_costmap']['local_costmap']['ros__parameters']
    metadata = costmap_response(
        width=global_params['width'], height=global_params['height'],
        resolution=global_params['resolution'],
        origin=(-global_params['width'] / 2, -global_params['height'] / 2)).map
    for heading in range(0, 360, 15):
        angle = math.radians(heading)
        waypoint = client.select_waypoint(SimpleNamespace(x=0., y=0.), SimpleNamespace(
            x=distance * math.cos(angle), y=distance * math.sin(angle)), metadata, distance, 1.)
        assert abs(waypoint.x) < global_params['width'] / 2
        assert abs(waypoint.y) < global_params['height'] / 2
        assert not waypoint.is_final
    assert global_params['rolling_window'] is True
    assert local_params['resolution'] == 0.1
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
