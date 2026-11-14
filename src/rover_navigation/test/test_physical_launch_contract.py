"""Contract tests for the physical rover's single Nav2 command stack."""
import ast
from pathlib import Path

import yaml


SRC = Path(__file__).resolve().parents[2]
NAVIGATION = SRC / 'rover_navigation'
BRINGUP_LAUNCH = SRC / 'rover_bringup/launch/rover.launch.py'
NAVIGATION_LAUNCH = NAVIGATION / 'launch/navigation_launch.py'
NAV2_PARAMS = SRC / 'rover_description/config/nav2_params.yaml'


def parsed(path):
    return ast.parse(path.read_text(encoding='utf-8'))


def calls_named(tree, name):
    return [node for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == name]


def string_keyword(call, name):
    value = next(keyword.value for keyword in call.keywords if keyword.arg == name)
    assert isinstance(value, ast.Constant) and isinstance(value.value, str)
    return value.value


def launch_argument_default(tree, argument):
    for call in calls_named(tree, 'DeclareLaunchArgument'):
        if (call.args and isinstance(call.args[0], ast.Constant)
                and call.args[0].value == argument):
            return string_keyword(call, 'default_value')
    raise AssertionError(f'Launch argument {argument!r} is missing')


def test_physical_launch_uses_one_nav2_stack_and_actual_parameter_file():
    navigation = parsed(NAVIGATION_LAUNCH)
    bringup = parsed(BRINGUP_LAUNCH)

    nav2_packages = [call for call in calls_named(navigation, 'FindPackageShare')
                     if call.args and isinstance(call.args[0], ast.Constant)
                     and call.args[0].value == 'nav2_bringup']
    assert len(nav2_packages) == 1
    assert len(calls_named(navigation, 'IncludeLaunchDescription')) == 1

    bringup_strings = {node.value for node in ast.walk(bringup)
                       if isinstance(node, ast.Constant) and isinstance(node.value, str)}
    assert not {'nav2_bringup', 'rover_navigation', 'rover_costmap'} & bringup_strings

    launch_source = NAVIGATION_LAUNCH.read_text(encoding='utf-8')
    assert "FindPackageShare('rover_description'), 'config', 'nav2_params.yaml'" in launch_source
    assert NAV2_PARAMS.is_file()


def test_physical_command_remap_is_direct_and_not_sign_inverted():
    navigation = parsed(NAVIGATION_LAUNCH)
    remaps = [(string_keyword(call, 'src'), string_keyword(call, 'dst'))
              for call in calls_named(navigation, 'SetRemap')]
    assert remaps == [('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')]

    assert not (NAVIGATION / 'rover_navigation/cmd_vel_relay.py').exists()
    assert not (NAVIGATION / 'rover_navigation/sim_wheel_odometry.py').exists()
    setup_source = (NAVIGATION / 'setup.py').read_text(encoding='utf-8')
    assert 'cmd_vel_relay' not in setup_source
    assert 'sim_wheel_odometry' not in setup_source


def test_launched_nav2_configuration_is_forward_only_except_bt_backup():
    config = yaml.safe_load(NAV2_PARAMS.read_text(encoding='utf-8'))
    planner = config['planner_server']['ros__parameters']['GridBased']
    controller = config['controller_server']['ros__parameters']['FollowPath']
    smoother = config['velocity_smoother']['ros__parameters']
    behaviors = config['behavior_server']['ros__parameters']['behavior_plugins']

    assert planner['motion_model_for_search'] == 'DUBIN'
    assert controller['min_vel_x'] == 0.0
    assert smoother['min_velocity'][0] == 0.0
    assert behaviors == ['spin', 'backup', 'wait']


def test_arrival_validation_matches_the_launched_goal_tolerance():
    navigation = parsed(NAVIGATION_LAUNCH)
    config = yaml.safe_load(NAV2_PARAMS.read_text(encoding='utf-8'))
    goal_tolerance = config['controller_server']['ros__parameters'][
        'goal_checker']['xy_goal_tolerance']

    assert float(launch_argument_default(
        navigation, 'final_position_tolerance')) == goal_tolerance
    assert int(launch_argument_default(navigation, 'final_position_samples')) >= 3
    assert float(launch_argument_default(
        navigation, 'final_position_sample_period')) > 0.0
