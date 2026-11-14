"""The hardware navigation policy must never regain blind recovery motion."""
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

DESCRIPTION = Path(__file__).resolve().parents[2] / 'rover_description'


@pytest.mark.parametrize('filename', ['navigate_to_pose.xml', 'navigate_through_poses.xml'])
def test_recovery_is_bounded_and_stationary(filename):
    root = ET.parse(DESCRIPTION / 'behavior_trees' / filename).getroot()
    for tag in ('Spin', 'BackUp', 'DriveOnHeading', 'ClearEntireCostmap', 'ClearCostmapExceptRegion'):
        assert root.find('.//' + tag) is None
    recovery = root.find('.//RecoveryNode')
    assert recovery.get('number_of_retries') == '2'
    assert root.find('.//Wait').get('wait_duration') == '3.0'
    assert root.find('.//FollowPath').get('goal_checker_id') == 'goal_checker'


def test_velocity_pipeline_and_server_limits_agree():
    cfg = yaml.safe_load((DESCRIPTION / 'config/nav2_params.yaml').read_text())
    controller = cfg['controller_server']['ros__parameters']['FollowPath']
    smoother = cfg['velocity_smoother']['ros__parameters']
    monitor = cfg['collision_monitor']['ros__parameters']
    guard = cfg['velocity_guard']['ros__parameters']
    assert controller['vx_min'] == smoother['min_velocity'][0] == 0.
    assert controller['vx_max'] == smoother['max_velocity'][0] == guard['max_linear']
    assert controller['wz_max'] == smoother['max_velocity'][2] == guard['max_angular']
    assert monitor['cmd_vel_in_topic'] == '/cmd_vel_filtered'
    assert monitor['cmd_vel_out_topic'] == guard['input_topic']
    assert guard['output_topic'] == '/diff_drive_controller/cmd_vel_unstamped'
    assert cfg['behavior_server']['ros__parameters']['behavior_plugins'] == ['wait']
    assert guard['sensor_timeout'] < 1.0
