"""Real ROS graph test: synthetic sensors -> validation -> EKF -> 3D probe.

Runs on localhost in a test domain, not the live rover's domain. No camera,
serial driver or GPS receiver is started. Set ROVER_TEST_DOMAIN_ID for parallel
workspaces; CTest runs this fixture serially within its workspace.
"""
import os
from pathlib import Path
import sys
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.asserts


def generate_test_description():
    domain = int(os.environ.get('ROVER_TEST_DOMAIN_ID', '197'))
    if not 0 <= domain <= 232:
        raise ValueError('ROVER_TEST_DOMAIN_ID must be between 0 and 232')
    probe_path = Path(__file__).resolve().parents[1] / 'scripts/probe_simulation.py'
    probe = ExecuteProcess(cmd=[sys.executable, '-B', str(probe_path), '--timeout-sec', '25'],
                           output='screen')
    return LaunchDescription([
        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=str(domain)),
        SetEnvironmentVariable(name='ROS_LOCALHOST_ONLY', value='1'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('rover_localization')) /
            'launch/sim_localization.launch.py'))),
        probe,
        launch_testing.actions.ReadyToTest(),
    ]), {'probe': probe}


class TestPipeline(unittest.TestCase):
    def test_probe_finishes(self, proc_info, probe):
        proc_info.assertWaitForShutdown(process=probe, timeout=35)


@launch_testing.post_shutdown_test()
class TestPipelineExit(unittest.TestCase):
    def test_probe_passed(self, proc_info, probe):
        launch_testing.asserts.assertExitCodes(proc_info, process=probe)
