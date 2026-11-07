"""Exercise noisy asynchronous inputs, real sensor transforms, outages and recovery."""
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


@launch_testing.parametrize('profile,seed', [('noisy',197), ('noisy',211), ('tracking_fault',197)])
def generate_test_description(profile, seed):
    script = str(Path(__file__).resolve().with_name('noisy_position_fixture.py'))
    probe = ExecuteProcess(cmd=[sys.executable, '-B', script, 'probe'], output='screen')
    sensors = ExecuteProcess(cmd=[sys.executable, '-B', script, 'sensors', profile, str(seed)], output='screen')
    domain = 199+(seed != 197)+(2 if profile == 'tracking_fault' else 0)
    local = str(Path(get_package_share_directory('rover_localization'))/'launch/localization.launch.py')
    return LaunchDescription([
        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value=str(domain)),
        SetEnvironmentVariable(name='ROS_LOCALHOST_ONLY', value='1'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(local),
                                 launch_arguments={'fused_imu_topic':'/imu/fused'}.items()),
        sensors, probe, launch_testing.actions.ReadyToTest(),
    ]), {'probe':probe}


class TestNoisyPipeline(unittest.TestCase):
    def test_probe_finishes(self, proc_info, probe):
        proc_info.assertWaitForShutdown(process=probe, timeout=35)


@launch_testing.post_shutdown_test()
class TestNoisyExit(unittest.TestCase):
    def test_fixture_expectations(self, proc_info, probe):
        launch_testing.asserts.assertExitCodes(proc_info, process=probe)
