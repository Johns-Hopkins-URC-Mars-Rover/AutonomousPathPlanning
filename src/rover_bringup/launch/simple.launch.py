"""Single-command yes/no selection for rover sensors and localization."""
import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

spec = importlib.util.spec_from_file_location('simple_switches', Path(__file__).with_name('simple_switches.py'))
switches = importlib.util.module_from_spec(spec)
spec.loader.exec_module(switches)


def start(context):
    names = ('zed', 'icm', 'gps', 'wheel', 'localization', 'drivers')
    values = {name: context.launch_configurations[name] for name in names}
    arguments = switches.launch_arguments(values)
    arguments.update({name: context.launch_configurations[name] for name in (
        'camera_model', 'heading_mode', 'initial_heading_deg', 'heading_topic',
        'publish_approximate_mounts', 'use_sim_time')})
    launch_file = PathJoinSubstitution([FindPackageShare('rover_bringup'), 'launch', 'rover.launch.py'])
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file),
                                     launch_arguments=arguments.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('zed', default_value='yes', description='ZED driver and optical odometry/gyro: yes/no'),
        DeclareLaunchArgument('icm', default_value='no', description='ICM driver and gyro input: yes/no'),
        DeclareLaunchArgument('gps', default_value='no', description='GPS driver and global localization: yes/no'),
        DeclareLaunchArgument('wheel', default_value='no', description='Use existing wheel odometry topic: yes/no'),
        DeclareLaunchArgument('localization', default_value='yes', description='Run localization EKF: yes/no'),
        DeclareLaunchArgument('drivers', default_value='yes', description='Launch selected hardware drivers: yes/no'),
        DeclareLaunchArgument('camera_model', default_value='zed2i'),
        DeclareLaunchArgument('heading_mode', default_value='unset'),
        DeclareLaunchArgument('initial_heading_deg', default_value='nan'),
        DeclareLaunchArgument('heading_topic', default_value='/imu/heading'),
        DeclareLaunchArgument('publish_approximate_mounts', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=start),
    ])
