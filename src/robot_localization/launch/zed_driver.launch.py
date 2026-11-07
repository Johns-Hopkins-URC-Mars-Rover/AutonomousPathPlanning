"""ZED optical driver and calibrated internal IMU TF; no estimator/motion TF."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_model', default_value='zed2i'),
        DeclareLaunchArgument('calibration_topic', default_value='/zed/zed_node/left_cam_imu_transform'),
        DeclareLaunchArgument('imu_frame', default_value='zed_imu_link'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'])),
            launch_arguments={
                'camera_model': LaunchConfiguration('camera_model'),
                'ros_params_override_path': PathJoinSubstitution([
                    FindPackageShare('robot_localization'), 'params', 'zed_optical.yaml']),
                'publish_tf': 'false', 'publish_map_tf': 'false',
                'publish_imu_tf': 'false', 'enable_gnss': 'false',
            }.items()),
        Node(package='robot_localization', executable='zed_imu_tf.py',
             name='zed_imu_calibration_tf', output='screen', parameters=[{
                 'calibration_topic': LaunchConfiguration('calibration_topic'),
                 'imu_frame': LaunchConfiguration('imu_frame'),
             }]),
    ])
