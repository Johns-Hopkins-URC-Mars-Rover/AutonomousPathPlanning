require 'minitest/autorun'
require 'yaml'

ROOT = File.expand_path('..', __dir__)

class LocalizationContractTest < Minitest::Test
  def yaml_file(name)
    YAML.safe_load(File.read(File.join(ROOT, 'config', name)))
  end

  def test_local_filter_consumes_optical_odometry_and_one_joint_imu
    params = yaml_file('ekf_local.yaml').fetch('ekf_local').fetch('ros__parameters')

    assert_equal '/zed/zed_node/odom', params.fetch('odom0')
    assert_equal '/imu/fused', params.fetch('imu0')
    refute params.key?('odom1'), 'wheel odometry must not be enabled before hardware exists'
    assert_equal 'odom', params.fetch('world_frame')
    assert_equal 'base_link', params.fetch('base_link_frame')
    assert_equal true, params.fetch('publish_tf')
    assert_equal false, params.fetch('two_d_mode')

    odom_fields = params.fetch('odom0_config')
    imu_fields = params.fetch('imu0_config')
    assert_equal 15, odom_fields.size
    assert_equal 15, imu_fields.size
    assert_equal [0, 1, 2, 3, 4, 5], odom_fields.each_index.select { |i| odom_fields[i] }
    assert_equal [9, 10, 11], imu_fields.each_index.select { |i| imu_fields[i] }
  end

  def test_zed_settings_keep_raw_imu_but_disable_internal_fusion_and_motion_tf
    params = yaml_file('zed_optical.yaml').fetch('/**').fetch('ros__parameters')
    tracking = params.fetch('pos_tracking')
    sensors = params.fetch('sensors')

    assert_equal true, tracking.fetch('pos_tracking_enabled')
    assert_equal false, tracking.fetch('imu_fusion')
    assert_equal false, tracking.fetch('publish_tf')
    assert_equal false, tracking.fetch('publish_map_tf')
    assert_equal true, sensors.fetch('publish_imu_raw')
  end

  def test_package_installs_launch_and_config_and_declares_filter_dependency
    cmake = File.read(File.join(ROOT, 'CMakeLists.txt'))
    package = File.read(File.join(ROOT, 'package.xml'))

    assert_match(/install\(DIRECTORY\s+config\s+launch/m, cmake)
    assert_match(/<exec_depend>robot_localization<\/exec_depend>/, package)
    assert File.file?(File.join(ROOT, 'launch', 'localization.launch.py'))
    assert File.file?(File.join(ROOT, 'launch', 'sim_localization.launch.py'))
    assert File.file?(File.join(ROOT, 'launch', 'zed_localization.launch.py'))
  end

  def test_real_zed_fake_imu_bench_launch_uses_one_fake_fused_imu_stream
    cmake = File.read(File.join(ROOT, 'CMakeLists.txt'))
    launch = File.read(File.join(ROOT, 'launch', 'zed_fake_imu_localization.launch.py'))

    assert_match(/install\(PROGRAMS\s+scripts\/fake_imu_publisher\.py/m, cmake)
    assert_match(/zed_localization\.launch\.py/, launch)
    assert_match(/executable='fake_imu_publisher\.py'/, launch)
    assert_match(/default_value='\/imu\/fused'/, launch)
  end

  def test_private_simulation_header_has_no_duplicate_package_directory
    assert File.file?(File.join(ROOT, 'include', 'simulation_model.hpp'))
    refute Dir.exist?(File.join(ROOT, 'include', 'rover_localization'))
  end

  def test_simulated_sensor_source_has_one_complete_node_definition
    source = File.read(File.join(ROOT, 'src', 'simulated_sensors.cpp'))

    assert_equal 1, source.scan(/^\s*class SimulatedSensors\b/).size
    assert_match(/timer_ = create_wall_timer\(period, std::bind\(&SimulatedSensors::update, this\)\)/, source)
  end

  def test_smoke_script_bounds_ros_launch_shutdown
    script = File.read(File.join(ROOT, 'scripts', 'smoke_test.sh'))

    assert_match(/shutdown_deadline=/, script)
    assert_match(/kill -INT "\$launch_pid"/, script)
    assert_match(/kill -TERM "\$launch_pid"/, script)
    assert_match(/kill -KILL "\$launch_pid"/, script)
  end

  def test_synthetic_run_explains_its_data_contract_in_terminal_logs
    script = File.read(File.join(ROOT, 'scripts', 'smoke_test.sh'))
    source = File.read(File.join(ROOT, 'src', 'simulated_sensors.cpp'))

    assert_match(/\[smoke 1\/4\] Building rover_localization/, script)
    assert_match(/\[smoke 2\/4\] Starting synthetic data and EKF/, script)
    assert_match(/\[smoke 3\/4\] Probing EKF output/, script)
    assert_match(/\[smoke 4\/4\] PASS/, script)
    assert_match(/Fake ZED optical odometry/, source)
    assert_match(/Fake joint IMU gyro/, source)
    assert_match(/Comparison-only ground truth/, source)
  end
end
