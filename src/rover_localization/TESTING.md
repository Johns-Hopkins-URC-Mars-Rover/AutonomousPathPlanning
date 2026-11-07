# Localization + L76X GPS testing

The Jetson owns the ZED/L76X drivers. The ROS machine owns the local EKF,
optional global EKF, RViz and recordings. Use ROS 2 Humble, synchronized UTC
system clocks, and `ROS_DOMAIN_ID=35` in every terminal on both machines.
Run exactly one driver per sensor and one local EKF. Stop old test launches,
including the temporary identity TF publishers from the earlier bench session.

## Build and source

Synchronize the edited source to both machines, preserving the installed ZED
wrapper and SDK on the Jetson. Do not copy build/install/log between machines.
On each machine, from its repository root:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=35
colcon build --symlink-install --packages-select rover_localization rover_description rover_imu_fusion rover_bringup l76x_gps_interface
source install/setup.bash
```

Source ROS and that machine's `install/setup.bash` in each new terminal.
The ROS machine needs `robot_localization`, RViz, TF and diagnostic messages;
only the Jetson needs ZED SDK/CUDA, wrapper, Jetson.GPIO and GPS serial access.

## Current bench test: real ZED + its IMU, GPS recorded

You have no verified rover mounts or global heading yet. Define the test body
as the camera frame itself (`zed_camera_link`) for this handheld test. This
avoids inventing a rover mount. The ZED's internal camera/IMU calibration is
still required: it is not an identity transform.

Jetson terminal A:

```bash
ros2 launch rover_bringup rover.launch.py camera_model:=zed2i enable_imu:=false enable_fusion:=false enable_gps:=true
```

This launches L76X, the optical-only ZED, and a bridge from the ZED SDK's
`/zed/zed_node/left_cam_imu_transform` to static TF. The wrapper keeps its camera
URDF on; it must not publish odom/map motion TF or a second IMU TF. Our bridge
uses the calibration message's actual parent/child, and never invents numbers.
If its topic/frame differs, launch `zed_driver.launch.py` directly with the
actual `calibration_topic` and `imu_frame`, plus L76X separately.

ROS-machine terminal B:

```bash
ros2 run tf2_ros tf2_echo zed_camera_link zed_imu_link
```

Allow discovery, verify a transform appears, then Ctrl-C. If missing, inspect
`left_cam_imu_transform`, camera URDF frames and Jetson bridge warnings. Do not
substitute a zero transform. Verify effective wrapper parameters:

```bash
for p in pos_tracking.imu_fusion pos_tracking.publish_tf sensors.publish_cam_imu_transf; do
  echo "$p"
  ros2 param get /zed/zed_node "$p"
done
```

Expected: false, false, true. `publish_map_tf` is not declared in every wrapper;
inspect the actual TF graph rather than treating an undeclared parameter as a
successful override. Installed wrapper versions must support the calibration
message option; verify with `ros2 launch zed_wrapper zed_camera.launch.py --show-args`.

ROS-machine terminal B, after the TF check:

```bash
ros2 launch rover_description gps_nav_launch.py base_frame:=zed_camera_link fused_imu_topic:=/zed/zed_node/imu/data
```

This starts one local estimator. GPS fusion and Nav2 remain disabled by default.
The validated camera and gyro outputs are `/localization/zed_odom_base` and
`/localization/imu`; the EKF publishes `/odometry/local` and
`odom -> zed_camera_link` for this bench configuration.

ROS-machine terminal C, before moving:

```bash
ros2 bag record -o localization_bench /zed/zed_node/odom /zed/zed_node/imu/data /zed/zed_node/left_cam_imu_transform /zed/zed_node/pose/status /zed/zed_node/odom/status /localization/zed_odom_base /localization/imu /odometry/local /gps/fix /diagnostics /tf /tf_static
```

Tracking-status topic names/types vary by wrapper version; inspect `ros2 topic list -t` and record the available status topics (install matching zed_msgs on the recording machine). Missing optional status topics should not be mistaken for missing standard odometry. Use a new output directory for each run. Ctrl-C closes the recording cleanly.
Capture effective parameter dumps and package versions with the session notes:

```bash
ros2 param dump /zed/zed_node
ros2 param dump /ekf_local
ros2 topic info /zed/zed_node/odom -v
ros2 topic info /zed/zed_node/imu/data -v
```

An additional sourced ROS terminal can inspect:

```bash
ros2 topic echo /diagnostics
ros2 topic echo /odometry/local --field pose.pose.position
```

Ctrl-C each streaming inspection before the next command. For RViz, run
`rviz2`, set Fixed Frame `odom`, add TF, and add Odometry `/odometry/local`.

## Measured motion and pass/fail evidence

1. Hold still for 20 seconds. Record drift and input ages/accept/reject counts.
2. Move along a marked straight line by a measured distance (e.g. 0.30 or 1 m),
   keeping orientation steady. Stop 10 seconds; return to the start mark.
3. Slowly rotate through a measured 90 degrees and stop; gyro must return near
   zero after stopping. Repeat rather than relying on one terminal message.
4. Compare changes between timestamp-matched start/end poses, not distance from
   an arbitrary odom origin. Compare raw ZED, converted pose and EKF together.
5. Record 3D displacement, distance error, return error, angular error, rates,
   rejected measurements, tracking status and any camera dropout.

Set numerical tolerances before the run based on the intended rover requirement
and reference measurement. A published estimate or green diagnostic only
establishes availability/contracts, not accuracy. The L76X is an additional
geographic reference, not established ground truth for a 15 cm bench movement.
If raw ZED is already wrong, investigate tracking/depth/scene quality first. If
raw/converted poses are reasonable but EKF diverges, inspect accepted camera
updates, covariance and time alignment. Do not hide a vertical failure by
forcing 2D or arbitrarily tightening covariances.

The local EKF can continue prediction after input loss; `/diagnostics` must be
checked for stale accepted input even if `/odometry/local` still arrives at 30 Hz.
The separate IMU-fusion node suppresses output after all usable IMUs time out.

## L76X contract and later GPS fusion

Raw GPS is `/gps/fix`, in WGS84 latitude/longitude degrees and ellipsoid altitude
meters. L76X enables RMC + GGA and pairs matching UTC epochs. Ellipsoid altitude
is GGA mean-sea-level altitude plus geoid separation. Without usable GGA,
altitude remains NaN; no altitude is invented. Acquisition stamps come from
RMC UTC date/time; synchronize machine clocks before judging message age.

`position_variance` in `l76x_gps_interface/config/gps_params.yaml` is ENU
variance in m^2. Its default zeros mean unknown. Configure measured uncertainty
before fusion; the GPS gate rejects unknown/nonpositive/invalid covariance,
no-fix status, invalid position/altitude, missing mount, stale/future or repeated
stamps. A stationary scatter estimate alone does not quantify absolute bias or
multipath; document how uncertainty was measured.

For a real rover, replace the bench body frame with `base_link` and supply
measured mounts. `rover_description/config/sensor_mounts.example.json` is a
non-runnable template. Fill it using measurements and set verified true, then:

```bash
ros2 launch rover_description sensor_mounts.launch.py mounts_file:=/absolute/path/to/measured_mounts.json
```

This is the single mount authority for a headless sensor setup. Do not run it
alongside a robot_state_publisher publishing those same sensor children. The
imported full URDFs contain simulation GPS/ICM mounts; their zero origins are
not measured hardware mounts. The ZED URDF + calibration bridge own only the
camera's internal tree. Add a measured `base_link -> imu_link` if enabling ICM;
the IMU filter no longer publishes placeholder mounts.

A verified global heading is a separate `sensor_msgs/Imu` input on
`/imu/heading`, in an ENU reference with yaw zero toward true east. Correct
magnetic declination and mounting offsets upstream. The ZED-only/two-IMU
filter's startup-relative yaw does NOT satisfy this contract.

Only after mounts, heading and GPS uncertainty are verified, enable the global
pipeline alongside the existing local EKF:

```bash
ros2 launch rover_localization gps_localization.launch.py enable_gps_fusion:=true heading_verified:=true mounts_verified:=true
```

Do not set the verification flags merely to bypass the check. `/gps/validated`
and `/localization/heading` feed navsat_transform. The global EKF uses local
body-frame motion, verified global orientation and `/odometry/gps`, and publishes
`/odometry/global` + the sole `map -> odom`. GPS never enters the continuous
local EKF. GPS loss should leave local odometry running; flag degraded global
position rather than claiming GPS accuracy from prediction alone.

## Synthetic checks and Gazebo

```bash
bash src/rover_localization/scripts/smoke_test.sh
```

Stop all live drivers/estimators first: the independent synthetic publisher uses
the default sensor topics. The probe checks finite 3D motion, increasing stamps,
time-matched truth, fresh TF and fresh accepted camera + IMU diagnostics. It
proves wiring/synthetic behavior only. It does not emulate ZED tracking failure,
GPS uncertainty, network latency or nonidentity physical mounts.

Gazebo is a separate test. `gazebo_launch.py` expands the controller configuration
path from the installed package and sets robot_state_publisher simulation time.
The drive controller no longer publishes motion TF; an estimator must own it.
The legacy `dual_ekf_navsat.yaml` remains a wheel/GPS example, not the live ZED
workflow. Gazebo camera images require visual odometry before they meet this
pipeline's camera input contract; do not launch the independent fake curved path
and treat it as Gazebo sensor fusion.

## Recording/replay tools

`run_imu_test.py` now records camera/GPS/EKF/diagnostic topics as well as IMUs/TF.
For a measured base_link rig, use `--no-icm` for ZED-only fusion and `--no-fused`
when using the ZED IMU directly. CSV logging rotates vectors and full covariance
into base_link using calibrated TF. For the unmounted camera bench, use the bag
command above; it preserves the original frame contracts without fake mounts.

CSV replay requires base_link rows and full 3x3 gyro/accel covariance matrices.
It rejects raw sensor-frame or legacy diagonal-only files. Prefer rosbag replay
for complete reproduction, since CSV lacks original receipt latency/TF history.
Replay with `--clock` and set `use_sim_time:=true` consistently; do not run the
hardware drivers or replay previously recorded EKF outputs into a second EKF
with the same output names. Use bag topic selection to replay only raw inputs
and calibration/TF, then rerun estimators.

## Source and native regression checks

Run the complete reproducible check command from any working directory:

```bash
python3 /absolute/path/to/AutonomousPathPlanning/scripts/check_localization.py --coverage
```

From the repository root on sourced ROS 2 Humble/Ubuntu, add the full package
build and tests (including the real ROS graph integration fixture):

```bash
python3 scripts/check_localization.py --coverage --ros --report /tmp/localization-check.json
```

The command discovers existing Eigen/C++ dependencies, rebuilds native tests in
a temporary directory, runs Python/Ruby/configuration checks and exits nonzero
on failures, timeouts or missing required dependencies. Use `--eigen-include`
if Eigen is installed in a nonstandard location. It installs nothing and does
not start hardware drivers. Without `--ros` it explicitly reports ROS/hardware
checks as not run. `--ros` builds packages, sources the resulting overlay for
tests and checks colcon test results; source ROS before invoking it.

The registered `localization_ros_e2e` fixture launches synthetic sensors, the
validator, real robot_localization EKF and the 3D smoke probe on localhost in a
test domain (197 by default). Set `ROVER_TEST_DOMAIN_ID` if other test workspaces
use that domain. The probe requires current camera/IMU diagnostic snapshots,
current odometry/truth, finite 3D agreement and fresh TF. A previously healthy
snapshot cannot remain a passing condition after its publisher disappears.
This fixture is hardware-free; the measured live test above is still required.

Individual checks are also available:

```bash
python3 -B -m unittest discover -s src/rover_localization/test -p 'test_*.py' -v
python3 -B -m unittest discover -s src/l76x_gps_interface/test -p test_gps_contract.py -v
ruby src/rover_localization/test/test_localization_contract.rb
```

On Ubuntu after building, run the full ROS package checks (not executed on the
macOS development machine):

```bash
colcon test --packages-select rover_localization rover_imu_fusion l76x_gps_interface --event-handlers console_direct+
colcon test-result --verbose
```

This includes the existing GoogleTest suite, native safety and CSV integration
checks, and Python sensor-contract tests. ROS transport, actual wrapper startup,
SDK calibration topic and GPS serial behavior still require the live procedure.
