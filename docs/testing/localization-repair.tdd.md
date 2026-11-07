# Localization and L76X repair evidence

Source plan: [localization-repair.plan.md](localization-repair.plan.md).
Scope: rover-owned packages/scripts. Upstream ZED/u-blox sources unchanged.
Changes remain uncommitted; Git metadata is read-only in this session.

## User journeys and implementation

- Run the ZED/L76X on a Jetson and the estimators on another ROS machine.
- Obtain local camera/gyro motion in a deliberately specified body frame.
- Observe rejected/stale measurements rather than equating output with accuracy.
- Record GPS now, enable global GPS localization only after its prerequisites
  are verified, and preserve continuous local odometry.
- Reproduce the run from recordings with correct frames/full covariance.

| Plan behavior | Regression / evidence | Outcome |
| --- | --- | --- |
| Reject NaNs, repeated timestamps and bad Z in smoke evaluation | `test_smoke_probe.py` | RED: 3 failures; GREEN: all 8 tests pass |
| Convert camera pose with a nonidentity rigid mount and propagate lever arm covariance | `test_sensor_contracts.py` | 5 tests pass, including rotated and translated mounts |
| Validate node callbacks, freshness, duplicates, missing TF, GPS/heading and calibration bridge | `test_sensor_nodes.py` | 7 callback/transport-double tests pass |
| Remove inverted mount and align launch interfaces | `test_launch_contracts.py` | RED: bringup emitted a placeholder static mount; GREEN: all 5 launch-construction tests pass |
| Add GPS RMC/GGA altitude and preserve startup failures | `test_gps_contract.py` | RED: absent GGA parser and UnboundLocalError masking startup; GREEN: all 7 tests pass |
| Record GPS/camera/EKF/diagnostics and allow no-ICM sessions | `test_recording_contracts.py` | RED: missing topics and required-ICM-only API; GREEN: both tests pass |
| Prevent invalid input entering IMU core; stop fresh-looking output after all inputs expire | `test_input_safety.cpp` | RED: native runtime failure on NaN acceptance; GREEN: standalone native regression passes |
| Replay only base-frame data with complete covariance | `test_replay_csv.py` + native replay binary | 3 native integration cases pass: valid CRLF CSV accepted, wrong frame and legacy covariance rejected |
| Keep motion checks active under NDEBUG | `test_simulation_model.cpp` | Standalone native test built/run with `-DNDEBUG`; passes |

The first sensor-contract test execution failed because the new module was not
implemented yet; that is missing-feature evidence, not a runtime reproduction of
an existing math bug. No claim is made that every added launch/node branch had
an independent pre-implementation RED run. The table records the actual evidence.

## Commands actually run

```bash
python3 -B -m unittest discover -s src/rover_localization/test -p 'test_*.py' -v
python3 -B -m unittest discover -s src/l76x_gps_interface/test -p test_gps_contract.py -v
ruby src/rover_localization/test/test_localization_contract.rb
python3 -B src/rover_imu_fusion/test/test_replay_csv.py /tmp/rover_replay_csv
/tmp/rover_input_safety
/tmp/rover_simulation_model_test
git -c core.whitespace=cr-at-eol diff --check
```

Results: 34 Python tests pass; Ruby 8 tests / 58 assertions pass; native IMU
safety and Release motion-model regressions pass; all 3 native replay cases pass.
The native replay test also exposed a CRLF header bug on a valid Python-written
CSV; after stripping the terminal carriage return, the same case passed.

Native binaries were compiled with clang++, C++17, `-O1 -DNDEBUG`, the repository
headers and the already-installed Eigen 3.4.0 headers. The safety/replay binaries
link `imu_ekf.cpp`, `imu_fusion.cpp`, `imu_initializer.cpp`, `imu_transform.cpp`.
No packages were installed. Python AST, package XML, URDF and launch XML parse;
modified YAML parses with Ruby's safe YAML loader. Bash smoke script syntax
checks pass. Original CRLF file conventions are preserved; diff whitespace
validation uses Git's `cr-at-eol` setting for those existing files.

## Coverage and limits

Standard-library trace measurements across the 34 Python tests cover:

| New module | Observed executable-line coverage |
| --- | --- |
| sensor_contracts.py | 74/80, 92.5% |
| sensor_inputs.py | 138/143, 96.5% |
| zed_imu_tf.py | 44/49, 89.8% |
| nmea.py | 61/63, 96.8% |

These conservative denominators include trace's synthetic line 0/None entries.
This is helper/callback coverage, not repository-wide, branch, ROS transport,
GPS serial-driver or C++ coverage. Transport doubles execute the real callbacks
but cannot validate DDS/QoS, TF distribution or installed wrapper compatibility.

No ROS 2/colcon is installed on this host. Full colcon builds, the existing
GoogleTest suite, live ZED/Jetson behavior, GPS serial commands, ROS bag playback,
RViz and Gazebo have NOT been executed here. Their commands and acceptance
procedure are in `src/rover_localization/TESTING.md`.

## Hardware and scientific boundaries

The user confirmed L76X `/gps/fix`, with neither mounts nor global heading
verified. Thus GPS fusion defaults off, and measured uncertainty defaults unknown.
Do not enable verification flags to bypass physical prerequisites. RMC UTC dates
and machine UTC clocks must agree. Do not replace missing altitude or calibrated
camera/IMU geometry with zeros.

No arbitrary camera rejection threshold or covariance tuning was introduced to
mask the observed error. The source checks do not prove the earlier ~17 m
position error is resolved. Capture raw camera odometry, converted pose, gyro,
GPS, filter output, tracking status, TF, diagnostics and effective parameters
through a measured movement to locate the first failing stage.

The optional global path uses robot_localization/navsat_transform rather than a
custom geographic estimator. It uses verified ENU true-east-zero orientation;
startup-relative IMU yaw is explicitly insufficient. Local odometry never fuses
GPS. Imported full-model GPS/IMU origins remain simulation fixtures; use the
single measured mount authority on hardware.

Primary references consulted:
- https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/humble-devel/doc/navsat_transform_node.rst
- https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/humble-devel/src/ros_filter.cpp
- https://raw.githubusercontent.com/stereolabs/zed-ros2-wrapper/master/zed_wrapper/launch/zed_camera.launch.py
- https://docs.stereolabs.com/docs/integrations/ros-2/zed-stereo-node


## Full-plan verification follow-through

Replaced the ephemeral verification procedure with:

```bash
python3 -B scripts/check_localization.py --coverage --report /tmp/rover-localization-verification.json
```

Actual result: PASS, 40 Python tests, Ruby 8 tests / 58 assertions, fresh C++
core builds, native safety checks, three CSV replay integration cases and the
NDEBUG motion-model test. Syntax/configuration checks also pass. The saved
[JSON verification result](localization-verification.json) explicitly records
`ros_requested: false` and each subprocess result.

Trace coverage, excluding synthetic line 0/None entries: sensor_contracts
74/78 (94.9%), sensor_inputs 138/141 (97.9%), zed_imu_tf 44/47 (93.6%), nmea
61/61 (100%). This still covers the helper modules, not the entire repository.

Additional RED/GREEN: stale health/output tests initially failed for missing
expiry APIs; all three now pass. Verification-command runtime tests initially
failed on placeholder success returns/missing Eigen checks; they now prove
nonzero subprocess exits and timeouts cause failure, and explicit header paths
are validated.

`localization_ros_e2e` is now registered through launch_testing_ament_cmake. It
starts the real synthetic sensor -> validation -> EKF -> probe graph, waits for
the probe's bounded shutdown and requires its zero exit code. It runs with
ROS_LOCALHOST_ONLY=1 and a separate test domain. This fixture has been syntax
checked here but NOT executed, since ROS is unavailable.

The explicit negative prerequisite check was also run:

```bash
python3 -B scripts/check_localization.py --ros --report /tmp/rover-ros-prerequisites.json
```

It exited 1 with the expected requirement to use sourced ROS 2/colcon on Ubuntu.
That is a verified missing prerequisite, not a successful ROS test. No mounts,
GPS uncertainty, heading or hardware success were fabricated. Final live motion
accuracy and resolution of the observed ~17 m error remain unverified.
