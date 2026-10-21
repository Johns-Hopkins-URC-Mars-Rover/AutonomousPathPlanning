# rover_imu_fusion

The rover's IMU path. An error-state EKF fuses the ZED camera IMU and the
ICM-20948 into one `sensor_msgs/Imu` on `/imu/data` (frame `base_link`) with
orientation, angular rate, linear acceleration and covariances taken from the
filter.

```
/zed/zed_node/imu/data  (zed_imu_link)  --+
                                          |   rotate into base_link (TF), keep covariance
/imu/data_raw           (imu_link)      --+--> reorder buffer --> stationary init --> EKF --> /imu/data
/imu/mag  (optional yaw aid)            --+                                                  /diagnostics
```

Later stages (GNSS, ZED VIO, wheel odometry) are out of scope; they can consume
`/imu/data`. `rover_localization/config/ekf_local.yaml` already reads it as
`imu0`.

## Status

| Part | State |
| --- | --- |
| Filter core (ROS-free C++), 49 unit tests | done |
| `imu_fusion_node`, diagnostics, launch, config | done, tested with synthetic and bag data |
| ZED input | ready; **not yet run against the real ZED** |
| ICM input | ready but **off by default**: needs measured covariances from the driver and a measured `base_link -> imu_link` |
| Magnetometer yaw aid | implemented, off by default, uncalibrated |

## How it works

State (details and Jacobians are documented at the top of
`include/rover_imu_fusion/imu_ekf.hpp`):

- orientation `q` (world <- base_link), body angular rate `w`, specific force `a`
- gyro bias `bg_i` and accel bias `ba_i` per IMU (i = 0 ICM, 1 ZED), in
  `base_link` axes
- 21-dimensional error state: attitude error (3, body frame), then `w`, `a`,
  and the four biases

Model:

- `q` is integrated with the rate state. `w`, `a` and the biases are random
  walks. The process noise `rate_walk` is how fast the turn rate may change, so
  it has to cover real angular acceleration.
- Both IMUs are **measurements** (`z = w + bg_i`, `z = a + ba_i`), not the
  propagation input. Different rates, asynchronous arrival and one IMU
  dropping out need no special handling.
- Each IMU message is rotated into `base_link` with the TF `base_link ->
  header.frame_id`, looked up once and cached. Lever-arm terms are ignored;
  `accel_inflation` widens the accel noise to cover them.
- Gravity pseudo-measurement (roll/pitch): `a ~ R(q)^T g`. Its noise grows with
  `| |a| - g |` and the update is skipped above `gravity_skip` (driving over
  rocks). It runs at most every `gravity_update_period`.
- Yaw comes from the gyro only, so **yaw is relative to the start heading and
  its covariance grows over time**. That growth is correct. The optional
  magnetometer update limits it (relative to the start heading, not north).
- Orientation covariance is reported about fixed world axes (REP-145).
  `linear_acceleration` includes gravity. Rates and acceleration are the filter
  state, so they are bias corrected.

Measurement noise comes from the covariance in each message. Per IMU you can
set `*_variance_override` (replaces the message covariance) or
`*_variance_fallback` (used only when the message has none). An all-zero
covariance counts as "not provided". With neither set, that IMU's samples are
**dropped with a warning**: no noise values are invented.

The two bias sets are only separable through the difference between the IMUs.
The ZED bias gets a tight prior (it is already SDK corrected) which anchors the
common mode, so the ICM bias is learned relative to the ZED.

Robustness:

- reorder buffer (`reorder_delay`, 15 ms) so measurements are processed in
  stamp order; samples older than the filter time are counted as stale and
  dropped
- per-IMU timeout (`imu_timeout`)
- NIS chi-square gating; after `nis_max_consecutive` rejections in a row the
  next sample is accepted anyway, so a real step change cannot lock a stream out
- saturation and stuck-value detection per IMU
- nothing is published until initialized: about 1 s of stationary data seeds
  roll/pitch and the gyro biases. If the rover moves during that window it
  restarts. A steady rotation is not mistaken for gyro bias (`gyro_mean_max`).

The output is stamped with the filter time, which lags the clock by
`reorder_delay`.

## Topics

| Topic | Type | Notes |
| --- | --- | --- |
| `/zed/zed_node/imu/data` | `sensor_msgs/Imu` in | ZED, SDK bias corrected. The ZED publishes it only while something subscribes; this node is that subscriber. |
| `/imu/data_raw` | `sensor_msgs/Imu` in | ICM-20948, only with `icm.enabled` |
| `/imu/mag` | `sensor_msgs/MagneticField` in | only with `use_magnetometer` |
| `/imu/data` | `sensor_msgs/Imu` out | reliable QoS, 100 Hz by default |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` out | per IMU: live or timed out, age, applied and rejected counts (stale, NIS, saturated, stuck), forced accepts, bias estimates; filter: initialized, yaw and roll/pitch sigma, gravity updates |

All topic names are parameters in `config/imu_fusion.yaml`.

## Prerequisites

- ZED wrapper running so `/tf` has `base_link -> ... -> zed_imu_link`. The ZED
  URDF comes from the `zed_description` package, which is not in this repo:
  confirm its root is `base_link` with
  `ros2 run tf2_ros tf2_echo base_link zed_imu_link`.
- Nothing else publishes `/imu/data`. `rover_localization/simulated_sensors`
  now publishes its IMU on `/sim/imu`.
- Optional ZED overrides (same clock as the ICM, publish rate, raw IMU) without
  touching the submodule:

  ```bash
  ros2 launch zed_wrapper zed_camera.launch.py camera_model:=<model> \
    ros_params_override_path:=$(ros2 pkg prefix rover_imu_fusion)/share/rover_imu_fusion/config/zed_overrides.yaml
  ```

  `zed.use_raw: true` switches to `/zed/zed_node/imu/data_raw` and needs
  `publish_imu_raw: true` from that file.

## Build and test

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rover_imu_fusion
colcon test --packages-select rover_imu_fusion --event-handlers console_direct+
colcon test-result --verbose
```

The consistency tests are slow in a Debug build (about 30 s); use
`--cmake-args -DCMAKE_BUILD_TYPE=Release` for speed.

## Run

```bash
source install/setup.bash
ros2 launch rover_imu_fusion imu_fusion.launch.py            # ZED only
ros2 launch rover_imu_fusion imu_fusion.launch.py use_icm:=true icm_x:=0.1 icm_z:=0.2 icm_yaw:=1.5708
```

Keep the rover still for the first second. Then check:

```bash
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
ros2 topic echo /diagnostics        # filter "ok", IMUs "live"
```

Level and at rest, `linear_acceleration` reads about 9.8 on the base-frame up
axis and `angular_velocity` is close to zero.

`use_icm:=true` also publishes a placeholder `base_link -> imu_link` from the
`icm_*` arguments (default all zero). **Replace it with the measured mount**
(metres, radians) before trusting the ICM.

### Enabling the ICM

1. The ICM driver publishes measured covariances on `/imu/data_raw` (see
   `icm20948_imu_interface/config/imu_params.yaml`), or set
   `icm.gyro_variance_override` / `icm.accel_variance_override` here.
2. A measured `base_link -> imu_link` exists.
3. Launch with `use_icm:=true`. Check that `rover_imu_fusion/icm` is `live` and
   its bias estimates settle.

### Magnetometer

`use_magnetometer: true` (needs `icm.enabled`) adds a yaw update from
`/imu/mag`. Set `mag.hard_iron`, `mag.soft_iron` and `mag.to_imu_rotation`
first. The identity rotation is an assumption: the AK09916 axes may differ from
the accel/gyro axes, so verify it. Yaw stays relative to the start heading.

## Test without hardware

The simulator can publish synthetic IMUs on the real topic names with known
biases and noise (ICM gyro bias `[0.02 -0.01 0.015]`, accel bias
`[0.05 0 -0.03]`; the ZED stream has none). Both are aligned with `base_link`.

```bash
# terminal 1
ros2 run rover_localization simulated_sensors --ros-args \
  -p publish_synthetic_imus:=true -p start_delay:=3.0
# terminal 2
ros2 launch rover_imu_fusion imu_fusion.launch.py use_icm:=true
# terminal 3
ros2 topic echo /imu/data --once
ros2 topic echo /simulation/ground_truth --once
```

`start_delay` keeps the simulated rover still long enough for the filter to
initialize. Expect the fused yaw rate near 0.2 rad/s, yaw close to the ground
truth, and the ICM gyro bias estimates in `/diagnostics` near the values above.

Each terminal needs the two `source` lines first. With `ros2 topic echo`, add
`--no-daemon` if it errors about `rclpy.ok()`.

## Record and replay

```bash
# record (ZED and ICM drivers running, rover still for the first second)
ros2 bag record -o imu_bag /imu/data_raw /zed/zed_node/imu/data /tf /tf_static
```

Replay on sim time so the node's clock follows the bag. The bag path must come
before `--clock` (in Humble `--clock` takes an optional rate and would swallow
the path). The bag's `/tf_static` supplies the transforms, so turn the
placeholder off:

```bash
# terminal 1
ros2 launch rover_imu_fusion imu_fusion.launch.py use_icm:=true publish_icm_tf:=false use_sim_time:=true
# terminal 2
ros2 bag play imu_bag --clock
# terminal 3
ros2 topic echo /imu/data --once
```

Drop `use_icm:=true publish_icm_tf:=false` for a ZED-only replay. Record
`/imu/mag` too if you want to try the magnetometer.

## Tuning

The `filter`, bias and `init` values in `config/imu_fusion.yaml` are starting
points, not measured sensor properties. They are the main lever on the output
covariances:

- `rate_walk` and `accel_walk` set how much the rate and acceleration
  covariances stay inflated. They must cover how fast the rover really turns
  and accelerates; lower them for a smoother, more confident output.
- `gravity_sigma` sets how strongly roll/pitch follow the accelerometer.
- Bias walk and initial bias sigma set how fast the ICM bias adapts.

## Other node

`zed_imu_node` is the earlier single-IMU pass-through (ZED to `/imu/data`, no
filter). It publishes the same topic, so never run it together with
`imu_fusion_node`. Its launch file is `zed_imu.launch.py`.

## Layout

```
include/rover_imu_fusion/
  imu_ekf.hpp            state, process model, updates, Jacobians (core)
  imu_initializer.hpp    stationary start
  reorder_buffer.hpp     stamp-ordered release
  imu_fusion.hpp         buffer + init + EKF pipeline (core)
  imu_transform.hpp      covariance checks, rotation helpers (core)
  imu_rotator.hpp        ROS: rotate an Imu message into base_link
src/                     core sources and the two nodes
config/                  imu_fusion.yaml, zed_imu.yaml, zed_overrides.yaml
launch/                  imu_fusion.launch.py, zed_imu.launch.py
test/                    gtest for the core
```

## Not done

- Not run against the real ZED or ICM. Check the ZED TF root and its covariance
  values first.
- Real noise values: the ICM driver still publishes zero covariance until its
  noise has been measured.
- Extrinsics: `base_link -> imu_link` is a placeholder.
