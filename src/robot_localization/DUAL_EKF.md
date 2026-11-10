# Direct two-IMU, wheel/ZED, and GPS localization

The supported entry point is `ros2 launch robot_localization dual_ekf.launch.py`.
All configurations, sensor adapters, heading/TF helpers and tests for this
workflow live in this package. No other custom localization or IMU-fusion
package is required. Hardware drivers publish the input messages separately.

## Data flow and ownership

```text
/zed/zed_node/odom ----> camera-to-base pose adapter ------+
/diff_drive_controller/odom --> validated wheel twist ----+--> ekf_local
/zed/zed_node/imu/data --> validated angular velocity ----+    /odometry/local
/imu/data_raw ----------> validated angular velocity ----+    odom -> base_link
                                                        |
                      same raw measurements ------------+--> ekf_global
                                                             /odometry/global
/gps/fix --> /gps/validated --> navsat_transform ------------> /odometry/gps
                               ^     ^                       map -> odom
                               |     |
/odometry/global --------------+     |
true-earth heading ------------------+
```

`/odometry/gps` is `nav_msgs/msg/Odometry` in `map`. It contains a converted
GPS position; it is not another fused estimate or encoder odometry. The global
EKF fuses its x/y with the motion measurements. `/gps/filtered` is the global
estimate converted back to latitude/longitude, not an independent GPS fix.

Both EKFs publish at 30 Hz. Navsat checks for fixes at 10 Hz; the receiver's
actual update rate still limits GPS output. The local estimate stays smooth
when GPS jumps. The global estimate can change as GPS corrects drift.

The initial configuration is **planar**: x, y, yaw, planar velocity and yaw
rate. Z/roll/pitch are constrained to zero. It suits an initial Nav2 deployment
on approximately planar ground. Full terrain attitude/elevation requires a
separately validated 3D configuration; flipping `two_d_mode` alone is not enough.

Measurement choices in `params/dual_ekf.yaml`:

| Input | Local filter | Global filter |
| --- | --- | --- |
| ZED pose, converted to base origin | x/y/yaw relative to its first pose | Differential x/y/yaw: motion increments |
| Wheel odometry | Forward velocity and yaw rate | Forward velocity and yaw rate |
| ZED IMU | Yaw rate | Yaw rate |
| ICM-20948 IMU | Yaw rate | Yaw rate |
| Converted GPS | Excluded | Absolute x/y |

The global filter does not consume the local filtered estimate plus its raw
inputs. This avoids counting those measurements twice. Absolute ZED pose is
not fed back through `map -> odom` into the global filter.

The ZED launch uses optical-only tracking (`pos_tracking.imu_fusion: false`)
because the external EKFs already use its gyro. Its odometry and map TF output
are disabled. If you run ZED manually, apply `params/zed_optical.yaml` and keep
those overrides. Both raw gyros feed the EKFs directly.
No accelerometer data or `/imu/mag` is fused. Raw gyro biases still need
calibration; robot_localization does not estimate a per-sensor gyro bias.

## Build in ROS 2 Humble / Ubuntu 22.04

Run on the Jetson or inside your Linux ROS container, from the workspace root:

```bash
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src/robot_localization \
  --ignore-src -r -y

colcon build --symlink-install \
  --packages-select robot_localization rover_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
ros2 pkg prefix robot_localization
```

The prefix must point into this workspace's `install/`, not `/opt/ros/humble`.
The sensor drivers must also already be built. If necessary, build them with:

```bash
colcon build --symlink-install \
  --packages-select icm20948_imu_interface l76x_gps_interface
source install/setup.bash
```

This clone is version 4.0 with lifecycle EKFs. The launch automatically
configures and activates them. Small source compatibility changes cover
Ubuntu 22.04's yaml-cpp target, Humble's message_filters APIs, TF broadcaster,
and NodeOptions. The EKF prediction/correction algorithms are unchanged.
Use the source-built package: the distribution binary does not contain these
rover-specific scripts, parameters or launch files.

## Frames and approximate mounts

The launch optionally publishes these assumptions from your description:

| Transform | xyz [m] | rpy [rad] |
| --- | --- | --- |
| base_link -> imu_link | 0, 0, 0 | 0, 0, 0 |
| base_link -> gps_link | 0, 0, 0 | 0, 0, 0 |
| base_link -> zed_camera_link | 0.3429, 0, 0 | 0, 0, 0 |

This assumes 27 inches is the front-to-back length, 21 inches is the width,
and the ZED is centered at the front. Heights and rotations are unmeasured;
all sensor origins are provisionally assumed at base height. A centered GPS
means the antenna, not the serial connector. Replace these with actual
measurements, particularly camera tilt and IMU axis orientation.

Use `publish_approximate_mounts:=true` only when the URDF or another TF publisher
is not already supplying those sensor mounts. The ZED driver launch publishes
the calibrated internal camera-to-IMU transform through its SDK TF bridge.
Never approximate the internal ZED IMU mount with another identity transform.

The wheel controller must have `enable_odom_tf: false`; the local EKF is the
only owner of `odom -> base_link`. Disable ZED's motion TF as above. Do not
run AMCL, SLAM, or another GPS filter publishing `map -> odom` alongside the
global EKF. Static sensor mounts do not provide a world position themselves.

## Start local localization now

One command to launch the existing sensors and local EKF:

```bash
ros2 launch rover_bringup rover.launch.py \
  enable_imu:=true enable_localization:=true \
  publish_approximate_mounts:=true
```

This does not start motor controllers or command the motors. A wheel driver
can supply `/diff_drive_controller/odom` when available; its absence does not
prevent either EKF from using ZED odometry and the other available inputs.

If sensors are already running, start only localization instead:

```bash
ros2 launch robot_localization dual_ekf.launch.py \
  publish_approximate_mounts:=true
```

Start only one of these commands. All topic names can be overridden using
`zed_odom_topic`, `wheel_odom_topic`, `zed_imu_topic`, `icm_imu_topic` and
`gps_topic` on the standalone launch.

## Missing inputs, ZED-only tests, and recovery

Inputs are processed independently. Neither EKF waits for every configured
sensor. An enabled source that never publishes, stops publishing, or fails
validation does not block healthy sources. No zero measurements are fabricated
for a missing source. Valid fresh messages from an enabled source are accepted
again automatically when it returns.

For a test with ZED odometry and both IMUs but no wheel odometry:

```bash
ros2 launch rover_bringup rover.launch.py \
  enable_imu:=true enable_localization:=true use_wheel_odom:=false \
  publish_approximate_mounts:=true
```

For ZED odometry and the ZED IMU without the external ICM or wheel encoders:

```bash
ros2 launch rover_bringup rover.launch.py \
  enable_localization:=true use_wheel_odom:=false use_icm_imu:=false \
  publish_approximate_mounts:=true
```

Both entry points accept `use_zed_odom`, `use_wheel_odom`, `use_zed_imu` and
`use_icm_imu` (all default true). These control which measurements the adapters
forward to the EKFs. Driver switches such as `enable_imu` control the hardware
process separately. An explicitly disabled input is reported as disabled;
relaunch with its `use_*` switch true to include it later. Leaving it enabled
allows it to join automatically when it starts publishing.

In `/diagnostics`, an enabled unavailable/stale source is WARN, a disabled
source is OK/disabled, and other valid sources remain OK. Invalid measurements
or duplicate publishers can produce a source ERROR without stopping the EKFs.
The `dual_sensor_inputs/availability` summary distinguishes full input
availability, degraded operation, and complete loss of fresh motion inputs.

Before any accepted measurement, an EKF waits without publishing an initialized
estimate. Once initialized, it continues prediction during a complete input
outage at the configured 30 Hz (`predict_to_current_time: true`) and its
uncertainty grows. The availability summary then reports ERROR;
this is a health indication, not a process shutdown. An output topic that is
still publishing does not guarantee a measured position. IMU yaw rates alone
cannot measure translation. GPS loss removes global corrections but leaves
local motion estimation running. Heading/fix requirements still apply to
initializing GPS coordinate conversion.

## Enable GPS/global localization when heading is available

There is currently no verified earth heading. ZED startup yaw is arbitrary;
the raw ICM topic explicitly has no orientation. A single-antenna GPS at rest
cannot provide vehicle heading. `/imu/mag` needs hard/soft-iron calibration,
mount alignment and a magnetic-to-true heading conversion before use.

Local estimation works without earth heading. Global GPS fusion requires
one of these explicit choices; the launch rejects an unspecified heading.

### Option A: measure the parked startup heading

Use a known surveyed direction or a true-north compass reading away from motor
interference. Enter degrees clockwise from true north: N=0, E=90, S=180, W=270.
Keep the rover parked until `/odometry/gps` starts. Restart this workflow if
the local estimator or ZED tracking origin is reset.

```bash
# Example ONLY if the rover is actually parked facing true east.
ros2 launch robot_localization dual_ekf.launch.py \
  publish_approximate_mounts:=true enable_global:=true \
  heading_mode:=startup initial_heading_deg:=90
```

`startup_heading.py` combines that initial bearing with subsequent local yaw
for navsat's coordinate alignment. It is not an independent heading observation
and is never fused back into either EKF. Do not substitute 90 for an unknown
heading. Bearing error rotates GPS corrections: at 100 m, a 10-degree error
can create approximately 17 m of cross-track error.

### Option B: publish an earth-referenced Imu orientation

```bash
ros2 launch robot_localization dual_ekf.launch.py \
  publish_approximate_mounts:=true enable_global:=true \
  heading_mode:=imu heading_topic:=/imu/heading
```

The heading message must contain a valid quaternion and nonzero orientation
covariance: ENU, yaw zero at true east, positive counterclockwise. Its frame
must have a TF to base_link. Correct magnetic declination and frame convention
upstream. The configured navsat yaw and declination offsets are zero.

The default datum is established from the first accepted GPS/heading/odometry
combination. `map` initially follows the rover's local startup axes; it is not
automatically true-east/true-north aligned. Set a known datum in a custom
navsat configuration if future missions need a repeatable surveyed map.

### GPS covariance must be supplied

The L76X driver already combines RMC/GGA and calculates ellipsoid altitude.
Its default `position_variance: [0.0, 0.0, 0.0]` means unknown uncertainty;
the validation node will reject these fixes for fusion.

Set ENU position variances in `l76x_gps_interface/config/gps_params.yaml` and
restart the GPS driver. Variance is squared standard deviation, in m^2.
For example `[100.0, 100.0, 225.0]` represents an assumed horizontal standard
deviation of 10 m and vertical standard deviation of 15 m. Those are possible
conservative commissioning priors, **not measured L76X performance**. A quoted
3–10 m accuracy range does not specify a statistical standard deviation.
Replace the prior with receiver-provided uncertainty or field measurements.

Even in planar mode, the input needs finite altitude for the GeographicLib
conversion; `zero_altitude` zeros the result afterwards. Valid RMC without a
matching valid GGA can still have NaN altitude and be rejected. Unknown or
zero wheel/gyro covariance is also rejected; fix the source covariance rather
than labeling the filter prediction as a successful sensor fusion.

To run the whole stack after these prerequisites are met, add the global
arguments to the `rover_bringup` command:

```bash
ros2 launch rover_bringup rover.launch.py \
  enable_imu:=true enable_localization:=true \
  publish_approximate_mounts:=true enable_global:=true \
  heading_mode:=startup initial_heading_deg:=90
```

Again, 90 is only correct for a rover actually facing east.

## Inspect and interpret the results

```bash
ros2 node info /ekf_local
ros2 topic hz /odometry/local
ros2 topic echo /odometry/local --once
ros2 topic echo /diagnostics --once
ros2 run tf2_ros tf2_echo odom base_link
```

`/odometry/local.header.frame_id` is `odom`; `child_frame_id` is `base_link`.
`pose.pose.position` is displacement in meters from the local startup origin.
`pose.pose.orientation` is a quaternion, not Euler angles.
`twist.twist.linear.x` is forward velocity in m/s; `angular.z` is turn rate in
rad/s. Positive turns are counterclockwise. The 6x6 covariance order is
x, y, z, roll, pitch, yaw (and corresponding velocities in twist). Entries
0/7/35 are x/y/yaw variance; sqrt gives a standard deviation, in m or rad.
Planar z/roll/pitch zeros are model constraints, not measured flatness.

With global enabled:

```bash
ros2 topic hz /odometry/global
ros2 topic echo /odometry/gps --once
ros2 topic echo /gps/filtered --once
ros2 run tf2_ros tf2_echo map odom
```

Global pose is in `map`, and its twist is in `base_link`. The global filter
publishes `map -> odom`, not a competing `odom -> base_link`. Their composition
locates the rover in map. A global filter can publish while no GPS is being
accepted: check `/odometry/gps` **and** diagnostic accepted counts before
calling it GPS localization. The filter can continue prediction after sensors
stop; output frequency alone does not prove fresh input.

Input diagnostics report accepted/rejected counts and the age of the last
accepted message for camera, wheel, both IMUs, GPS and heading. Each raw topic
must have exactly one publisher; do not replay bags alongside live drivers.
Synchronize clocks if ROS nodes run on multiple machines. In RViz use `odom`
for local motion and `map` for global motion; add Odometry, TF and GPS-derived
Odometry displays. Point clouds are for perception and are not EKF inputs.

Nav2 should read `/odometry/local` for velocity and use the same
`map -> odom -> base_link` TF tree. Fine obstacle clearance comes from local
perception; GPS corrections retain the receiver's meter-scale uncertainty.

## Reproduce the synthetic integration test

After installing test dependencies, build this package with rover tests enabled.
The upstream C++ test suite can remain disabled for the Humble build:

```bash
colcon build --symlink-install --packages-select robot_localization \
  --cmake-args -DBUILD_TESTING=OFF -DBUILD_ROVER_TESTING=ON
source install/setup.bash
colcon test --packages-select robot_localization --event-handlers console_direct+ \
  --ctest-args -R '^rover_' --output-on-failure
colcon test-result --verbose
```

The tests use domain 198, localhost only, synthetic mounts/sensor streams,
both real EKFs and the real navsat node. They cover the full-input baseline,
an absent wheel publisher, explicitly configured ZED-only odometry, late first
measurements, individual camera/IMU/GPS outages, publisher reconnection, and a
complete input outage followed by recovery. They check output frames, growing
prediction uncertainty, availability diagnostics and that a 2 m GPS jump does
not jump local odometry. These establish graph/configuration behavior, not
real tracking accuracy or tuned noise.

The `rover_noisy_e2e` test adapts the earlier rover localization noisy-motion
fixture to this planar local EKF. It uses two seeded baseline runs and a
camera tracking-fault run, with a synthetic camera mount, one synthetic gyro,
timestamp errors, latency, and sensor outages. Wheel odometry, the second IMU,
and GPS are disabled in this fixture. It reports planar position and yaw errors
against independent synthetic truth and checks input loss and recovery. The
fault run characterizes the effect of incorrect camera poses; it does not prove
that the EKF rejects them. The assumptions and error bounds are regression
fixtures, not measured hardware performance.

The Docker workflow below builds only robot_localization and rover_bringup.
It checks that neither retiring package is discoverable, then tests the real
EKFs/navsat directly and through rover_bringup using synthetic sensor messages:

```bash
bash scripts/test_localization_container.sh
```

For ROS-free sensor math and adapter checks:

```bash
python3 -B -m unittest discover -s src/robot_localization/test -p 'test_rover_*.py'
python3 -B -m unittest discover -s src/robot_localization/test -p 'test_sensor_contracts.py'
```

Synthetic results establish the graph and coordinate-conversion behavior,
not real sensor accuracy or tuned covariance.

## Upstream references

- [Nav2 dual-EKF GPS navigation tutorial](https://docs.nav2.org/rolling/tutorials/general_tutorials/navigation2_with_gps/navigation2_with_gps/)
- [robot_localization GPS integration source](https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/integrating_gps.rst)
- [navsat_transform parameters](https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/navsat_transform_node.rst)
- [Sensor selection, differential/relative modes and duplicate measurements](https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/configuring_robot_localization.rst)
- [ZED positional tracking and TF parameters](https://docs.stereolabs.com/docs/integrations/ros-2/zed-stereo-node)

The checked-in source documentation and compiled behavior govern this clone;
Rolling examples may use different APIs from Humble.
