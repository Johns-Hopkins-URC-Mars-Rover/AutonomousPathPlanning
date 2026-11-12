# Autonomous Path Planning

ROS 2 Humble workspace for the rover's positioning and sensor interfaces.

## Packages

| Package | Purpose |
| --- | --- |
| `ublox_dgnss` | Reads high-precision position data from supported u-blox GNSS receivers (F9P, F9R, and X20P). It also provides UBX messages, `NavSatFix` output, base/rover launch files, and NTRIP correction support. |
| `icm20948_imu_interface` | Publishes ICM-20948 accelerometer, gyroscope, magnetometer, and diagnostic data over ROS 2. |
| `zed-ros2-wrapper` | Stereolabs ROS 2 driver for ZED cameras. It publishes camera images, depth, point clouds, odometry, and sensor data. |
| `robot_localization` | Contains the local/global EKFs, ZED/wheel odometry and raw ZED/ICM gyro configuration, sensor adapters, heading/TF helpers, GPS conversion, and tests. See [`DUAL_EKF.md`](src/robot_localization/DUAL_EKF.md). |
| `rover_bringup` | Provides the top-level rover launch file that starts the enabled hardware drivers. |

### One-command sensor selection

After building and sourcing the ROS workspace, select components using `yes` or
`no` (the values are command arguments, not interactive prompts):

```bash
ros2 launch rover_bringup simple.launch.py \
  zed:=yes icm:=no wheel:=yes gps:=no localization:=yes drivers:=yes
```

`icm:=no` disables both the external ICM driver and its EKF input. `zed` controls
the ZED driver, optical odometry and gyro input; `wheel` selects an existing
wheel odometry topic, but does not start a motor controller. `gps:=yes` starts
GPS global localization when localization is enabled, as well as the GPS
driver when `drivers:=yes`. Use `drivers:=no` when all selected drivers already
run elsewhere. Defaults are `zed:=yes`, `icm:=no`, `wheel:=no`, `gps:=no`,
`localization:=yes`, and `drivers:=yes`.

GPS/global localization requires verified sensor mounts and an earth-referenced
heading. For example, with a measured startup heading, add
`gps:=yes heading_mode:=startup initial_heading_deg:=<measured_bearing>`;
the bearing uses true north = 0 and east = 90 degrees. The launch rejects an
unspecified heading. A URDF or other TF publisher must supply the sensor mounts;
see [the dual-EKF guide](src/robot_localization/DUAL_EKF.md) for provisional
bench mounts and the alternative verified heading topic. Do not publish two
sets of mount transforms.

The workspace also contains rover-specific navigation, localization, and GPS interface packages under `src/`.

## 1. Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- `colcon` and `rosdep`
- Git submodules checked out

The ZED package additionally requires a compatible NVIDIA CUDA installation and the [ZED SDK](https://www.stereolabs.com/developers/release/).

## 2. Clone the workspace

```bash
git clone --recurse-submodules https://github.com/Johns-Hopkins-URC-Mars-Rover/AutonomousPathPlanning.git
cd AutonomousPathPlanning
```

If the repository was cloned without submodules, run:

```bash
git submodule update --init --recursive
```

## 3. Install dependencies

```bash
source /opt/ros/humble/setup.bash
sudo rosdep init  # Run once per machine; skip if already initialized
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

If `rosdep` cannot resolve the ICM-20948 Python driver, install it separately:

```bash
python3 -m pip install sparkfun-qwiic-icm20948
```

## 4. Build

Build the entire workspace:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Or build only the package you need:

```bash
# u-blox driver and its dependencies
colcon build --symlink-install --packages-up-to ublox_dgnss

# ICM-20948 interface
colcon build --symlink-install --packages-select icm20948_imu_interface

# Self-contained local/global EKFs and rover bringup
colcon build --symlink-install --packages-select robot_localization rover_bringup \
  --cmake-args -DBUILD_TESTING=OFF

# ZED wrapper (ZED SDK and CUDA must already be installed)
colcon build --symlink-install --packages-up-to zed_wrapper \
  --packages-skip zed_debug \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

After any build, source the workspace before running a node:

```bash
source install/setup.bash
```

## 5. Run

### Rover bringup

Connect the IMU and ZED camera, then start both drivers with one command:

```bash
ros2 launch rover_bringup rover.launch.py camera_model:=zed2i
```

Replace `zed2i` with the attached camera model. ICM and localization are disabled
by default; L76X GPS and ZED are enabled. The camera default is `zed2i`, so the
argument can be omitted on a rover using that model. Either driver can be
disabled while diagnosing hardware:

```bash
ros2 launch rover_bringup rover.launch.py enable_imu:=false camera_model:=zedx
ros2 launch rover_bringup rover.launch.py enable_zed:=false
```

When another subsystem is ready, add its package as an `exec_depend` in
`src/rover_bringup/package.xml` and include its launch file from
`src/rover_bringup/launch/rover.launch.py` in the same way.

### u-blox GNSS

Connect the receiver over USB, then launch high-precision latitude/longitude and `NavSatFix` output:

```bash
ros2 launch ublox_dgnss ublox_rover_hpposllh_navsatfix.launch.py
```

For a ZED-X20P receiver, use:

```bash
ros2 launch ublox_dgnss ublox_x20p_rover_hpposllh_navsatfix.launch.py
```

USB permissions may require the udev rules documented in [`src/ublox_dgnss/README.md`](src/ublox_dgnss/README.md).

### ICM-20948 IMU

Connect the sensor over I2C and ensure the current user has permission to access the I2C device, then run:

```bash
ros2 run icm20948_imu_interface imu_publisher
```

The node publishes `/imu/data_raw`, `/imu/mag`, and `/imu/diagnostics`.

### Rover localization

For the direct two-IMU and wheel/ZED dual-EKF workflow, follow
[the dual-EKF build and run guide](src/robot_localization/DUAL_EKF.md).
Enable local estimation from bringup with:

```bash
ros2 launch rover_bringup rover.launch.py enable_imu:=true \
  enable_localization:=true publish_approximate_mounts:=true
```

The approximate mounts assume a centered IMU/GPS and a front-mounted ZED on
the 27-inch-long rover. Disable them when the URDF supplies measured mounts.
GPS/global estimation additionally requires an earth heading and usable GPS
covariance; neither is silently guessed.

Missing wheel or IMU data does not block the remaining inputs. For a test
without wheel encoders, add `use_wheel_odom:=false`; see the guide for the
other source switches and dropout/recovery diagnostics.

Run `bash scripts/test_localization_container.sh` to build and test the source
EKFs in isolation, including through bringup, with synthetic sensors. The test
environment excludes the retiring custom localization and IMU-fusion packages.

### ZED camera

Connect the camera and replace `<camera_model>` with its model, such as `zed2i`, `zedx`, or `zedxm`:

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=<camera_model>
```

See [`src/zed-ros2-wrapper/README.md`](src/zed-ros2-wrapper/README.md) for supported models and advanced configuration.

## Docker (optional)

The included image provides ROS 2 Humble and the rover's common ROS dependencies:

```bash
docker build -t autonomous-path-planning:humble .
docker run -it --name autonomous-path-planning \
  --mount type=bind,source="$(pwd)",target=/ros2_ws \
  autonomous-path-planning:humble bash
```

Inside the container, repeat steps 3–5 (omit `sudo` because the container runs as root). Hardware must be passed through to the container. The base image does not include the ZED SDK, CUDA runtime, or NVIDIA container configuration required for a ZED camera.

To reopen the container later:

```bash
docker start -ai autonomous-path-planning
```

## Localization with L76X GPS

The build, run and output guide is [robot_localization/DUAL_EKF.md](src/robot_localization/DUAL_EKF.md).
On two machines, the Jetson runs the ZED/L76X/ICM drivers with localization
disabled; the ROS computer runs `robot_localization/dual_ekf.launch.py` and RViz.
Synchronize their clocks and provide sensor TF on the ROS network. GPS/global
fusion requires a measured startup bearing or a calibrated heading topic and
usable GPS covariance. The full rover URDF's sensor origins are simulation
fixtures, not measured hardware mounts.
