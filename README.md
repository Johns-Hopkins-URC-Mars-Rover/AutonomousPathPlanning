# Autonomous Path Planning

ROS 2 Humble workspace for the rover's positioning and sensor interfaces.

## Packages

| Package | Purpose |
| --- | --- |
| `ublox_dgnss` | Reads high-precision position data from supported u-blox GNSS receivers (F9P, F9R, and X20P). It also provides UBX messages, `NavSatFix` output, base/rover launch files, and NTRIP correction support. |
| `icm20948_imu_interface` | Publishes ICM-20948 accelerometer, gyroscope, magnetometer, and diagnostic data over ROS 2. |
| `zed-ros2-wrapper` | Stereolabs ROS 2 driver for ZED cameras. It publishes camera images, depth, point clouds, odometry, and sensor data. |
| `rover_localization` | Configures the system-installed `robot_localization` binaries as local/global EKFs with GPS conversion. |
| `rover_bringup` | Provides the top-level rover launch file that starts the enabled hardware drivers. |

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

# Rover local/global EKF configuration and bringup
colcon build --symlink-install --packages-select rover_localization rover_bringup

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

Install the ROS binary dependency, then enable dual-EKF localization from bringup:

```bash
sudo apt install ros-humble-robot-localization
```

```bash
ros2 launch rover_bringup rover.launch.py enable_imu:=true \
  enable_localization:=true
```

The ZED wrapper's TF publication is disabled by bringup so the EKFs remain the
only owners of `odom -> base_link` and `map -> odom`. Global estimation requires
usable GPS covariance and an absolute orientation source.

Run `bash scripts/test_localization_container.sh` to build and test the rover
configuration against the released ROS binaries.

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

On two machines, the Jetson runs the ZED/L76X/ICM drivers with localization
disabled; the ROS computer runs `rover_localization/dual_ekf_launch.py` and RViz.
Synchronize their clocks and provide sensor TF on the ROS network. GPS/global
fusion requires a measured startup bearing or a calibrated heading topic and
usable GPS covariance. The full rover URDF's sensor origins are simulation
fixtures, not measured hardware mounts.
