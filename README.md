# Autonomous Path Planning

ROS 2 Humble workspace for the rover's positioning and sensor interfaces.

## Packages

| Package | Purpose |
| --- | --- |
| `ublox_dgnss` | Reads high-precision position data from supported u-blox GNSS receivers (F9P, F9R, and X20P). It also provides UBX messages, `NavSatFix` output, base/rover launch files, and NTRIP correction support. |
| `icm20948_imu_interface` | Publishes ICM-20948 accelerometer, gyroscope, magnetometer, and diagnostic data over ROS 2. |
| `zed-ros2-wrapper` | Stereolabs ROS 2 driver for ZED cameras. It publishes camera images, depth, point clouds, odometry, and sensor data. |
| `rover_localization` | Configures a local `robot_localization` EKF for ZED optical odometry and the team's joint IMU stream. Includes a hardware-free synthetic sensor launch; see [`src/rover_localization/README.md`](src/rover_localization/README.md). |

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

See the [localization testing guide](src/rover_localization/TESTING.md) for
the hardware-free synthetic test, real-ZED/fake-IMU bench test, and validation
limits.

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
