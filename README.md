# Autonomous Path Planning

Incorporates Nav2 with ROS2 to autonomously drive our rover to a GPS coordinate while also avoiding obstacles. The workspace starts the rover's IMU, GPS, ZED camera, robot model, dual-EKF localization, and Nav2. Nav2 uses the ZED point cloud for its local and global costmaps. It sends velocity commands to the rover's wheel controller.

For design details and troubleshooting, see the [project Wiki](https://github.com/Johns-Hopkins-URC-Mars-Rover/AutonomousPathPlanning/wiki).

## Packages present in this repository

| Package | Purpose |
| --- | --- |
| `icm20948_imu_interface` | Publishes raw IMU and magnetic-field data. |
| `l76x_gps_interface` | Publishes the L76X GPS fix. |
| `rover_description` | Contains the rover URDF, meshes, and sensor transforms. |
| `rover_localization` | Runs the local EKF, global EKF, and `navsat_transform`. |
| `rover_navigation` | Runs Nav2 and sends a GPS target to it. |
| `rover_bringup` | Starts the physical-rover sensor and localization stack. |
| `zed-ros2-wrapper` | Official Stereolabs ZED ROS 2 driver. |

Packages under `src/archives` are not part of the active system and are not built. Some of them, such as the ublox_dgnss, will be incorporated in future builds.

## Setup Requirements

Use Ubuntu 22.04 with ROS 2 Humble. The rover computer must also have:

- a compatible NVIDIA/CUDA installation and ZED SDK;
- access to the ZED camera, I2C bus, and `/dev/ttyUSB0`;
- the physical wheel controller running;
- synchronized system time if ROS nodes run on more than one computer.

The wheel controller is external to this repository. It must:

- subscribe to `/diff_drive_controller/cmd_vel_unstamped` as `geometry_msgs/msg/Twist`;
- publish `/diff_drive_controller/odom` as `nav_msgs/msg/Odometry`;
- treat positive `linear.x` as physical forward and positive `angular.z` as counter-clockwise.

## 1. Clone and install

```bash
git clone --recurse-submodules \
  https://github.com/Johns-Hopkins-URC-Mars-Rover/AutonomousPathPlanning.git
cd AutonomousPathPlanning

source /opt/ros/humble/setup.bash
sudo rosdep init  # Run this once per computer. Skip it if already initialized.
rosdep update
rosdep install --from-paths src --ignore-src -r -y

python3 -m pip install --user \
  sparkfun-qwiic-icm20948 smbus2 Jetson.GPIO
```

If the repository was cloned without its submodule, run:

```bash
git submodule update --init --recursive
```

## 2. Build

We use colcon to build all our packages. Note: the ZED SDK must be installed before building the ZED wrapper.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-skip zed_debug \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Source the workspace again in every new terminal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```
You can make the above an alias for ease of use

## 3. Start the rover

Connect the sensors and start the physical wheel controller first. Then run:

```bash
ros2 launch rover_bringup rover.launch.py \
  enable_imu:=true \
  enable_gps:=true \
  enable_zed:=true \
  enable_robot_state_publisher:=true \
  enable_localization:=true \
  use_sim_time:=false
```

This starts the sensor drivers, Madgwick heading filter, robot transforms, both EKFs, and `navsat_transform`. It does not start Nav2.
Similalrly the above can be made into an alias. On the jetson, the current alias for the above is "kms".

## 4. Verify that the rover is publishing necessary datapoints

Do not start navigation until these commands show steady data:

```bash
ros2 topic hz /imu/data
ros2 topic hz /gps/fix
ros2 topic hz /zed/zed_node/odom
ros2 topic hz /zed/zed_node/point_cloud/cloud_registered
ros2 topic hz /diff_drive_controller/odom
ros2 topic hz /odometry/local
ros2 topic hz /odometry/global
ros2 topic hz /odometry/gps
```

The required transform chain must also exist:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link zed_camera_link
```

The full transform chain is:

```text
map -> odom -> base_link -> chassis -> sensor frames
```

GPS may not produce a useful fix indoors. Test GPS navigation outdoors with a clear view of the sky.

## 5. Start GPS navigation

Open a second sourced terminal. Latitude and longitude are required:

```bash
ros2 launch rover_navigation navigation_launch.py \
  lat:=39.000000 \
  lon:=-76.000000 \
  heading:=0.0 \
  use_sim_time:=false
```

Replace the example coordinates with the real target. `heading` is the final map heading in degrees: east is `0`, north is `90`, west is `180`, and south is `-90`.
Set heading:=0.0 if you do not care about the rover's final heading orientation. 

The navigation launch will:

1. convert the GPS target to the `map` frame;
2. start Nav2 and its two rolling costmaps;
3. plan toward the target;
4. avoid obstacles seen in the ZED point cloud;
5. publish wheel commands on `/diff_drive_controller/cmd_vel_unstamped`.

## What should be published?

| Topic | Message | Source |
| --- | --- | --- |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | ICM-20948 driver |
| `/imu/mag` | `sensor_msgs/msg/MagneticField` | ICM-20948 driver |
| `/imu/data` | `sensor_msgs/msg/Imu` | Madgwick filter |
| `/gps/fix` | `sensor_msgs/msg/NavSatFix` | L76X driver |
| `/zed/zed_node/odom` | `nav_msgs/msg/Odometry` | ZED wrapper |
| `/zed/zed_node/point_cloud/cloud_registered` | `sensor_msgs/msg/PointCloud2` | ZED wrapper |
| `/diff_drive_controller/odom` | `nav_msgs/msg/Odometry` | Physical wheel controller |
| `/odometry/local` | `nav_msgs/msg/Odometry` | Local EKF |
| `/odometry/global` | `nav_msgs/msg/Odometry` | Global EKF |
| `/odometry/gps` | `nav_msgs/msg/Odometry` | `navsat_transform` |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | Nav2 |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | Nav2 |
| `/diff_drive_controller/cmd_vel_unstamped` | `geometry_msgs/msg/Twist` | Nav2 |

To view the rover in RViz, use `map` as the Fixed Frame. Add RobotModel, TF, both costmap topics, the ZED point cloud, and the `/plan` path.

## Docker development image

The Docker image contains ROS 2 Humble and the dependencies used by this workspace:

```bash
docker build -t autonomous-path-planning:humble .
docker run --rm -it \
  --mount type=bind,source="$(pwd)",target=/ros2_ws \
  autonomous-path-planning:humble
```

The image does not contain the proprietary ZED SDK, CUDA runtime, NVIDIA device setup, or physical hardware permissions. Use a ZED/CUDA-compatible base image or the configured rover host for live-camera operation.
