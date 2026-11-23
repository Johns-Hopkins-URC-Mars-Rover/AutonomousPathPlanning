# Autonomous Path Planning

ROS 2 Humble software for GPS navigation of the Johns Hopkins Mars Rover.
The same localization and Nav2 configuration is used on the physical rover and
in Gazebo. Gazebo replaces the hardware drivers, but keeps their public topics.

## Main data flow

```text
GPS + IMU + ZED + wheel odometry
              |
              v
  local EKF + global EKF + navsat_transform
              |
              v
       map -> odom -> base_link
              |
              v
  Nav2 costmaps, planner, and controller
              |
              v
/diff_drive_controller/cmd_vel_unstamped
```

Nav2 creates both costmaps. Do not launch the archived `rover_costmap` package.

## Build

Ubuntu 22.04 and ROS 2 Humble are required.

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run the physical rover

Start the hardware, robot transforms, calibrated IMU orientation, and dual EKFs:

```bash
ros2 launch rover_bringup rover.launch.py \
  enable_imu:=true \
  enable_gps:=true \
  enable_zed:=true \
  enable_robot_state_publisher:=true \
  enable_localization:=true \
  use_sim_time:=false
```

Then provide a required GPS target:

```bash
ros2 launch rover_navigation navigation_launch.py \
  lat:=<target_latitude> \
  lon:=<target_longitude> \
  heading:=0.0 \
  use_sim_time:=false
```

`heading` is the desired final map yaw in degrees: east is `0`, north is `90`.

## Run the Gazebo rover

The simulation uses `simp_rover.urdf` to reduce computation. Its GPS, IMU, ZED,
wheel controller, sensor mounts, topic names, localization, and Nav2 parameters
match the physical rover interfaces.

Start Gazebo and localization:

```bash
ros2 launch rover_description simulation.launch.py gui:=true
```

The default world is `park.world`, which uses `park_fixed.tif` as its terrain
heightmap. For a flat debugging world, pass
`world:=$(ros2 pkg prefix rover_description)/share/rover_description/world/flat.world`.

Then use the same GPS navigation launch, with simulation time enabled:

```bash
ros2 launch rover_navigation navigation_launch.py \
  lat:=<target_latitude> \
  lon:=<target_longitude> \
  heading:=0.0 \
  use_sim_time:=true
```

For a headless run, omit `gui:=true`. To start Nav2 without the GPS client, use
`enable_nav2:=true` on `simulation.launch.py`.

### Docker on macOS

Start Docker Desktop, build the image once, and verify the environment:

```bash
bash scripts/run_simulation_container.sh --build bash -lc \
  'source /opt/ros/humble/setup.bash; python3 /source/docker/gazebo_environment_smoke.py'
```

Run all package and headless Gazebo integration tests with:

```bash
bash scripts/test_simulation_container.sh
```

## What must be publishing

| Topic | Type | Purpose |
| --- | --- | --- |
| `/gps/fix` | `sensor_msgs/msg/NavSatFix` | GPS fix |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | Raw ICM-compatible IMU |
| `/imu/mag` | `sensor_msgs/msg/MagneticField` | Magnetometer |
| `/imu/data` | `sensor_msgs/msg/Imu` | Madgwick orientation used by localization |
| `/zed/zed_node/odom` | `nav_msgs/msg/Odometry` | ZED-compatible visual odometry |
| `/zed/zed_node/point_cloud/cloud_registered` | `sensor_msgs/msg/PointCloud2` | Costmap obstacle input |
| `/diff_drive_controller/odom` | `nav_msgs/msg/Odometry` | Wheel odometry |
| `/odometry/local` | `nav_msgs/msg/Odometry` | Local EKF output |
| `/odometry/global` | `nav_msgs/msg/Odometry` | Global EKF output |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | Local Nav2 costmap |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | Global Nav2 costmap |
| `/diff_drive_controller/cmd_vel_unstamped` | `geometry_msgs/msg/Twist` | Final drive command |

The expected TF chain is:

```text
map -> odom -> base_link -> chassis -> sensor frames
```

The global EKF owns `map -> odom`, the local EKF owns `odom -> base_link`, and
`robot_state_publisher` owns the fixed rover and sensor transforms.

## Quick checks

```bash
ros2 topic hz /odometry/local
ros2 topic hz /odometry/global
ros2 topic hz /zed/zed_node/point_cloud/cloud_registered
ros2 run tf2_ros tf2_echo map base_link
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
```

The physical ZED requires the initialized `src/zed-ros2-wrapper` submodule, the
ZED SDK, CUDA, and supported hardware. These are not needed by Gazebo.

More detail is available in the repository wiki and in
[`docs/SIMULATION_INTEGRATION_PLAN.md`](docs/SIMULATION_INTEGRATION_PLAN.md).
