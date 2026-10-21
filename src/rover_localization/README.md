# Rover localization

This package is Mariam's local localization pipeline. It combines camera motion and one **joint IMU stream** into a continuous estimate of the rover's local position, heading, and velocity. The estimator is the existing `robot_localization` EKF. The ZED driver remains in `src/zed-ros2-wrapper` and is not edited here.

## Files and ownership

```text
src/
├── zed-ros2-wrapper/                 upstream camera driver (left untouched)
├── icm20948_imu_interface/           one raw IMU driver; joint fusion is separate work
├── ublox_dgnss/                      future GNSS input
└── rover_localization/
    ├── config/ekf_local.yaml         local EKF measurement selection and frames
    ├── config/zed_optical.yaml       ZED optical tracking override; raw IMU stays on
    ├── launch/localization.launch.py EKF only; accepts existing sensor topics
    ├── launch/zed_localization.launch.py  ZED wrapper + EKF for a live rover
    ├── launch/sim_localization.launch.py  synthetic sensors + the same EKF
    ├── include/rover_localization/simulation_model.hpp  testable path model
    ├── src/simulated_sensors.cpp     synthetic odometry, joint IMU, and truth
    └── test/                        configuration and motion-model checks
```

## The planned path

```text
ZED images ──> ZED optical tracking ──> ZED odometry ──┐
ZED IMU ─┐                                              ├──> local EKF ──> local rover odometry
         ├──> upstream two-IMU fusion ──> joint IMU ────┘                 + odom -> base_link TF
other IMU┘

ZED images + depth + local rover odometry ──> future SLAM/mapping
future wheel odometry ──> additional local EKF input
future GNSS fix ──> coordinate conversion + global localization
```

"ZED IMU off" means disabling **the ZED's internal IMU fusion in its positional tracking**, so its odometry is optical-only. The ZED IMU data is still published for the separate two-IMU fusion work. The local EKF consumes that team's *one* joint IMU output, not the two raw IMU topics individually.

| Stage                           | Inputs                                                            | Outputs                                                                                                       | Owner / notes                                                                                                                                                                                                 |
| ------------------------------- | ----------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ZED driver and optical tracking | Stereo camera frames                                              | `nav_msgs/Odometry` in a local odometry frame; image, depth, and camera calibration topics remain available | Existing ZED ROS 2 wrapper. Enable positional tracking and disable its internal IMU fusion in rover launch/config; do not edit the wrapper's source. Actual topic prefix depends on the camera configuration. |
| Two-IMU fusion                  | ZED IMU and second IMU                                            | One timestamped `sensor_msgs/Imu` topic with a frame ID and meaningful covariance                           | Separate team. This package treats it as an input contract. Orientation is optional until its heading reference and quality are known.                                                                        |
| Local state estimation          | ZED optical odometry and joint IMU                                | `nav_msgs/Odometry` on `/odometry/local`, plus one `odom -> base_link` TF   | This package launches `robot_localization`'s EKF. The initial config fuses camera pose and IMU angular velocity; IMU orientation and acceleration await validation.                                                                                        |
| SLAM/mapping (later)            | ZED images, depth, camera calibration, and local odometry         | Map and map-relative rover pose                                                                               | A separate mapping stage; raw images and depth do not go directly into this EKF. Exactly one system must publish `map -> odom`.                                                                             |
| Wheel odometry (later)          | Encoder measurements                                              | `nav_msgs/Odometry` or a twist measurement                                                                  | Add as another local EKF input when a real publisher exists.                                                                                                                                                  |
| GNSS (later)                    | `sensor_msgs/NavSatFix`, local odometry, and a suitable heading | Geographic/global position estimate                                                                           | Use `navsat_transform_node` and a global localization design; keep the local `odom -> base_link` estimate continuous.                                                                                     |

The local `odom` frame starts near the rover and can drift over time. A future `map` frame can correct long-term drift. The camera and joint IMU must have known, fixed transforms to `base_link`, and only the local EKF should publish `odom -> base_link` in this design. ZED odometry must not publish a competing transform. Timestamps, coordinate conventions, and covariance are part of each input contract. The EKF currently uses 3D mode so it does not silently force a rover on uneven terrain to stay planar.

## Hardware-free simulation

The simulation launch needs ROS 2 Humble and `robot_localization`, but no camera or physical IMU. The simulator follows a known planar path and publishes:

1. Fake ZED optical `nav_msgs/Odometry` at `/zed/zed_node/odom`, with deterministic measurement noise and covariance.
2. Fake joint `sensor_msgs/Imu` at `/imu/fused`, providing angular velocity. It marks orientation and acceleration unavailable.
3. Ground truth at `/simulation/ground_truth`, used **only for comparison**, never fused into the EKF.

Build and run on a ROS 2 Humble machine:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rover_localization --symlink-install
source install/setup.bash
ros2 launch rover_localization sim_localization.launch.py
```

In separate sourced terminals, check the messages and transform:

```bash
ros2 topic echo /odometry/local --once
ros2 topic echo /simulation/ground_truth --once
ros2 run tf2_ros tf2_echo odom base_link
```

The estimated and true paths should move in the same direction and stay reasonably close. The synthetic sensor node has `camera_dropout_start`, `camera_dropout_duration`, `imu_dropout_start`, and `imu_dropout_duration` parameters for future failure checks. This simulation verifies wiring and basic filter behavior; it does not establish physical accuracy. It does not generate camera images or depth because the EKF consumes ZED odometry, not pixels.

## Live sensor bringup

The upstream IMU team must publish one `/imu/fused` `sensor_msgs/Imu` topic. The actual topic may be changed with `fused_imu_topic:=...`. The ZED model is a launch argument; `zed2i` is only the default example. The live launch applies `config/zed_optical.yaml` through the wrapper's parameter override path:

```bash
ros2 launch rover_localization zed_localization.launch.py camera_model:=zed2i
```

Before motion testing, provide the measured, static `base_link -> zed_camera_link` mount transform and `base_link -> <joint IMU frame>` transform from the rover's robot description or TF setup. The ZED wrapper can publish its own camera-internal fixed frames, but its motion TF is disabled here so it cannot compete with the EKF. Confirm the actual ZED odometry topic and joint IMU frame on the target machine; override `zed_odom_topic` if the wrapper namespace differs. The ZED SDK, CUDA, initialized wrapper submodule, and joint IMU publisher are required for this live launch.

## Current status and next checks

- The package now installs EKF configuration and three launch files. The synthetic publisher matches the two current input contracts.
- Local source/configuration checks pass. ROS runtime, camera operation, EKF output, and TF behavior still require an Ubuntu/ROS machine; they have not been claimed as validated from this macOS checkout.
- Before tuning, verify the real joint IMU's topic, rate, frame, angular-velocity covariance, and whether its orientation has a reliable heading. Check the ZED odometry covariance and tracking status during poor texture or rapid turns.

Wheel and GNSS inputs remain disabled until their publishers and frames are available. SLAM/mapping consumes the local output in a later stage.
