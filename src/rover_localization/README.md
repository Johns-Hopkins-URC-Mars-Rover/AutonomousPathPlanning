# Rover localization

This package is part of the rover's localization pipeline. Its job is to turn camera motion and one **joint IMU stream** into a continuous estimate of the rover's local position, heading, and velocity. The estimator will be the existing `robot_localization` EKF; this package supplies its configuration, launch setup, sensor-frame connections, and checks. The ZED driver stays in `src/zed-ros2-wrapper`.

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

"ZED IMU off" means disabling **the ZED's internal IMU fusion in its positional tracking**, so its odometry is optical-only. The ZED IMU data still needs to be published for the separate two-IMU fusion work. The local EKF consumes that team's *one* joint IMU output, not the two raw IMU topics individually. We will compare this design with ZED's built-in visual-inertial tracking later if tracking quality warrants it.

| Stage                           | Inputs                                                            | Outputs                                                                                                       | Owner / notes                                                                                                                                                                                                 |
| ------------------------------- | ----------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ZED driver and optical tracking | Stereo camera frames                                              | `nav_msgs/Odometry` in a local odometry frame; image, depth, and camera calibration topics remain available | Existing ZED ROS 2 wrapper. Enable positional tracking and disable its internal IMU fusion in rover launch/config; do not edit the wrapper's source. Actual topic prefix depends on the camera configuration. |
| Two-IMU fusion                  | ZED IMU and second IMU                                            | One timestamped `sensor_msgs/Imu` topic with a frame ID and meaningful covariance                           | Separate team. This package treats it as an input contract. Orientation is optional until its heading reference and quality are known.                                                                        |
| Local state estimation          | ZED optical odometry and joint IMU                                | `nav_msgs/Odometry` on a rover-owned topic such as `/odometry/local`, plus one `odom -> base_link` TF   | This package launches `robot_localization`'s EKF. Its configuration selects only trustworthy fields from each input.                                                                                        |
| SLAM/mapping (later)            | ZED images, depth, camera calibration, and local odometry         | Map and map-relative rover pose                                                                               | A separate mapping stage; raw images and depth do not go directly into this EKF. Exactly one system must publish `map -> odom`.                                                                             |
| Wheel odometry (later)          | Encoder measurements                                              | `nav_msgs/Odometry` or a twist measurement                                                                  | Add as another local EKF input when a real publisher exists.                                                                                                                                                  |
| GNSS (later)                    | `sensor_msgs/NavSatFix`, local odometry, and a suitable heading | Geographic/global position estimate                                                                           | Use `navsat_transform_node` and a global localization design; keep the local `odom -> base_link` estimate continuous.                                                                                     |

The local `odom` frame starts near the rover and can drift over time. A future `map` frame can correct long-term drift. The camera and joint IMU must have known, fixed transforms to `base_link`, and only the local EKF should publish `odom -> base_link` in this design. ZED odometry must not publish a competing transform. Timestamps, coordinate conventions, and covariance are part of each input contract.

## Hardware-free test we will add

We can test the localization wiring without a camera or either physical IMU. A simulator will follow a known path and publish two **synthetic** measurements with the same ROS message types and topic contracts as the real inputs:

1. A fake ZED optical `nav_msgs/Odometry` message with position/orientation error and realistic covariance.
2. A fake joint `sensor_msgs/Imu` message with angular velocity, and orientation only if the real joint stream will provide it.
3. A separate ground-truth `nav_msgs/Odometry` message used **only for comparison**, never fused into the EKF.

A simulation launch file will start that publisher and the *same local EKF configuration* used by the real pipeline, changing topic remaps only if necessary. Checks will confirm that `/odometry/local` and `odom -> base_link` exist, move in the expected direction, and remain finite and continuous. We will compare the estimated path with ground truth, then test camera and IMU dropouts separately. This verifies message wiring, TF ownership, filter configuration, and basic failure behavior; it does not prove real-world accuracy or calibrate the physical sensors. No synthetic image or depth generator is needed to test the EKF because its camera input is odometry, not pixels.

## What exists today

- `src/simulated_sensors.cpp` publishes synthetic **wheel** velocity on `/wheel/odometry`, synthetic yaw rate on `/imu/data`, and `/simulation/ground_truth`. It is useful as an early test fixture but does not yet simulate ZED odometry or the joint IMU contract.
- `config/ekf_local.yaml` names those simulation topics. It is not installed or launched by this package yet, and no EKF executable is implemented here.
- There is no real-sensor localization launch file, sensor-frame description, or hardware-free test for the agreed ZED + joint-IMU pipeline yet.

The next implementation step is to add the real and simulation launch/configuration, adapt the simulator to the agreed input contracts, and validate the local output. Wheel and GNSS integration can follow when those streams exist.
