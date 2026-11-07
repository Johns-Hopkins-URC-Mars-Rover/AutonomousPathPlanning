# Rover localization

This package configures `robot_localization`; it does not implement a new motion
estimator. The [testing workflow](TESTING.md) contains build, Jetson/ROS-machine
commands, recording, measured-motion checks and GPS prerequisites.

```text
Jetson: ZED images -> optical odometry -----------------------------+
        ZED IMU -> optional two-IMU filter -> one IMU stream --------+--> input validation
ROS PC: validated camera pose -> explicit camera-to-base conversion +--> local EKF
                                                                       /odometry/local
                                                                       odom -> base_link

L76X RMC + GGA -> /gps/fix -> GPS validation -----------------------+
verified global heading -> heading validation ---------------------+--> navsat_transform
/odometry/global -------------------------------------------------+    /odometry/gps

local body-frame motion + verified heading + /odometry/gps -> global EKF
                                                            /odometry/global
                                                            map -> odom
```

The global path is optional and disabled until mounts, global heading and GPS
uncertainty are verified. GPS corrections do not enter the continuous local EKF.
For the unmounted camera bench, explicitly select `base_frame:=zed_camera_link`;
the same local EKF then publishes `odom -> zed_camera_link`.

| Component | Inputs | Outputs / ownership |
| --- | --- | --- |
| `zed_driver.launch.py` | Camera and SDK | Optical-only ZED odometry; images/depth; IMU; calibrated internal TF bridge. No motion TF. |
| `sensor_inputs.py` local mode | Raw ZED odometry and one IMU stream | Validated camera pose converted to the configured body frame on `/localization/zed_odom_base`; gyro input on `/localization/imu`; acceptance/freshness diagnostics. |
| Local EKF | Validated camera xyz/roll/pitch/yaw and IMU angular velocity | `/odometry/local`, sole `odom -> body` transform. IMU acceleration/orientation remain disabled pending validation. |
| `rover_description` | Measured mounts | `base_link -> camera/GPS/ICM`; one mount authority. Never invent the internal ZED IMU transform. |
| `sensor_inputs.py` GPS mode | `/gps/fix` and verified earth-referenced `/imu/heading` | `/gps/validated`, `/localization/heading`, diagnostics. Unknown covariance/altitude, bad times and missing TF are rejected. |
| navsat + global EKF | Validated GPS/heading and local motion | GPS odometry; `/odometry/global`; sole `map -> odom`. |

The default raw joint IMU topic is `/imu/data`, matching `rover_imu_fusion`.
Override `fused_imu_topic` for another stream such as `/imu/fused` or the direct
ZED IMU. A stable topic alone is insufficient: frame, timestamps, units and
covariance must satisfy the input contract.

`localization.launch.py` runs only local estimation and validation;
`zed_localization.launch.py` adds the camera driver for a single-machine setup.
For two machines, use `rover_bringup` on the Jetson and
`rover_description/gps_nav_launch.py` on the ROS PC. That combined workflow
starts exactly one local EKF and optionally includes GPS/global localization
and Nav2. Do not start another local launch beside it.

The camera pose adapter composes `T_odom_camera * T_camera_body` and propagates
small-angle pose covariance, including the mounting lever arm. It does not
reuse camera twist. Measured mount uncertainty is not modeled. Both EKFs retain
3D mode; a planar synthetic trajectory does not prove 3D hardware behavior.

`sim_localization.launch.py` generates a known planar path, noisy camera pose
and gyro-only IMU on production-compatible topics (`/zed/zed_node/odom`,
`/imu/fused`) and truth on `/simulation/ground_truth`. It uses the same validated
local path and EKF. It is separate from Gazebo and from the two-IMU filter's
accelerometer-dependent initialization.

Images/depth belong to perception/mapping, not directly to the EKF. A future
mapping system must coordinate ownership of `map -> odom` with the GPS global
EKF. Wheel odometry remains an additional future input to the live camera path.
Upstream ZED and u-blox code is unchanged.

Source and native regression checks are documented in
[the repair evidence](../../docs/testing/localization-repair.tdd.md). Real
tracking quality, calibrated mounts, timing, GPS accuracy and the earlier 17 m
error still require the recorded hardware test described in TESTING.md.
