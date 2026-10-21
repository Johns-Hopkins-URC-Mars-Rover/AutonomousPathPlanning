# rover_imu_fusion

The rover's IMU path. At this stage it is a single-IMU pass-through: the ZED
camera's IMU is rotated into the rover body frame and published on `/imu/data`.
There is no filter yet and the ICM-20948 is not used.

```
ZED SDK -> /zed/zed_node/imu/data  (frame zed_imu_link)
                  |
            zed_imu_node  (this package, rotates by TF base_link <- zed_imu_link)
                  |
              /imu/data  (frame base_link)  ->  rover_localization ekf_local (imu0)
```

`/imu/data` is the stable output of this package. A later error-state EKF
(both IMUs, biases, honest covariances) will publish on the same topic, so
consumers do not need to change.

## What the node does

For every `/zed/zed_node/imu/data` message:

| Field | Treatment |
| --- | --- |
| `header.stamp` | Kept as published by the ZED. |
| `header.frame_id` | Replaced with `base_frame` (default `base_link`). |
| `angular_velocity`, `linear_acceleration` | Rotated into `base_link` with the rotation of the TF `base_link <- zed_imu_link`. Lever-arm terms (centripetal, tangential) are ignored. |
| `*_covariance` | If the ZED covariance is valid, rotated as `R C R^T`. If it is missing, see "Covariances" below. |
| `orientation` | The ZED's own estimate, re-expressed for `base_link`. Forwarded only with a valid covariance, otherwise flagged unavailable (`orientation_covariance[0] = -1`). Its yaw is relative to wherever the ZED started, not north. |

Values follow REP-145: angular velocity in rad/s, `linear_acceleration` in
m/s^2 and **includes gravity**. The ZED values are already bias corrected by
the ZED SDK.

The TF rotation is looked up once per frame and cached. Until it is available
the node publishes nothing and logs `Waiting for TF`.

## Covariances

The node never invents noise values.

- Valid ZED covariance: rotated and forwarded.
- Missing covariance (all zero, `-1` in element 0, not finite, not symmetric,
  not positive semi-definite): the message carries zeros, which means
  "unknown", and the node warns once.
- Optional fallback: set `angular_velocity_variance` and/or
  `linear_acceleration_variance` (3 positive diagonal values) in
  `config/zed_imu.yaml`. They are used only when the ZED covariance is missing.

Check what the ZED actually publishes before relying on it:
`ros2 topic echo /zed/zed_node/imu/data --once`

## Topics and parameters

| Name | Default | Notes |
| --- | --- | --- |
| `input_topic` | `/zed/zed_node/imu/data` | Subscribed with sensor-data QoS. |
| `output_topic` | `/imu/data` | Reliable QoS, depth 10. |
| `base_frame` | `base_link` | Must match the root of the ZED URDF. |
| `angular_velocity_variance` | unset | Optional fallback, (rad/s)^2. |
| `linear_acceleration_variance` | unset | Optional fallback, (m/s^2)^2. |

## Prerequisites on the rover

- The ZED wrapper is running, so `/tf` contains `base_link -> ... ->
  zed_imu_link`. The ZED URDF comes from the `zed_description` package, which
  is not in this repository: confirm that its root is `base_link`
  (`ros2 run tf2_ros tf2_echo base_link zed_imu_link`).
- The ZED only publishes its IMU topic while something subscribes. This node
  is that subscriber.
- Nothing else may publish `/imu/data`. In simulation,
  `rover_localization/simulated_sensors` currently does; move it before running
  both.

## Build, test, run

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rover_imu_fusion
colcon test --packages-select rover_imu_fusion --event-handlers console_direct+
colcon test-result --verbose

source install/setup.bash
ros2 launch rover_imu_fusion zed_imu.launch.py
```

Check the output:

```bash
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
```

At rest with the rover level, `linear_acceleration` should read about 9.8 on
the base-frame up axis, and `angular_velocity` close to zero.

## Layout

```
include/rover_imu_fusion/imu_transform.hpp   ROS-free helpers: covariance checks, rotation
src/imu_transform.cpp
src/zed_imu_node.cpp                         ROS node around the helpers
config/zed_imu.yaml
launch/zed_imu.launch.py
test/test_imu_transform.cpp
```

## Not done yet

- ICM-20948 gyro and accel fusion (the driver is unchanged and publishes zero
  covariances, so it would need noise parameters first).
- The error-state EKF, bias estimation, and an orientation with an honest,
  growing yaw covariance.
- Diagnostics on `/diagnostics`.
- Moving the simulator's IMU off `/imu/data`.
