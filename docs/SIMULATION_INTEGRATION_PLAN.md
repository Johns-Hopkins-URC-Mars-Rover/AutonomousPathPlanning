# Physical-to-simulation integration contract

The simulation must replace hardware, not replace the rover architecture. The
dual EKFs, `navsat_transform`, Nav2 parameters, navigation client, frames, and
public topics remain the same as the field-tested physical stack.

## Model choice

Gazebo loads `simp_rover.urdf` because the full CAD model is expensive to
simulate. `rover.urdf` remains the physical/RViz model. Both URDFs define the
same GPS, ICM-20948, ZED, and ZED IMU links with the same parent, translation,
and rotation. Automated tests compare these transforms.

The simplified model intentionally differs only in body, suspension, collision,
and wheel geometry needed by Gazebo. Its six wheel axes follow the ROS command
convention directly; no simulation-only sign relay is used.

## Shared ROS boundary

| Interface | Physical producer | Gazebo producer | Consumer |
| --- | --- | --- | --- |
| `/gps/fix` | L76X driver | Gazebo GPS | `navsat_transform` |
| `/imu/data_raw` | ICM driver | Gazebo IMU | Madgwick and EKFs |
| `/imu/mag` | ICM driver | Idealized magnetic-field simulator | Madgwick |
| `/imu/data` | Madgwick | Madgwick | EKFs and `navsat_transform` |
| `/zed/zed_node/imu/data` | ZED wrapper | Gazebo ZED IMU | Diagnostics/future fusion |
| `/zed/zed_node/odom` | ZED wrapper | Noisy Gazebo pose plugin | EKFs |
| `/zed/zed_node/point_cloud/cloud_registered` | ZED wrapper | Gazebo depth camera | Nav2 voxel layers |
| `/diff_drive_controller/odom` | Wheel controller | `ros2_control` Gazebo controller | EKFs |
| `/diff_drive_controller/cmd_vel_unstamped` | Motor controller input | Gazebo controller input | Drive system |

The simulated magnetic field is generated from Gazebo attitude so the same
Madgwick processing stage exists in both environments. Simulation uses zero
declination, yaw offset, and hard-iron bias because its sensor frame is ideal.
The physical calibration remains isolated in `rover_bringup`.

## TF ownership

```text
map -> odom -> base_link -> chassis -> sensor frames
```

- Global EKF: `map -> odom`
- Local EKF: `odom -> base_link`
- `robot_state_publisher`: fixed body and sensor transforms
- Wheel controller: odometry message only; `enable_odom_tf` is false

`/simulation/ground_truth` is test evidence only and is never fused.

## Launch sequence

```bash
ros2 launch rover_description simulation.launch.py gui:=true
```

This starts Gazebo, simulated hardware topics, Madgwick, the physical dual-EKF
launch, and the state publisher. Nav2 is normally started through the same GPS
entry point used on the rover:

```bash
ros2 launch rover_navigation navigation_launch.py \
  lat:=<target_latitude> lon:=<target_longitude> \
  heading:=0.0 use_sim_time:=true
```

For direct Nav2 action tests, pass `enable_nav2:=true` to the simulation launch.

## Verification gates

1. Build and package tests pass in a clean ROS 2 Humble container.
2. Gazebo publishes every interface in the table above with simulation stamps.
3. The TF chain has one owner per dynamic transform.
4. Positive linear velocity moves physically forward; positive angular velocity
   turns counter-clockwise. Reverse remains negative linear velocity.
5. GPS conversion agrees with global localization within the declared tolerance.
6. A spawned obstacle appears in the Nav2 local costmap.
7. A reachable Nav2 goal succeeds and cancellation is acknowledged.

Run all automated checks with:

```bash
bash scripts/test_simulation_container.sh
```

Gazebo pose truth used for simulated ZED odometry is not a performance model of
real visual odometry. Passing simulation proves interface and autonomy behavior,
not real sensor accuracy, traction, latency, or calibration.
