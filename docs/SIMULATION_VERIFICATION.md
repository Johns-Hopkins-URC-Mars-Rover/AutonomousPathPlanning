# Simulation verification

Verified on November 22, 2026 with ROS 2 Humble and Gazebo Classic 11 in the
repository's Linux AMD64 Docker image. The host was macOS, so Gazebo ran
headlessly through `xvfb` and Docker emulation.

## Command

```bash
bash scripts/test_simulation_container.sh
```

The script performs a clean package build, runs the package tests, and starts
fresh Gazebo processes for motion and navigation profiles.

## Package results

- 5 active packages built successfully.
- 97 tests passed.
- 0 failures, errors, or skipped tests.
- The full and simplified URDF sensor-frame transforms matched.
- Required simulation topic contracts and wheel-axis conventions passed static
  checks.

## Gazebo motion profile

The probe received `/clock`, GPS, raw IMU, magnetic field, fused IMU, ZED point
cloud, ZED-compatible odometry, wheel odometry, both EKF outputs, and independent
ground truth. It also verified `map -> odom -> base_link` and the ZED frame tree.

Measured on the final run:

| Check | Result |
| --- | ---: |
| GPS conversion vs. global pose | 0.53 m error |
| Positive `linear.x` | 0.31 m forward |
| Negative `linear.x` | 0.28 m backward |
| Positive `angular.z` | 0.70 rad counter-clockwise |
| Local-EKF vs. truth motion delta | 0.052 m error |

No command or odometry sign relay was active.

## Gazebo Nav2 profile

The second clean launch verified:

- all sensor/localization streams and the connected TF tree;
- GPS conversion within 0.75 m of the global pose;
- a physical box spawned 2.5 m ahead changed its local-costmap cell from `0`
  to lethal cost `99` using the ZED-compatible point cloud;
- a reachable forward Nav2 goal completed successfully;
- an active goal acknowledged cancellation;
- a deliberately invalid goal outside the rolling global costmap failed as
  expected.

The shared Nav2 configuration sets a 100 ms behavior-tree loop and a 1000 ms
action/service server timeout. This prevents false server timeouts when depth,
costmap, localization, and control callbacks briefly load the computer.

## What this proves

The simulated hardware boundary matches the physical rover's public topics and
frames, and the same EKF/navsat/Nav2/navigation-client architecture can run in
Gazebo. It also proves the configured command signs, costmap observation path,
basic planning/control, and action behavior in the flat test world.

It does not prove real-world traction, ZED visual-odometry accuracy, GPS error,
camera latency, magnetic interference, or obstacle performance on slopes. The
simulated `/zed/zed_node/odom` is noisy Gazebo pose truth and
`/simulation/ground_truth` is never used by localization.
