# Gazebo/localization Integration Work

Baseline: `1da6fe3`; branch: `fix/gazebo-localization-integration`.

## Completion Gates

1. Docker can run ROS 2 Humble and Gazebo Classic on this Mac, with evidence of a running simulation clock.
2. Active packages build from their installed resources, without archived packages or machine-specific source paths.
3. The rover spawns, controllers activate, sensors publish, and the local/global EKFs produce finite estimates with a connected TF tree.
4. Commanded movement, GPS conversion, navigation success/cancellation, and sensor dropout have executable checks.
5. Documentation describes the tested entry points and separates simulation evidence from unverified hardware behavior.

## Shared Contract

| Interface | Producer | Consumer |
| --- | --- | --- |
| `/clock` | Gazebo | All simulation nodes |
| `/diff_drive_controller/odom` | Wheel controller | Simulation sign adapter |
| `/simulation/wheel_odometry` | Simulation sign adapter | Local/global EKFs |
| `/zed/zed_node/imu/data` | Simulated IMU | EKFs and navsat |
| `/gps/fix` | Simulated GPS | navsat |
| `/odometry/gps` | navsat | Global EKF |
| `/odometry/local`, `odom -> base_link` | Local EKF | Nav2 and global TF chain |
| `/odometry/global`, `map -> odom` | Global EKF | navsat and Nav2 |
| `/depth/points` | Simulated depth camera | Nav2 costmaps |
| `/cmd_vel` | Nav2 velocity smoother | Relay |
| `/diff_drive_controller/cmd_vel_unstamped` | Relay | Wheel controller |

The URDF owns body/sensor transforms. No competing wheel-odometry TF publisher is allowed. Ground truth is only a test reference, never an estimator input. Simulated IMU orientation must be checked against Gazebo world axes and geographic conversion conventions. Simulation mounts are not measured hardware mounts.

## Sequence and Ownership

First, the environment agent establishes the Docker runtime and reports a usable image/command. Implementation starts only after this gate.

Then three implementation agents own disjoint areas:

- Model agent: `rover_description` launch/world/URDF/controllers, resource portability and Gazebo sensor behavior.
- Localization agent: `rover_localization`, `rover_bringup`, localization tests, intended hardware launch compatibility.
- Navigation/test agent: `rover_navigation`, Nav2 configuration, active test harness and end-to-end test scripts.

Root coordinates documentation, shared interfaces, review and final combined testing. Each agent must report its changed interfaces to the other two, cross-review their assumptions, and report exact commands/results. Root arbitrates shared files. Do not commit, push, merge or delete user data during delegated work.

## Verification Loops

For each defect: reproduce with a focused check, record the failure, apply a minimal fix, rerun that check, and then run affected integration checks. Stop after three unsuccessful attempts at the same blocker and report the evidence; choose a different supported approach only when its assumptions are explicit.

Validation proceeds through static resource/configuration checks, a clean container build, headless Gazebo smoke tests, stationary/straight/turn motion checks, TF/time checks, GPS conversion, localization dropout/recovery, and Nav2 reachable/unreachable goals and cancellation. Tolerances must be set before assessing results. Repeat startup to expose timing failures.

Record environment versions, actual test counts, logs and limitations. A launch process starting is not sufficient proof that sensors, localization or navigation work. GUI rendering, hardware accuracy and real sensor calibration need separate evidence.
