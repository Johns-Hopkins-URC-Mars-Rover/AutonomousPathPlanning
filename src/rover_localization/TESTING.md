# Localization testing

This guide contains the ROS 2 test procedures for `rover_localization`.
They verify topic wiring, EKF output, and TF behavior. A passing synthetic
test does not validate physical sensor accuracy, timing, calibration, or
uncertainty on the rover.

## Prerequisites

Run these procedures on Ubuntu 22.04 with ROS 2 Humble. From the repository
root, build and source the package:

```bash
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-robot-localization
colcon build --symlink-install --packages-select rover_localization
source install/setup.bash
```

Every additional terminal must start in the repository root and source both
`/opt/ros/humble/setup.bash` and `install/setup.bash`.

## Hardware-free synthetic test

This requires ROS 2 Humble and `robot_localization`, but no camera or physical
IMU. The simulator follows a known planar path and publishes:

1. Fake ZED optical `nav_msgs/Odometry` at `/zed/zed_node/odom`, with
   deterministic measurement noise and covariance.
2. Fake joint `sensor_msgs/Imu` at `/imu/fused`, providing angular velocity.
   Orientation and acceleration are marked unavailable.
3. Ground truth at `/simulation/ground_truth`, used only for comparison and
   never fused into the EKF.

### Where the fake data lives

The fake data is generated live by
`src/rover_localization/src/simulated_sensors.cpp`; there is no bag file or
static sensor dataset. At 30 Hz, the node advances a known curved rover path
(default `0.5 m/s` forward and `0.2 rad/s` yaw rate), publishes the clean path
as ground truth, then adds deterministic seeded noise to make the fake ZED pose
and fake joint-IMU gyro measurements. The launch file
`launch/sim_localization.launch.py` connects those generated messages to the
same `localization.launch.py` EKF used by the live system.

The terminal output labels four stages: build, start fake data plus EKF, probe
the output against the clean path, and PASS. It also lists every topic and
explains that `/simulation/ground_truth` is comparison-only. A successful run
therefore proves message types, topic wiring, basic filter motion, approximate
agreement, and `odom -> base_link`; it does not prove physical sensor quality.

Run the complete check from the repository root:

```bash
bash src/rover_localization/scripts/smoke_test.sh
```

The script builds the package, launches synthetic sensors and the EKF, waits up
to 40 seconds for moving `/odometry/local`, compares it to
`/simulation/ground_truth`, checks `odom -> base_link`, prints PASS or FAIL,
and stops the launch process. It does not use a ZED or physical IMUs.

To inspect the simulation manually after building and sourcing, run each line
below in a separately sourced terminal:

```bash
ros2 launch rover_localization sim_localization.launch.py
ros2 topic echo /odometry/local --once
ros2 topic echo /simulation/ground_truth --once
ros2 run tf2_ros tf2_echo odom base_link
```

The estimated and true paths should move in the same direction and stay
reasonably close. The synthetic node exposes `camera_dropout_start`,
`camera_dropout_duration`, `imu_dropout_start`, and `imu_dropout_duration`
parameters for future fault-injection checks. It does not generate camera
images or depth because the EKF consumes ZED odometry, not pixels.

## Real ZED with a fake IMU: stationary bench test

Use this when the ZED is available but the joint IMU publisher is not. The
fixture at `config/fake_imu_stationary.yaml` is a `sensor_msgs/msg/Imu`
message, not an EKF parameter file. It publishes zero angular velocity in
`base_link` axes with nonzero test covariance; orientation and acceleration
are marked unavailable. It substitutes for the **joint IMU output**, not the
two raw IMUs or their fusion algorithm. No separate IMU mount TF is needed
because the fixture's frame is already `base_link`.

The ZED wrapper must already be built with its SDK/CUDA dependencies. Provide
the measured static `base_link -> zed_camera_link` mount transform through the
robot description or TF setup. Do not use a zero transform unless the frames
really coincide.

Keep the rover and camera stationary. Turning the camera causes the fake gyro
and real visual motion to disagree. For a moving test without hardware, use
the synthetic test above.

1. Terminal A: launch the ZED, EKF, and fake IMU together. Replace the model
   if required.

   ```bash
   ros2 launch rover_localization zed_fake_imu_localization.launch.py \
     camera_model:=zed2i
   ```

   The launch starts a publisher that emits the same stationary IMU contract as
   `config/fake_imu_stationary.yaml`, at 50 Hz with fresh timestamps. Run only
   one publisher on `/imu/fused`; stop this launch before switching to the real
   joint IMU. Do not launch `sim_localization.launch.py` with the real camera
   because it also publishes fake ZED odometry.

2. Terminal B: inspect inputs, output, and transform.

   ```bash
   ros2 topic echo /zed/zed_node/odom --once
   ros2 topic echo /imu/fused --once
   ros2 topic hz /imu/fused
   # Ctrl-C stops the rate check; then continue:
   ros2 topic echo /odometry/local --once
   ros2 run tf2_ros tf2_echo odom base_link
   ```

Expect an IMU rate near 50 Hz, advancing timestamps, local odometry output,
and a resolvable `odom -> base_link` transform. At rest, the pose should be
roughly steady, allowing for visual tracking noise. If output is missing,
check the actual camera topic (`ros2 topic list`), input frames/TF, and EKF
diagnostics. If the camera uses another topic, add
`zed_odom_topic:=/actual/topic` to the launch command.

If the camera is already running, configure it with the optical-only override
and motion TF disabled, then launch only the EKF:

```bash
ros2 launch rover_localization localization.launch.py \
  zed_odom_topic:=/zed/zed_node/odom
```

Use wall-clock time for this live-camera procedure. A bag replay using `/clock`
needs a separate, consistently configured simulation-time setup.

## Source-level checks

These checks work without ROS. They validate source/configuration contracts,
not the running ROS pipeline:

```bash
ruby src/rover_localization/test/test_localization_contract.rb
python3 -m unittest discover -s src/rover_localization/test -p test_smoke_probe.py -v
```
