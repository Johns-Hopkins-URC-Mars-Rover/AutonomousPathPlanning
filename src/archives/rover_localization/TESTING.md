# Localization: next tests

The noisy baseline works in synthetic tests. **The original large position error
is not confirmed fixed:** confidently wrong camera poses still pull the EKF far
from truth. GPS fusion remains disabled until mounts, heading and uncertainty
are verified. Follow this document; no other localization test guide is needed.

## 1. Run now on your non-ROS machine

From the repository root, with Docker Desktop running:

```bash
bash scripts/test_localization_container.sh
```

This builds ROS Humble inside Docker and runs the real production local EKF.
The host needs no ROS, camera, GPU or ZED SDK. The first build downloads
packages; the test container runs unprivileged, without external networking,
with read-only source and temporary build/log storage.

Two suites run: a short noisy-circle smoke test and three longer scenarios
(two noisy baseline seeds, one tracking-failure/recovery case). Noise is seeded;
real ROS scheduling means exact errors/sample counts vary between runs.

| Synthetic impairment | Assumed magnitude |
| --- | --- |
| Camera white noise | 3.5 cm standard deviation per axis; orientation 0.006/0.006/0.015 rad |
| Camera drift/outlier | Random walk; 4 mm/s vertical drift; one (+0.25, -0.15, +0.35) m outlier |
| Gyro | 0.006 rad/s noise; bias and random walk |
| Transport | Camera delay 0–80 ms; gyro delay 0–15 ms; original acquisition stamps retained |
| Loss | Camera 1 s; gyro 1.2 s; both sensors 1.7 s, then recovery |
| Invalid timestamps | One stale and one future camera stamp; transport can reorder messages |
| Tracking failure | Incorrect camera xyz near (-7.11, 0.15, -15.41) m for two seconds |

Known nonzero synthetic mounts exercise frame conversion. Ground truth is used
only for generation/evaluation and is never fused. Noise assumptions are not
measured ZED calibration; covariances model white noise, deliberately omitting
unmodeled drift/outlier uncertainty. GPS is not part of this benchmark.

Expect timestamp-rejection warnings. Baselines must meet fixture limits of
0.5 m position RMSE and 1.5 m peak error; those are regression bounds, not rover
accuracy requirements. Simultaneous loss must report stale camera and gyro,
then observe fresh inputs after recovery. EKF publication can continue during
loss as prediction; a 30 Hz output alone does not prove healthy fusion.

**The tracking-fault case passes by reproducing a large error and then recovering.**
Read the final `KNOWN LIMIT` message even when all tests pass. This case is not
an outlier-rejection guarantee. The printed `NOISY_FUSION_RESULT` records errors,
input counters and outage/recovery evidence. To keep a run log locally:

```bash
bash scripts/test_localization_container.sh > /tmp/localization-test.log 2>&1
```

Additional ROS-free checks require Python 3, Ruby, a C++17 compiler and Eigen:

```bash
python3 -B scripts/check_localization.py --coverage
python3 -B scripts/benchmark_fusion.py --report /tmp/imu-motion.json
```

The first checks source/configuration, sensor/GPS contracts, native IMU safety,
CSV replay and motion math. The second runs the actual IMU fusion C++ core on
noisy rotations, bias and sensor loss. It evaluates orientation/rate, not position.
Neither command installs dependencies or modifies hardware data/tuning.

## 2. Next hardware test: record the failure before tuning

Use your **Mac as SSH terminals**. The **Jetson runs the ZED/L76X drivers, local
EKF and recording**, using its installed ROS Humble, ZED SDK/wrapper and rover
packages. Your Mac needs neither ROS nor Docker for this hardware test. The
Docker command in section 1 is a separate synthetic test with no external
network; it does not receive the real camera stream.

Connect the Mac and Jetson over a reachable network. Open a Mac Terminal tab and
SSH into the Jetson (replace the IP address):

```bash
ssh jhurover@<JETSON_IP>
```

After login, commands execute on the Jetson. Build once in its repository:

```bash
cd ~/ws/AutonomousPathPlanning
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=35
colcon build --symlink-install --packages-select rover_localization rover_description rover_imu_fusion rover_bringup l76x_gps_interface
source install/setup.bash
```

Adjust the repository path if your Jetson checkout is elsewhere. Open four Mac
tabs (A–D) and SSH into the same Jetson in each. In **every SSH session**, run:

```bash
cd ~/ws/AutonomousPathPlanning
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=35
```

All commands below run inside these Jetson SSH sessions. Bags and other files
are saved on the Jetson, not the Mac. Keep the tabs open while their processes
run. Stop earlier fake publishers and temporary identity transforms; use exactly
one driver/estimator per stream. Check the Jetson clock because sensor timestamp
validation still matters even with every node on one machine.

For this unmounted handheld bench, the body frame is `zed_camera_link`; no rover
mounting transform is invented. The calibrated internal camera-to-IMU transform
is still required. The Jetson must have enough CPU/GPU, memory and disk for the
driver, estimator and bag recording; watch for dropped data or stale diagnostics.

1. **Mac tab A → Jetson SSH:** start the real camera, its IMU and GPS:

   ```bash
   ros2 launch rover_bringup rover.launch.py camera_model:=zed2i enable_imu:=false enable_fusion:=false enable_gps:=true
   ```

2. **Mac tab B → Jetson SSH:** verify calibrated TF, then Ctrl-C:

   ```bash
   ros2 run tf2_ros tf2_echo zed_camera_link zed_imu_link
   ```

   If missing, inspect `/zed/zed_node/left_cam_imu_transform`, the camera URDF and
   calibration-bridge warnings. Never replace this with a zero transform. Check
   `pos_tracking.imu_fusion=false`, `pos_tracking.publish_tf=false`, and
   `sensors.publish_cam_imu_transf=true` on `/zed/zed_node`. The wrapper must
   support the calibration-message option; inspect its declared parameters.

3. **The same SSH session in tab B:** run one local EKF using the ZED gyro directly:

   ```bash
   ros2 launch rover_description gps_nav_launch.py base_frame:=zed_camera_link fused_imu_topic:=/zed/zed_node/imu/data
   ```

   Output is `/odometry/local` and the sole `odom -> zed_camera_link` TF.
   GPS/global fusion and Nav2 stay disabled. Later, the measured rover rig can
   use `base_link` and the default joint IMU topic `/imu/data`.

4. **Mac tab C → Jetson SSH:** record before moving; use a new output directory
   each run. Ctrl-C closes the recording:

   ```bash
   ros2 bag record -o localization_bench /zed/zed_node/odom /zed/zed_node/imu/data /zed/zed_node/left_cam_imu_transform /localization/zed_odom_base /localization/imu /odometry/local /gps/fix /diagnostics /tf /tf_static
   ```

   Inspect `ros2 topic list -t` and add available ZED tracking-status topics;
   their names/types vary by wrapper version. Save `ros2 param dump /zed/zed_node`
   and `ros2 param dump /ekf_local` plus Jetson clock checks and package versions.

5. **Mac tab D → Jetson SSH:** inspect diagnostics, then Ctrl-C before inspecting
   position:

   ```bash
   ros2 topic echo /diagnostics
   # Press Ctrl-C, then run:
   ros2 topic echo /odometry/local --field pose.pose.position
   ```

   Hold still 20 s; move a measured 0.30 m straight; stop 10 s; return; rotate
   a measured 90 degrees. Repeat and note movement times. Ordinary SSH displays
   terminal output; it does not automatically display RViz on your Mac. RViz is
   optional for this test. If using a Jetson display or a separately configured
   remote desktop, set Fixed Frame `odom` and add TF + Odometry `/odometry/local`.

6. Compare raw camera, converted camera and EKF at matching timestamps in
   meters. During rest, drift/velocity should be small; translation and rotation
   should follow measured movement. Set acceptance tolerances from measured
   sensor behavior. If raw camera jumps, investigate tracking first; if only
   converted pose jumps, inspect TF; if only EKF jumps, inspect updates,
   covariance and timing. Do not mask the failure by forcing 2D, retuning blindly
   or judging one terminal sample. Preserve the bag for replay without drivers.

For replay use `--clock` and `use_sim_time:=true` throughout; select raw sensor
and calibration/TF topics only. Do not replay old EKF outputs beside a new EKF
with the same output topics. CSV IMU replay alone cannot reproduce this failure.

## 3. Add L76X fusion only after verification

Record `/gps/fix` now. It is WGS84 latitude/longitude in degrees and ellipsoid
altitude in meters (paired RMC/GGA; MSL altitude + geoid separation). Missing
altitude or uncertainty remains unknown and is rejected for fusion.

Before enabling global localization:

- Measure rigid rover camera/GPS/IMU mounts. Fill the non-runnable template
  `rover_description/config/sensor_mounts.example.json`, mark it verified and
  launch `sensor_mounts.launch.py mounts_file:=/absolute/path/to/measured.json`.
  Use one TF authority; URDF simulation origins are not hardware measurements.
- Configure measured ENU GPS variance in m² using `position_variance` in
  `l76x_gps_interface/config/gps_params.yaml`. Stationary scatter alone does not
  quantify absolute bias or multipath.
- Supply independently verified ENU heading on `/imu/heading`, yaw zero toward
  true east, with mount/declination corrections. Startup-relative ZED/IMU yaw
  does not provide global heading.

Then, alongside the existing local EKF:

```bash
ros2 launch rover_localization gps_localization.launch.py enable_gps_fusion:=true heading_verified:=true mounts_verified:=true
```

Do not set verification flags to bypass missing measurements. GPS enters the
optional global EKF (`/odometry/global`, sole `map -> odom`), not the local EKF.
Test GPS loss separately; continued prediction does not prove GPS accuracy.

## Verification evidence

Regression tests first reproduced cached publisher health and acceptance of
invalid benchmark samples. Fixes passed the same tests. The reproducible
commands above replace separate plan/evidence documents and generated logs.
Final run on 2026-11-07: 48 Python tests, Ruby configuration checks, native
safety/replay/motion checks and 10 Docker ROS checks passed. Helper line coverage
was 93.6–100% (not whole-repository coverage). Noisy baseline position RMSE was
0.074/0.064 m; both-input loss and recovery were observed. The tracking-fault
case still reached 18.48 m median fault error before recovering to 0.058 m RMSE.
Native IMU orientation RMSE was 0.428 degrees (ZED only) and 0.370 degrees
(two IMUs then dropout). These are synthetic results, not hardware accuracy.
The original ZED failure still needs the recorded hardware test in section 2.
