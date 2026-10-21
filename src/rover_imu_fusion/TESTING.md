# Testing the IMU fusion on the rover

How to record a stationary and a moving run, check them, and tune the filter
offline. Commands run from the repo root with the workspace built and sourced:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Quick way: one command

With the rover stack running (see Common setup), `scripts/run_imu_test.py` does
the countdown, records the CSV and the bag together, stops both cleanly, checks
them, and for a stationary run prints the analysis:

```bash
python3 scripts/run_imu_test.py stationary                       # 90 s
python3 scripts/run_imu_test.py moving --duration 90 --notes "2 slow turns, then figure 8"
```

Everything lands in `~/rover_data/<mode>_<timestamp>/` (`bag/`, `imu_compare.csv`,
`run_info.txt`, and `stationary.png` for a stationary run). Ctrl-C ends a run
early and still saves. For a moving run it prints GO after the 5 s still start.
Options: `--duration`, `--countdown`, `--out`, `--no-bag`, `--notes`. It exits
non-zero if the bag is unreadable or any of the icm, zed or fused rows are missing.

The manual steps below do the same thing by hand.

## Common setup

1. Start the rover stack (ZED and ICM drivers plus fusion):

   ```bash
   ros2 launch rover_bringup rover.launch.py camera_model:=<model>
   ```

   `enable_imu` defaults to true, which turns the ICM on in the fusion.
2. Check that `/imu/data` publishes and `/diagnostics` shows the filter
   initialized and both IMUs live:

   ```bash
   ros2 topic hz /imu/data
   ros2 topic echo /diagnostics --no-daemon
   ```

3. Keep the rover **still for the first 5 s of every run**. The filter needs a
   stationary start to initialize, and the analysis scripts drop the first 5 s.

Each run produces two recordings. The CSV is for analysis, the bag is for
replaying the node.

## Stationary run

Checks sensor noise against the published covariances and the fused yaw drift.

1. Put the rover on a flat surface and leave it alone. Do not touch the table.
2. In one terminal, log the CSV (4 minutes or more is plenty):

   ```bash
   python3 scripts/imu_compare.py --output results/stationary_<date>.csv
   ```

3. In another terminal, record the inputs:

   ```bash
   ros2 bag record -o ~/rover_data/bags/stationary_<date> \
     /imu/data_raw /zed/zed_node/imu/data /tf /tf_static
   ```

4. After 4 minutes, stop **both with Ctrl-C and wait for them to exit**. Do not
   kill the terminal or power off the rover first.
5. Check the bag is readable before leaving the rover:

   ```bash
   ros2 bag info ~/rover_data/bags/stationary_<date>
   ```

   It must list the topics with message counts. A bag with no `metadata.yaml`
   was not closed cleanly (the first one we recorded was unreadable for this
   reason).
6. Analyze:

   ```bash
   python3 scripts/analyze_stationary.py results/stationary_<date>.csv \
     --plot results/stationary_<date>.png
   ```

   The `var/cov` column should be near 1 for the raw sensors. For the fused
   output it is well below 1 when the process noise is too large. Gravity
   should read 9.807 m/s², and yaw drift should be under about 1 deg/min.

## Moving run

Checks that the filter follows real motion without lag. A stationary run cannot
show this.

1. Start the same logging and recording as above, with a name like `moving_<date>`.
2. Leave the rover still for 5 s, then drive a fixed routine, for example:
   - in-place turn left and right at slow and fast speed (note the peak rate)
   - straight drive with hard start and stop
   - a figure of eight
   - a few seconds of driving over rough ground, if the rover will see it
3. Stop with Ctrl-C and verify the bag with `ros2 bag info`.
4. Write down roughly what you did and when (a stopwatch is enough). It lets
   you match features in the plots to the routine.

## Tuning from a recording

`tools/replay_csv.cpp` feeds a CSV's ICM and ZED rows through the same filter
code as the node, with the parameters you choose. Build once:

```bash
cd src/rover_imu_fusion/tools
g++ -O2 -std=c++17 -I/usr/include/eigen3 -I../include replay_csv.cpp \
  ../src/imu_ekf.cpp ../src/imu_fusion.cpp ../src/imu_initializer.cpp \
  ../src/imu_transform.cpp -o /tmp/replay_csv
```

Run it, with the walk values you want to try:

```bash
/tmp/replay_csv results/stationary_<date>.csv --rate-walk 0.1 --accel-walk 1.0
/tmp/replay_csv results/stationary_<date>.csv --no-icm        # ZED only
```

The summary prints mean, std, published covariance and `var/cov` per axis.
Other options: `--gravity-sigma`, `--icm-gyro-bias-walk`, `--icm-accel-bias-walk`,
`--accel-inflation`, `--skip`, and `--out fused.csv` to save the replayed
output in the same format as `imu_compare.py`.

For a **stationary** CSV, lower noise at a similar `var/cov` is better.

For a **moving** CSV the std figures are meaningless, because the signal is
moving. Instead replay with `--out` for each candidate and plot the fused gyro z
against the ZED gyro z. The fused curve should sit on top of the ZED one. If it
trails the ZED when the rover starts or stops turning, the walk values are too
low. Pick the lowest values that show no visible lag.

Limits of the replay: it assumes both IMUs are already in `base_link` axes (no
mount rotation) and only has the diagonal of each covariance. Confirm the final
values by replaying the bag through the real node:

```bash
# terminal 1
ros2 launch rover_imu_fusion imu_fusion.launch.py use_icm:=true publish_icm_tf:=false use_sim_time:=true
# terminal 2
ros2 bag play ~/rover_data/bags/moving_<date> --clock
# terminal 3
python3 scripts/imu_compare.py --sim-time --output results/replay_<date>.csv
```

Then put the chosen values in `config/imu_fusion.yaml` under `filter:`.
