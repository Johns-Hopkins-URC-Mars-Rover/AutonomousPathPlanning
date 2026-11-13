# Gazebo test 1

This runs the repository's existing headless Gazebo integration test and saves
its output on the host. It launches two separate simulations, first motion with
Nav2 off, then navigation with Nav2 on. It does not open a Gazebo window.

## Run

Start Docker Desktop. From the repository root:

```bash
python3 'gazebo test 1/run_test.py' --build
```

`--build` builds or refreshes the Docker image; omit it on later runs when the
image already exists. The original script mounts the source read-only and runs
builds and simulations in disposable container storage. Each invocation saves
`runs/<timestamp>/console.log` and `runs/<timestamp>/results.csv` in this folder.
Keep the log: it contains detailed errors and output beyond the CSV. A failed
run can produce a partial CSV; check `overall,process_exit_code` and the log.
The wrapper returns a nonzero exit code if the underlying test fails.

## Inputs

- Default world: `src/rover_description/world/flat.world`, a flat ground plane
  with a geographic origin. Model: `simp_rover.urdf`, a rigid-suspension fixture.
- Simulated wheel odometry, GPS, ZED IMU, and depth camera feed the localization
  and navigation stack. Ground-truth odometry is used as a test reference only.
- Motion profile: raw linear commands +0.15 and -0.15 m/s, then angular command
  +0.4 rad/s, each for 2 simulated seconds. Original wheel axes make positive
  raw commands move backward and positive raw turns rotate negatively. Separate
  simulation adapters correct signs for the EKFs and Nav2.
- Navigation profile: a 1 m goal, a 5 m goal that is canceled, a 200 m
  outside-map goal, and a 0.6 x 0.8 x 1.0 m box spawned 2.5 m ahead.

## Outputs and interpretation

The probe observes `/clock`, `/simulation/ground_truth`,
`/diff_drive_controller/odom`, `/odometry/local`, `/odometry/global`, `/gps/fix`,
`/zed/zed_node/imu/data`, and `/depth/points`. It checks TF connections,
sensor/odometry validity and GPS conversion. CSV stream counts are message
counts at the time the stream check printed, not rates or full-run totals.

The CSV lists the actual values printed by the probe and their fixed acceptance
limits. A successful run has 0 errors/failures in the package test summary,
both profile rows marked PASS, and process exit code 0. The navigation PASS
indicates the 1 m goal succeeded, cancellation completed, the outside-map goal
was rejected or aborted, and the box was marked in the local costmap. It does
not prove obstacle avoidance. Not every assertion has a separate measured
value in the script's output; consult the log for failures.

These results are simulation checks, not calibrated physical rover performance.
No simulated visual odometry is provided. Values will vary between runs.
