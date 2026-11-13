# Verification record

Date: 2026-11-12. Work branch: `fix/gazebo-localization-integration`; baseline `1da6fe3`.
All changes remain uncommitted for review.

**Update:** At the user's request the six original wheel axes have been restored.
Positive commands now move the simulated model backward relative to `base_link`.
The motion test expects that direction. A positive-command run moved ground
truth -0.277 m along the rover's forward axis, while the local estimate differed
from ground-truth displacement by 0.529 m. A separate run failed the unchanged
0.4 m localization disagreement threshold at 0.509 m. The passing movement and
Nav2 results below predate this restoration and do not verify the current model.

## Environment and reproducible commands

Apple Silicon Mac, Docker Desktop, Linux AMD64 emulation. The container supplies
Gazebo Classic 11.10.2, upstream Humble robot_localization 3.5.4, Nav2 bringup
1.1.20, and Mesa/Xvfb. Image tested: `rover-gazebo:humble`, image ID
`sha256:861e8088b1e8b2b25dab8f89f174c2a8085e280838a3efd282888647c30c8c32`.
The Dockerfile pins the ROS distribution/base tag; future package updates may
produce another image ID, so record versions when comparing runs.

```bash
# Build the compatibility image and verify an advancing Gazebo ROS clock.
bash scripts/run_simulation_container.sh --build bash -lc \
  'source /opt/ros/humble/setup.bash; python3 /source/docker/gazebo_environment_smoke.py'

# Clean container build, package tests, physical motion and full Nav2 checks.
bash scripts/test_simulation_container.sh

# Synthetic active hardware-profile checks, including input loss/recovery.
bash scripts/test_localization_container.sh
```

Sources are mounted read-only. Build/install/log output lives in disposable
container storage. Default simulation is georeferenced `flat.world` with the
simplified rover; the original CAD URDF is retained separately.

## Results and evidence

- Environment smoke PASS: 10 advancing `/clock` messages, 0.100 to 1.000 seconds.
- Four active packages repeatedly built successfully from installed resources.
- Final clean entry-point repeat PASS (exit 0): Colcon **58 tests, 0 errors,
  0 failures, 0 skipped**. Both real Gazebo profiles passed afterward.
- Final active localization harness PASS (exit 0). Real upstream EKF/navsat synthetic profiles PASS: full, dropout/recovery,
  ZED-only, top-level bringup ZED-only, and simple bringup with ICM disabled.
  The final focused full run measured local x=2.598 m, global x=4.597 m,
  maximum adjacent local position step=0.0078 m. Predeclared bounds were local
  position error <=0.35 m, velocity error <=0.05 m/s, local adjacent step<0.15 m,
  and global GPS correction>1 m. Dropout tests verify degrading/recovering
  diagnostics and covariance growth on complete sensor loss.

  | Final harness profile | Local x (m) | Global x (m) | Max adjacent local step (m) |
  | --- | ---: | ---: | ---: |
  | Full | 2.596 | 4.592 | 0.0089 |
  | Dropout/recovery | 4.399 | 6.402 | 0.0088 |
  | ZED only | 2.588 | 4.587 | 0.0113 |
  | Bringup ZED only | 2.596 | 4.584 | 0.0092 |
  | Simple bringup ICM off | 2.597 | 4.588 | 0.0087 |
- Localization callback/helper coverage: 84%, 361 statements/57 missed.
  Navigation after the lifecycle fix: **34 tests passed**, **86%** statement
  coverage across the package (client 82%, relay 95%, readiness 91%).
- Actual Gazebo model sustained finite physics/IMU/ground-truth for >30 simulated
  seconds. GPS covariance diagonals were 0.25, 0.25, 1.0; depth cloud 640x480
  contained finite ground points and expected infinite sky pixels.
- Independent model motion RED: +0.2 m/s for 3.016 simulated seconds moved
  -0.568 m forward. After wheel-axis correction GREEN: +0.579 m forward.
- Turning isolation: +0.4 rad/s for 3.016 simulated seconds initially produced
  +0.198 rad. Explicit lateral/longitudinal friction with the correct
  collision-frame axle direction produced +0.996 rad. These are fixture
  parameters, not calibrated rover dynamics.
- Combined runtime passes before the last startup-race regression check verified
  finite sensor/odometry streams, TF connectivity, GPS conversion, forward
  movement, Nav2 reachable-goal success, acknowledged terminal cancellation,
  outside-costmap failure, and physical obstacle marking (clear cost 0 -> 99).
- The motion repeat also verified reverse displacement -0.276 m and positive
  turn +0.700 rad. Forward displacement was 0.315 m and local/truth displacement
  disagreement 0.071 m; the probe fixes its thresholds before evaluating a run.
- Final repeat after the Nav2 lifecycle fix: forward displacement **0.315 m**,
  local/truth displacement disagreement **0.024 m**, reverse **-0.276 m**,
  turn **+0.693 rad**. Current-GPS conversion disagreements were **0.982 m**
  (motion profile) and **0.651 m** (navigation profile), within the predeclared
  2 m bound. Nav2 reached the goal, acknowledged cancellation with terminal
  canceled status, handled the outside-costmap goal, and marked the physical
  box (cost **0 -> 99**). The complete entry-point command returned **exit 0**.

Raw local logs: `/private/tmp/rover-active-tests-2.log`,
`/private/tmp/rover-localization-final-check.log`,
`/private/tmp/rover-final-entrypoint.log`, `/private/tmp/rover-final-verified.log`,
and `/private/tmp/rover-final-repeat.log`. Earlier failures are intentionally
retained; read each run's final status rather than treating every log as PASS.
The final complete localization harness log is
`/private/tmp/rover-localization-final-harness.log` (exit 0).

## Defects reproduced and repaired

- Machine-specific park terrain URI and absent camera mesh.
- Stale archived launch/package references and test mounts.
- Passive rocker joints in the simplified fixture generated invalid physical
  states after contact; rigid fixture joints resolve the reproduced failure.
- Reversed wheel axes, inconsistent geometric wheel separation, and camera
  remappings missing their configured camera-name prefix.
- Fixed-delay controller/Nav2 startup replaced by service/lifecycle/data/TF
  readiness and explicit failure shutdown.
- LocalCartesian filtered-GPS reverse conversion in the installed Humble binary
  crashed; the hardware profile now uses UTM conversion.
- Source timestamp validation failed after a simulation/bag clock rewind; it
  now resets accepted timestamp history on observed clock rewind.
- Unbounded navigation waits, unacknowledged cancellation, late goal acceptance,
  swallowed process failures, and ambiguous heading conventions.
- Action-server visibility preceded Nav2 lifecycle activation, causing a
  startup goal rejection. Client and probe now wait for active lifecycle states.
- Circular radius did not cover the rover collision geometry; conservative
  polygon footprint and footprint-aware trajectory critic are configured.
- Probe bugs: reliable clock subscription incompatible with best-effort clock;
  depth API returned an ndarray; infinite sky samples preceded valid ground;
  test wrapper omitted external source paths and initially discovered zero tests.
  All were diagnosed explicitly rather than counted as implementation success.

## Verification boundaries

These tests establish the headless flat-world simulation and synthetic ROS
contracts. They do not establish physical rover accuracy, calibrated suspension
or friction, measured hardware mounts, ZED SDK/real driver operation, desktop GUI
rendering, or park-terrain traversal. Park resource resolution was smoke-tested,
but terrain navigation was not. The physical box check establishes sensor-to-
costmap marking; collision-free obstacle detours are not claimed. The retained
4.x estimator lifecycle compatibility branch was not tested with a 4.x binary.
