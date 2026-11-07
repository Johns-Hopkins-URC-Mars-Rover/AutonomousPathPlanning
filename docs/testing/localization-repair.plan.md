# Localization and GPS repair plan

Scope: rover-owned localization, description, bringup, L76X interface, IMU
fusion, and recording/replay helpers. Preserve upstream ZED and u-blox sources.
Do not invent sensor mounts, noise measurements, heading or hardware results.

1. RED: reproduce nonfinite/repeated-stamp smoke-probe passes; exercise rigid
   camera-to-base conversion, covariance checks, GPS validity and RMC/GGA parsing.
2. Repair local inputs: explicit camera pose adapter, one TF authority, stable
   IMU topic, configurable bench frame without fabricated internal IMU TF.
3. Add optional GPS global EKF + navsat_transform, gated on validated GPS
   covariance/altitude and an explicitly verified earth-referenced heading.
   Keep GPS out of the continuous local EKF.
4. Repair synthetic checks, simulated clock/controller paths, IMU finite/freshness
   guards, recording coverage and frame-safe offline replay.
5. Document real hardware commands, acceptance boundaries and RED/GREEN evidence.
6. Verify Python unit/integration contracts, C++ checks where dependencies permit,
   source syntax, XML/YAML, diffs and coverage. ROS/hardware checks require Ubuntu.

Success: deterministic checks reject invalid/stale data; topic/TF contracts are
consistent; GPS has an opt-in validated path; regression tests pass. This does
not establish that the observed 17 m error is fixed on real hardware. That needs
recorded raw ZED + IMU + GPS + EKF + diagnostics + TF from a measured motion.

No commits or external writes. No changes to measured sensor parameters.

## Execution status

All six source-work milestones have been implemented and checked within this
host's capabilities. Regression and verification evidence is in
[localization-repair.tdd.md](localization-repair.tdd.md). Hardware mounts,
global heading, GPS uncertainty and live reproduction remain unverified; GPS
fusion is therefore disabled by default. No claim of a resolved hardware
position error is made.


## Final verification completion

- Added `scripts/check_localization.py`: portable source/configuration checks,
  fresh native builds/tests, optional helper coverage, JSON results and explicit
  `--ros` package build/test execution. No prebuilt /tmp binaries are required.
- Registered `localization_ros_e2e` with launch_testing. It executes the complete
  synthetic ROS pipeline in a localhost test domain and checks probe exit status.
- Fixed stale diagnostic snapshots and old output surviving smoke-probe health
  checks; regression tests cover expiry, failed/future status and stale output.
- The reproducible non-ROS command passed. Its `--ros` prerequisite check fails
  clearly on this host, which has neither ROS 2 nor colcon. Physical measurement
  and the real ROS test execution remain external verification requirements.
