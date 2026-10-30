#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd -- "$script_dir/../../.." && pwd)"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'ROS 2 Humble was not found at /opt/ros/humble/setup.bash' >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
if ! command -v colcon >/dev/null 2>&1; then
  echo 'colcon is not installed or not on PATH' >&2
  exit 1
fi

cd "$workspace_dir"
echo '=== Rover localization: synthetic EKF smoke test ==='
echo 'No physical ZED camera or IMU is used in this run.'
echo 'The simulator publishes the same topics that the real rover will use:'
echo '  /zed/zed_node/odom       fake ZED optical Odometry measurement'
echo '  /imu/fused               fake joint-IMU gyro measurement'
echo '  /simulation/ground_truth clean reference path; never sent to the EKF'
echo '  /odometry/local          EKF output being checked'
echo '  odom -> base_link        EKF transform being checked'
echo "Workspace: $workspace_dir"
echo '[smoke 1/4] Building rover_localization...'
colcon build --packages-select rover_localization --symlink-install
source "$workspace_dir/install/setup.bash"

# The launch starts the unchanged production EKF alongside the C++ simulator.
# It deliberately uses the real topic names so this checks the same wiring that
# will later carry ZED optical odometry and the upstream fused IMU stream.
echo '[smoke 2/4] Starting synthetic data and EKF...'
ros2 launch rover_localization sim_localization.launch.py &
launch_pid=$!
echo "  Launch PID: $launch_pid"
cleanup() {
  if kill -0 "$launch_pid" 2>/dev/null; then
    echo "[smoke cleanup] Requesting ROS launch shutdown (PID $launch_pid)..."
    kill -INT "$launch_pid" 2>/dev/null || true
    local shutdown_deadline=$((SECONDS + 5))
    while kill -0 "$launch_pid" 2>/dev/null; do
      if (( SECONDS >= shutdown_deadline )); then
        echo 'ROS launch did not exit after SIGINT; sending SIGTERM.' >&2
        kill -TERM "$launch_pid" 2>/dev/null || true
        shutdown_deadline=$((SECONDS + 2))
        while kill -0 "$launch_pid" 2>/dev/null && (( SECONDS < shutdown_deadline )); do
          sleep 0.1
        done
        if kill -0 "$launch_pid" 2>/dev/null; then
          echo 'ROS launch did not exit after SIGTERM; sending SIGKILL.' >&2
          kill -KILL "$launch_pid" 2>/dev/null || true
        fi
        break
      fi
      sleep 0.1
    done
    wait "$launch_pid" 2>/dev/null || true
    echo '[smoke cleanup] Background launch stopped.'
  fi
}
trap cleanup EXIT

# probe_simulation.py subscribes only to EKF output and the simulator's private
# truth topic. It verifies movement, time-matched XY agreement, and the EKF TF.
echo '[smoke 3/4] Probing EKF output against the comparison-only ground truth...'
python3 "$script_dir/probe_simulation.py" --timeout-sec 40
echo '[smoke 4/4] PASS: synthetic topic wiring, EKF motion, error threshold, and TF passed.'
