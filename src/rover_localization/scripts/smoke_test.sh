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
colcon build --packages-select rover_localization --symlink-install
source "$workspace_dir/install/setup.bash"

ros2 launch rover_localization sim_localization.launch.py &
launch_pid=$!
cleanup() {
  if kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT

python3 "$script_dir/probe_simulation.py" --timeout-sec 40
