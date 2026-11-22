#!/usr/bin/env bash
# Drive the REAL localization stack with fake sensors along a KNOWN PATH and compare
# EXPECTED vs ACTUAL. No hardware and no Gazebo needed.
#
# Usage: scripts/run_fake_sim.sh [straight|turn_in_place|square|out_and_back|custom] ["path spec"]
#   scripts/run_fake_sim.sh square
#   scripts/run_fake_sim.sh custom "wait:10;drive:5@0.5;turn:90@30;drive:5@0.5;wait:10"
# Everything is written to ~/sim_runs/<timestamp>_<path>/ (see summary.txt first).
# Stop the real sensor drivers first: the validators need exactly one publisher per topic.
set -o pipefail
ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
path="${1:-straight}"
spec="${2:-}"
run="${RUN_DIR:-$HOME/sim_runs/$(date +%Y%m%d_%H%M%S)_$path}"
mkdir -p "$run"
source /opt/ros/humble/setup.bash
source "$ws/install/setup.bash"

if ps -eo args | grep -E "[g]ps_publisher|[i]mu_publisher|[z]ed_wrapper|[c]omponent_container" >/dev/null; then
  echo "Real sensor drivers are running; stop them first (fake data would be rejected)." >&2
  exit 2
fi

setsid ros2 launch rover_localization hardware_localization.launch.py \
  enable_global:=true heading_mode:=startup initial_heading_deg:=90.0 \
  publish_approximate_mounts:=true > "$run/localization.log" 2>&1 &
loc=$!
sleep 6   # let the EKFs and navsat come up
setsid python3 "$ws/scripts/record_run.py" --ros-args -p out_dir:="$run" -p duration_s:=900.0 \
  > "$run/recorder.log" 2>&1 &
rec=$!
extra=()
[[ -n "$spec" ]] && extra=(-p "path_spec:=$spec")
setsid python3 "$ws/scripts/fake_sensors.py" --ros-args -p out_dir:="$run" -p path_name:="$path" \
  "${extra[@]}" > "$run/fake_sensors.log" 2>&1 &
fake=$!
echo "Running path '$path' (see $run) ..."
wait "$fake" 2>/dev/null
wait "$rec" 2>/dev/null
kill -INT -- -"$loc" 2>/dev/null; sleep 3; kill -KILL -- -"$loc" 2>/dev/null
python3 "$ws/scripts/analyze_fake_run.py" "$run"
