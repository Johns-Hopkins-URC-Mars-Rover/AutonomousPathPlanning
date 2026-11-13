#!/usr/bin/env bash
# Test both complete installed simulation profiles in disposable Docker storage.
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$workspace_dir/scripts/run_simulation_container.sh" "$@" bash -lc '
  set -e
  source /opt/ros/humble/setup.bash
  colcon build --base-paths /source/src/rover_description /source/src/rover_localization /source/src/rover_navigation /source/src/rover_bringup --cmake-args -DBUILD_TESTING=ON
  source install/setup.bash
  colcon test --base-paths /source/src/rover_description /source/src/rover_localization /source/src/rover_navigation /source/src/rover_bringup --event-handlers console_direct+
  test -s build/rover_description/test_results/rover_description/model_resources.xunit.xml
  colcon test-result --test-result-base build --verbose
  xvfb-run -a python3 /source/scripts/run_simulation_tests.py
'
