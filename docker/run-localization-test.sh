#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/colcon-log build \
  --base-paths /source/src/rover_localization \
  --packages-select rover_localization \
  --build-base /tmp/build --install-base /tmp/install \
  --cmake-args -DBUILD_TESTING=ON
source /tmp/install/setup.bash
colcon --log-base /tmp/colcon-log test \
  --base-paths /source/src/rover_localization \
  --packages-select rover_localization \
  --build-base /tmp/build --install-base /tmp/install \
  --event-handlers console_direct+ \
  --ctest-args -R '^localization_(ros|noisy)_e2e$' --output-on-failure
test -s /tmp/build/rover_localization/test_results/rover_localization/localization_ros_e2e.xunit.xml
test -s /tmp/build/rover_localization/test_results/rover_localization/localization_noisy_e2e.xunit.xml
colcon test-result --test-result-base /tmp/build --verbose
printf '%s\n' \
  'PASS: noisy baseline, input-loss diagnostics and recovery met fixture expectations.' \
  'KNOWN LIMIT: the tracking-fault case passes by reproducing a large position error, not by rejecting it.' \
  'Hardware accuracy and the cause of the original ZED failure remain unverified. See src/rover_localization/TESTING.md.'
