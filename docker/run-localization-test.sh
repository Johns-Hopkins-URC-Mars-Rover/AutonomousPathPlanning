#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
# Fail if the image accidentally supplies either retiring package.
for package in rover_localization rover_imu_fusion; do
  if ros2 pkg prefix "$package" >/dev/null 2>&1; then
    echo "Unexpected legacy package installed: $package" >&2
    exit 1
  fi
done
colcon --log-base /tmp/colcon-log build --symlink-install \
  --base-paths /source/src/robot_localization /source/src/rover_bringup \
  --packages-select robot_localization rover_bringup \
  --build-base /tmp/build --install-base /tmp/install \
  --cmake-args -DBUILD_TESTING=OFF -DBUILD_ROVER_TESTING=ON -DCMAKE_BUILD_TYPE=Release
source /tmp/install/setup.bash
colcon --log-base /tmp/colcon-log test \
  --base-paths /source/src/robot_localization /source/src/rover_bringup \
  --packages-select robot_localization \
  --build-base /tmp/build --install-base /tmp/install \
  --event-handlers console_direct+ \
  --ctest-args -R '^rover_' --output-on-failure
test -s /tmp/build/robot_localization/test_results/robot_localization/rover_dual_ekf_e2e.xunit.xml
colcon test-result --test-result-base /tmp/build --verbose
# Verify the real top-level launch with drivers disabled and synthetic inputs.
ROVER_TEST_BRINGUP=1 ROVER_TEST_PROFILE=zed_only python3 -m launch_testing.launch_test \
  /source/src/robot_localization/test/dual_ekf_launch_test.py \
  --junit-xml=/tmp/bringup-dual-ekf.xunit.xml --package-name=robot_localization
printf '%s\n' 'PASS: standalone dual EKFs/navsat and rover_bringup work with both retiring packages absent.'
