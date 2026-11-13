#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export PYTHONDONTWRITEBYTECODE=1
colcon --log-base /tmp/colcon-log build --symlink-install \
  --base-paths /source/src/rover_localization /source/src/rover_bringup /source/src/rover_navigation \
  --packages-select rover_localization rover_bringup rover_navigation \
  --build-base /tmp/build --install-base /tmp/install \
  --cmake-args -DBUILD_TESTING=ON -DBUILD_ROVER_TESTING=ON
source /tmp/install/setup.bash
python3 -B -m unittest discover -s /source/src/rover_localization/test -p 'test_*.py'
python3 /source/src/rover_localization/test/launch_contract_test.py
python3 -B -m unittest discover -s /source/src/rover_bringup/test -p 'test_*.py'
python3 -m pytest -q -p no:cacheprovider /source/src/rover_navigation/test
for profile in full dropouts zed_only; do
  ROVER_TEST_PROFILE="$profile" python3 -m launch_testing.launch_test \
    /source/src/rover_localization/test/dual_ekf_launch_test.py \
    --junit-xml="/tmp/localization-$profile.xunit.xml" --package-name=rover_localization
done
ROVER_TEST_BRINGUP=1 ROVER_TEST_PROFILE=zed_only python3 -m launch_testing.launch_test \
  /source/src/rover_localization/test/dual_ekf_launch_test.py \
  --junit-xml=/tmp/bringup.xunit.xml --package-name=rover_localization
ROVER_TEST_SIMPLE=1 ROVER_TEST_PROFILE=icm_off python3 -m launch_testing.launch_test \
  /source/src/rover_localization/test/dual_ekf_launch_test.py \
  --junit-xml=/tmp/simple.xunit.xml --package-name=rover_localization
printf '%s\n' 'PASS: active localization, sensor switches, navigation unit tests and real EKF/navsat profiles.'
