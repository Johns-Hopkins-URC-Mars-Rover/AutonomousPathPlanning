#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/colcon-log build --symlink-install \
  --base-paths /source/src/rover_localization /source/src/rover_bringup \
  --packages-select rover_localization rover_bringup \
  --build-base /tmp/build --install-base /tmp/install \
  --cmake-args -DBUILD_TESTING=OFF
source /tmp/install/setup.bash
colcon --log-base /tmp/colcon-log test \
  --base-paths /source/src/rover_localization /source/src/rover_bringup \
  --packages-select rover_localization \
  --build-base /tmp/build --install-base /tmp/install \
  --event-handlers console_direct+
colcon test-result --test-result-base /tmp/build --verbose
ros2 launch rover_localization dual_ekf_launch.py --show-args
printf '%s\n' 'PASS: rover localization configuration builds against the released ROS binaries.'
