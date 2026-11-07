#!/usr/bin/env bash
# Runs the real camera+gyro EKF with synthetic inputs; no ROS on the host needed.
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if ! command -v docker >/dev/null 2>&1; then
  echo 'Docker is required for the ROS graph test. Native tests use scripts/benchmark_fusion.py.' >&2
  exit 2
fi
if ! docker info >/dev/null 2>&1; then
  echo 'Docker engine is stopped or unavailable. Start Docker Desktop, wait until it is ready, then rerun this command.' >&2
  exit 2
fi
docker build -t rover-localization-test \
  -f "$workspace_dir/docker/localization-test.Dockerfile" "$workspace_dir/docker"
docker run --rm --network none --read-only \
  --tmpfs /tmp:exec,mode=1777,size=2g \
  --cap-drop ALL --security-opt no-new-privileges \
  --mount "type=bind,source=$workspace_dir/src/rover_localization,target=/source/src/rover_localization,readonly" \
  rover-localization-test
