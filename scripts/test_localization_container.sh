#!/usr/bin/env bash
# Build rover configuration against the released robot_localization binaries.
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if ! command -v docker >/dev/null 2>&1; then
  echo 'Docker is required for the ROS graph test; Python adapter tests can run natively.' >&2
  exit 2
fi
if ! docker info >/dev/null 2>&1; then
  echo 'Docker engine is stopped or unavailable. Start Docker Desktop, wait until it is ready, then rerun this command.' >&2
  exit 2
fi
docker build -t rover-localization-test \
  -f "$workspace_dir/docker/localization-test.Dockerfile" "$workspace_dir/docker"
docker run --rm --network none --read-only \
  --tmpfs /tmp:exec,mode=1777,size=4g \
  --cap-drop ALL --security-opt no-new-privileges \
  --mount "type=bind,source=$workspace_dir/src/rover_localization,target=/source/src/rover_localization,readonly" \
  --mount "type=bind,source=$workspace_dir/src/rover_bringup,target=/source/src/rover_bringup,readonly" \
  rover-localization-test
