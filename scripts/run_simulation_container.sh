#!/usr/bin/env bash
# Run Humble + Gazebo Classic on macOS/Linux without modifying host sources.
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
image="${ROVER_SIM_IMAGE:-rover-gazebo:humble}"
if ! docker info >/dev/null 2>&1; then
  echo 'Start Docker Desktop and rerun this command.' >&2
  exit 2
fi
if [[ "${1:-}" == "--build" ]]; then
  docker build --platform linux/amd64 -t "$image" \
    -f "$workspace_dir/docker/gazebo.Dockerfile" "$workspace_dir/docker"
  shift
fi
if [[ $# -eq 0 ]]; then
  set -- bash
fi
docker run --rm --init --platform linux/amd64 --shm-size=1g \
  --cap-drop ALL --security-opt no-new-privileges \
  --mount "type=bind,source=$workspace_dir,target=/source,readonly" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-197}" \
  "$image" "$@"
