#!/usr/bin/env bash
# Test active rover integration with upstream Humble robot_localization.
set -euo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$workspace_dir/scripts/run_simulation_container.sh" "$@" \
  bash -lc 'cp /source/docker/run-localization-test.sh /tmp/run-localization-test.sh; exec bash /tmp/run-localization-test.sh'
