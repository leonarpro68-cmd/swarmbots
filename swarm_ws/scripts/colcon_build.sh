#!/usr/bin/env bash
# Run `colcon build` inside the dev container.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}/docker"

docker compose run --rm swarm bash -lc \
    "cd /home/dev/swarm_ws && colcon build --symlink-install $*"
