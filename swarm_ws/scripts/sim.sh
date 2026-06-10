#!/usr/bin/env bash
# Build (if needed) and launch the swarm simulation.
# Usage: ./scripts/sim.sh [n_robots]
set -euo pipefail

N="${1:-2}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}/docker"

xhost +local:docker >/dev/null 2>&1 || true
trap 'xhost -local:docker >/dev/null 2>&1 || true' EXIT

export UID="${UID:-$(id -u)}"
export GID="${GID:-$(id -g)}"

docker compose run --rm --service-ports swarm bash -lc "
  cd /home/dev/swarm_ws &&
  colcon build --symlink-install --packages-select swarm_description swarm_worlds &&
  source install/setup.bash &&
  ros2 launch swarm_worlds sim.launch.py n_robots:=${N}
"
