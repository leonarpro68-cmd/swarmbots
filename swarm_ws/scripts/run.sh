#!/usr/bin/env bash
# Start (or attach to) the swarm dev container with X11 + GPU.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}/docker"

# Allow local X clients (revoked on shell exit).
xhost +local:docker >/dev/null 2>&1 || true
trap 'xhost -local:docker >/dev/null 2>&1 || true' EXIT

export UID="${UID:-$(id -u)}"
export GID="${GID:-$(id -g)}"

if [ "$(docker ps -q -f name=^swarm_dev$)" ]; then
    docker exec -it swarm_dev bash
else
    docker compose run --rm --service-ports swarm bash
fi
