#!/usr/bin/env bash
# Build the Docker image for the swarm workspace.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}/docker"

export UID="${UID:-$(id -u)}"
export GID="${GID:-$(id -g)}"

docker compose build "$@"
