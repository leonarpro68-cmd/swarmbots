#!/usr/bin/env bash
# Build the workspace and launch the swarm sim on the HOST (no Docker).
# Swarm robot: Summit XLS omni (mecanum). Para minibots usar sim.launch.py a mano.
# Requires: ROS 2 Humble + Gazebo Fortress + ros-humble-ros-ign* installed on host.
# Usage: ./scripts/sim_native.sh [n_robots]
set -euo pipefail

N="${1:-2}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "ERROR: ROS 2 Humble no encontrado en /opt/ros/humble/" >&2
    exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-up-to swarm_description swarm_worlds summit_xl_description swarm_behavior

# shellcheck disable=SC1091
source install/setup.bash

ros2 launch swarm_worlds sim_summit.launch.py n_robots:="${N}"
