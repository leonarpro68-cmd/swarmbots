#!/usr/bin/env bash
# Build the workspace and launch the swarm sim on the HOST (no Docker).
# Swarm robot: Summit XLS omni (mecanum). Para minibots usar sim.launch.py a mano.
# Requires: ROS 2 Jazzy + Gazebo Harmonic + ros-jazzy-ros-gz* installed on host.
# Usage: ./scripts/sim_native.sh [n_robots]
set -euo pipefail

N="${1:-2}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo "ERROR: ROS 2 Jazzy no encontrado en /opt/ros/jazzy/" >&2
    exit 1
fi
# Los scripts de setup de ROS Jazzy referencian variables sin definir
# (AMENT_TRACE_SETUP_FILES, etc.) → desactivar nounset solo al sourcear.
# shellcheck disable=SC1091
set +u; source /opt/ros/jazzy/setup.bash; set -u

colcon build --symlink-install --packages-up-to swarm_description swarm_worlds summit_xl_description swarm_behavior

# shellcheck disable=SC1091
set +u; source install/setup.bash; set -u

ros2 launch swarm_worlds sim_summit.launch.py n_robots:="${N}"
