#!/usr/bin/env bash
set -e

# Source ROS 2 and the local overlay (if it has been built).
source /opt/ros/jazzy/setup.bash
if [ -f /home/dev/swarm_ws/install/setup.bash ]; then
    source /home/dev/swarm_ws/install/setup.bash
fi

exec "$@"
