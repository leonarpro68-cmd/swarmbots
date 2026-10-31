#!/usr/bin/env bash
set -euo pipefail
# Execute only from the original installed swarm_ws, with sourced ROS 2.
# NEVER publishes movement: deliberately omit --execute.
: "${SWARM_WS:?Point SWARM_WS to installed swarm_ws}"
: "${PICKUP1_NPZ:?Point PICKUP1_NPZ to the exact timed NPZ}"
: "${PICKUP1_CLOCK:?Point PICKUP1_CLOCK to the matching V3 clock JSON}"
: "${PICKUP1_SHA:?Set PICKUP1_SHA to SHA-256 of PICKUP1_NPZ}"
cd "$SWARM_WS"
python3 scripts/d9913_stage_coordinator.py \
 --stage pickup1 \
 --trajectory "$PICKUP1_NPZ" \
 --successive-clock "$PICKUP1_CLOCK" \
 --expected-sha256 "$PICKUP1_SHA" \
 --nav-speed-limit 2.0 \
 --nav-accel-limit 2.0 \
 --observe-seconds 8 \
 --output /tmp/supermarket_pickup1_original_dryrun.json
