#!/usr/bin/env bash
set -eo pipefail
cd "${HOME}/swarm_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
P="${HOME}/swarm_ws/scripts/supermarket_v3/data"
NPZ="$P/pickup1_supermarket_seed100_v3.npz"
CLOCK="$P/pickup1_supermarket_seed100_clock.json"
python3 - "$NPZ" "$CLOCK" <<'PY_CHECK'
import hashlib,json,sys
from pathlib import Path
p,c=map(Path,sys.argv[1:]); digest=hashlib.sha256(p.read_bytes()).hexdigest()
data=json.loads(c.read_text())
assert digest==data['input_sha256'], 'El SHA256 del NPZ no coincide con el reloj'
assert data['seed']==100 and data['stage']=='pickup1','Etapa/semilla incorrectas'
assert data['status']=='OFFLINE_SCHEDULE_CERTIFIED_NOT_EXECUTABLE','Reloj offline no certificado'
print('Hash compatible:',digest)
PY_CHECK
# Deliberadamente NO pasa --execute ni --confirm-runtime-v2.
python3 scripts/d9913_stage_coordinator.py \
 --stage pickup1 \
 --trajectory "$NPZ" \
 --successive-clock "$CLOCK" \
 --expected-sha256 "4a3015c4ac3d64da0555b485a3bfac77b7243340d251c8f6388c19ac35f19bd4" \
 --nav-speed-limit 2.0 \
 --nav-accel-limit 2.0 \
 --observe-seconds 8 \
 --output scripts/resultados/supermarket_pickup1_seed100_d9913_dryrun.json
