#!/usr/bin/env bash
set -euo pipefail
WS="${1:-$HOME/swarmbots/swarm_ws}"
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[[ -d "$WS/scripts" ]] || { echo "ERROR: workspace no encontrado: $WS/scripts" >&2; exit 2; }
DST="$WS/scripts/supermarket_milp_corridor_fix"
[[ ! -e "$DST" ]] || { echo "ERROR: destino existe, no se sobrescribe: $DST" >&2; exit 2; }
mkdir -p "$DST"
cp -a "$SRC"/. "$DST"/
find "$DST" -type d -name '__pycache__' -prune -exec rm -rf {} +
find "$DST" -name '*.pyc' -delete
printf 'INSTALADO: %s\n' "$DST"
printf 'El proyecto original, sus launches y trayectorias NO se modificaron.\n'
