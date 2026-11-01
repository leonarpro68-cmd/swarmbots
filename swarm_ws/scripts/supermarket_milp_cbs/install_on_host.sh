#!/usr/bin/env bash
set -euo pipefail
PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${1:-$HOME/swarmbots/swarm_ws}"
[[ -d "$WS/scripts/supermarket_milp" ]] || { echo 'ERROR: instala primero fases 1 y 2' >&2; exit 2; }
DEST="$WS/scripts/supermarket_milp_cbs"
[[ ! -e "$DEST" ]] || { echo "ERROR: el destino ya existe; no se sobreescribe: $DEST" >&2; exit 2; }
mkdir -p "$DEST"
cp -a "$PKG"/. "$DEST"/
echo "INSTALADO: $DEST"
echo 'Sin cambios en launch, Gazebo, scripts físicos o planificación validada.'
