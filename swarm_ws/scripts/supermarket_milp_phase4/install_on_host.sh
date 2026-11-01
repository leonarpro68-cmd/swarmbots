#!/usr/bin/env bash
set -euo pipefail
WS="${1:?Uso: install_on_host.sh /ruta/workspace}"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET="$WS/scripts/supermarket_milp_phase4"
if [[ -e "$TARGET" ]]; then echo "ERROR: $TARGET existe; no se reemplaza" >&2; exit 2; fi
mkdir -p "$TARGET"
cp -a "$DIR/." "$TARGET/"
echo "Instalado: $TARGET"
