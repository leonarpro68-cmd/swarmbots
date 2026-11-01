#!/usr/bin/env bash
# NEW package only: never change seed100 launch, its runner, its autonomous mission script or prepared contracts.
set -euo pipefail
WS="${1:?Uso: bash install_on_host.sh ~/swarmbots/swarm_ws}"
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAUNCH="$WS/src/swarm_worlds/launch"
TARGET="$WS/scripts/supermarket_seed_pipeline"
AUTO="$WS/scripts/supermarket_auto_experiment/run_auto_experiment.py"
[[ -f "$LAUNCH/sim_summit_supermarket.launch.py" && -f "$LAUNCH/supermarket_support.launch.py" && -f "$AUTO" ]] || {
  echo 'ERROR: faltan launch de supermarket o automatizador ya validado' >&2; exit 2;
}
[[ ! -e "$TARGET" && ! -e "$LAUNCH/supermarket_full_prepared.launch.py" ]] || {
  echo 'ERROR: nuevo pipeline/launch ya instalado; no se sobreescriben planes ni código.' >&2; exit 2;
}
python3 - "$AUTO" "$SRC" <<'PY'
from pathlib import Path
import ast,sys
code=Path(sys.argv[1]).read_text()
if '--authorize-noninteractive' not in code or '--physical-policy' not in code or '--unattended' not in code:
    raise SystemExit('ERROR: el automatizador actual no tiene la versión de soporte (one-launch + oleadas). No se instala.')
oldv2="scenario_for_original_D8_seed100_{mode.upper()}.json"
oldv1="scenario_for_original_D8_seed100_MILP.json"
if oldv2 in code:
    # Seed scoped: no hardcoded seed100; name still has mode for fixed/MILP but new launch only calls MILP.
    code=code.replace(oldv2,"scenario_for_original_D8_seed{seed_from_prepared(prepared)}_{mode.upper()}.json")
elif oldv1 in code:
    code=code.replace(oldv1, "scenario_for_original_D8_seed{seed_from_prepared(prepared)}_MILP.json")
    code=code.replace("scenario=prepared/'scenario_for_original_D8_seed{seed_from_prepared(prepared)}_MILP.json'", "scenario=prepared/f'scenario_for_original_D8_seed{seed_from_prepared(prepared)}_MILP.json'")
else:
    raise SystemExit('ERROR: formato inesperado de ruta D8; no se instala sin revisión.')
# Add simple strict seed getter; prepared must be .../plans/seed_N/prepared.
marker="STAGES=('pickup1','delivery1','pickup2','delivery2')"
if marker not in code:raise SystemExit('ERROR: no se encontró marcador de etapas en automatizador')
seed_fn="""
def seed_from_prepared(prepared):
    parent=prepared.resolve().parent.name
    if not parent.startswith('seed_') or not parent[5:].isdigit():
        raise RuntimeError('Carpeta de plan preparado no contiene semilla declarada')
    return int(parent[5:])
"""
code=code.replace(marker,marker+seed_fn,1)
ast.parse(code)
# Save staged source on package. Installer copies to worktree only if all prechecks passed.
out=Path(sys.argv[2])/'run_auto_experiment_seed.py'
out.write_text(code,encoding='utf8')
print('AUTOMATIZADOR_SEMILLA_AISLADO OK:',out)
PY
# Stage copy using no-clobber: no modification to existing scripts or seed100.
mkdir -p "$TARGET"
cp -a "$SRC"/. "$TARGET"/
find "$TARGET" -type d -name __pycache__ -prune -exec rm -rf {} +
find "$TARGET" -name '*.pyc' -delete
cp "$SRC/launch/supermarket_full_prepared.launch.py" "$LAUNCH/supermarket_full_prepared.launch.py"
chmod +x "$TARGET/run_full_offline.py" "$TARGET/run_prepared_mission.py"
python3 -m py_compile "$TARGET/run_full_offline.py" "$TARGET/run_prepared_mission.py" "$TARGET/run_auto_experiment_seed.py"
echo "INSTALADO: $TARGET"
echo "NUEVO LAUNCH: $LAUNCH/supermarket_full_prepared.launch.py"
echo 'SIN CAMBIOS: supermarket_full_experiment.launch.py (seed100).'
echo 'Para el nuevo launch hay que compilar swarm_worlds dentro de Docker.'
