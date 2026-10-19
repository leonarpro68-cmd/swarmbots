#!/usr/bin/env bash
# Lanzador único desde Ubuntu host (contenedor swarm_dev existente) o dentro de Docker.
set -euo pipefail
SEED=123
CLOSE_ON_SUCCESS=0
for arg in "$@"; do
  case "$arg" in
    --seed=*) SEED="${arg#--seed=}" ;;
    --close-on-success) CLOSE_ON_SUCCESS=1 ;;
    --help|-h) echo 'Uso: ./run_swarmbots.sh --seed=123 [--close-on-success]'; exit 0 ;;
    *) echo "Argumento desconocido: $arg (usa --seed=123)" >&2; exit 2 ;;
  esac
done
EXTRA=""
if [[ "$CLOSE_ON_SUCCESS" == 1 ]]; then EXTRA=" --close-on-success"; fi
[[ "$SEED" =~ ^[0-9]+$ ]] || { echo 'Semilla inválida' >&2; exit 2; }
INNER="cd ~/swarm_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && exec python3 scripts/run_swarmbots_auto.py --seed $SEED --world-original \"\$(ros2 pkg prefix swarm_worlds)/share/swarm_worlds/worlds/tugbot_warehouse.sdf\"$EXTRA"
if [[ -f /.dockerenv ]]; then
  bash -lc "$INNER"
else
  docker inspect -f '{{.State.Running}}' swarm_dev 2>/dev/null | grep -qx true || {
    echo 'El contenedor swarm_dev debe estar ejecutándose; no inicia un contenedor nuevo.' >&2
    exit 1
  }
  if [[ -t 0 && -t 1 ]]; then
    docker exec -it swarm_dev bash -lc "$INNER"
  else
    docker exec -i swarm_dev bash -lc "$INNER"
  fi
fi
