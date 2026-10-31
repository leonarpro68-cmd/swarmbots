# SwarmBots D.9.8C.5 — Despeje del depósito y CBS discreto de tres fases

**Solo planificación offline. No mueve robots, no publica `/cmd_vel`, no acciona pinzas, no altera el registro de misión.**

## Secuencia propuesta

1. Oleada 1: `summit0` con V2 al verde y `summit2` con A2 al azul; `summit1` espera.
2. **Tras confirmar ambas liberaciones**, `summit0` se retira del depósito verde hasta un punto de estacionamiento calculado por CBS, mientras `summit1` y `summit2` están inmóviles.
3. Oleada 2: `summit1` con V1 al verde; los otros dos robots permanecen estacionados.

Los estados finales de las dos primeras fases son **predicciones de la cuadrícula**, no odometría nueva. En ejecución real deben replanificarse las fases con posiciones observadas. Los ticks CBS no son segundos, ni trayectorias Hybrid V2/D.9.2.

## Instalar

En host: `unzip -o ~/Descargas/swarm_D98C5_despeje_CBS.zip -d ~/swarmbots`

En Docker:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/scripts:$PWD/src/swarm_behavior:$PYTHONPATH"
python3 -m unittest discover -s scripts/tests -p test_d98c5_parking_cbs.py -v
```

## Generar plan (Gazebo cerrado)

```bash
python3 scripts/d98c5_parking_cbs.py \
  --snapshot scripts/resultados/d9_live/seed_100_pickup1_009/d98c1/delivery1_live_snapshot.json \
  --waves scripts/resultados/d9_live/seed_100_pickup1_009/d98c1/delivery1_wave_proposal.json \
  --geometry scripts/resultados/d9_live/seed_100_pickup1_009/d98c1/delivery1_wave_geometry.json
```

Informe: `scripts/resultados/d9_live/seed_100_pickup1_009/d98c1/delivery1_wave_parking_cbs_preview.json`.

Resultado comprobado sobre la instantánea histórica: `CBS_DISCRETE_THREE_PHASES_READY_NOT_EXECUTABLE`, estacionamiento nominal de `summit0` en `(3.75, -10.0)` metros. Es una solución preliminar, no garantía de seguridad en Gazebo.

## Antes de la ejecución física

- Recuperar estado con tres cargas y verificar uniones `DetachableJoint` reales o reconstruir `pickup1`.
- Obtener odometría fresca y estado confirmado del registro de misión D.8.
- Adaptar Hybrid V2 + D.9.2 a cada fase/robot activo, con los robots detenidos como obstáculos.
- Añadir comprobaciones temporales, separación continua y límites dinámicos; garantizar un único publicador de velocidades por robot.
- Confirmar **desprendimiento real** antes de la fase de despeje y antes de iniciar la segunda oleada.
- Planificar despeje y segunda oleada **desde la odometría real de ese momento**, no reutilizando los vértices previstos.

## Alcance técnico

Reutiliza `swarm_behavior.swarm_planning.cbs` y funciones de `d98c4_wave_cbs_preview`; por ello requiere que D.9.8C.4 esté instalado. El procedimiento de búsqueda inspecciona los vértices disponibles del grafo D.8, comprueba una ruta de despeje y valida una ruta de la segunda oleada evitando el punto estacionario del Summit 0. No certifica geometría de carga sobresaliente ni colisiones continuas bajo el controlador físico.
