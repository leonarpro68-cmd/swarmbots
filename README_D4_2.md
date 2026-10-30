# SwarmBots Hybrid V2 — D.4.2, ejecución supervisada de una etapa

Este ZIP **actualiza solamente** `swarm_ws/scripts/run_hybrid_v2_stage_ros.py`. Requiere D.4.1 funcional y el seguidor temporizado ya activo. No modifica el algoritmo SLSQP, el controlador ni el launch de Gazebo. **No hay validación automática de distancias a todos los obstáculos**; por ello, el modo de movimiento requiere confirmación explícita del operador sobre la coincidencia entre el mundo Gazebo, el JSON de escenario y la huella física de los Summit. El radio 0.55 m sigue siendo experimental.

## Instalar

**Host Ubuntu**:

```bash
cd ~/swarmbots
unzip -o ~/Descargas/swarm_hybrid_v2_D4_2_ejecutor_supervisado.zip -d ~/swarmbots
```

**Dentro de Docker** (Gazebo ya lanzado, un solo coordinador de movimiento):

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PYTHONPATH"
python3 -m py_compile scripts/run_hybrid_v2_stage_ros.py
```

## 1. Comprobar archivos y escenario

```bash
ls -lh scripts/resultados/d4_live/pickup1_live_report.json scripts/resultados/d4_live/pickup1_live_controls.npz
```

**Confirmar** que el `scenario_json` del reporte D.4.1 es el escenario empleado para crear el SDF del mundo que está ejecutándose en Gazebo. Verificar que los robots `summit0..2` y obstáculos/depositos/basuras coinciden, y que el modelo 0.55 m + 0.05 m resulta apropiado para las colisiones físicas. El script **no puede comprobar por sí solo** esa identidad. Cualquier duda: no ejecutar movimiento.

## 2. Preflight SIN movimientos

```bash
python3 scripts/run_hybrid_v2_stage_ros.py \
  --report scripts/resultados/d4_live/pickup1_live_report.json \
  --controls scripts/resultados/d4_live/pickup1_live_controls.npz \
  --stage pickup1 \
  --scenario /tmp/swarm_hybrid_123_scenario.json
```

Esperado: `PRECHECK OK` y `DRY RUN OK`. Replanificar con `replan_hybrid_v2_from_odom.py` si el robot se movió fuera de la tolerancia.

## 3. Ejecutar **una sola recogida** (solo tras validar el mundo/huella)

Con Gazebo funcionando, los tres seguidores habilitados, sin otro planificador o teleoperación enviando órdenes al mismo tiempo, y con opción de interrumpir la simulación:

```bash
python3 scripts/run_hybrid_v2_stage_ros.py \
  --report scripts/resultados/d4_live/pickup1_live_report.json \
  --controls scripts/resultados/d4_live/pickup1_live_controls.npz \
  --stage pickup1 \
  --scenario /tmp/swarm_hybrid_123_scenario.json \
  --robot-radius 0.55 \
  --operational-margin 0.05 \
  --world-verified \
  --execute
```

El ejecutor comprueba odometría reciente y posición inicial de cada robot, existencia de un único suscriptor por comando, tiempo simulado, límite de velocidad inicial, frames reportados, correspondencia de puntos iniciales y parámetros del informe. Envía tres órdenes con inicio común, espera ACK, supervisa separación de centros >=1.15 m y error respecto a referencia <=0.16 m, y cancela ante fallos detectados. **No protege contra todos los fallos físicos ni verifica todos los obstáculos online.** `Ctrl+C` cancela la sesión; si hay riesgo inmediato, pausar Gazebo o utilizar el mecanismo de parada de emergencia disponible.

Nota: ante desacuerdo entre odometría instantánea y controles, el programa debe abortar. No incrementar la tolerancia ni modificar puntos de control para hacerlo pasar.
