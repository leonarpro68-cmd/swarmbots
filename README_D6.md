# D.6 — cuatro etapas con operador y replanning desde odometría

**Qué añade**: `scripts/d6_mission_state.py` mantiene el mapa dinámico de basuras recogidas y entregas confirmadas; `scripts/prepare_hybrid_v2_d6_stage.py` extiende el replanificador D.4.1 a `pickup1`, `delivery1`, `pickup2`, `delivery2`. Cada etapa toma un snapshot nuevo de las tres odometrías y construye CBS + V2 para el mapa actualizado. **No ejecuta movimientos automáticamente**.

**Lo que NO añade**: agarre/retirada física de las basuras de Gazebo, comprobación del vaciado de la carga, auditoría de obstáculos mediante la geometría real, ni coordinación continua sin supervisión. El `path_follower` señalando `done` significa fin de la trayectoria, **no** basura recogida o depositada. Confirmar únicamente tras comprobar la acción física y el estado de Gazebo; si las acciones no existen, no confirmar y no continuar.

## Instalación

Extraer `swarm_ws/scripts/` del ZIP dentro de la carpeta `swarm_ws/scripts/` real. Desde Docker:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PYTHONPATH"
python3 -m py_compile scripts/d6_mission_state.py scripts/prepare_hybrid_v2_d6_stage.py
```

## Preparación del escenario

El `scenario.json` debe corresponder exactamente al mundo Gazebo en ejecución. Verificar manualmente que las odometrías `summitN/odom` coinciden con coordenadas Gazebo world, y que el brazo/pinza o el mecanismo de recogida y depósito está operativo. Mantener `/clock`. Iniciar una misión nueva (no sobrescribe una existente):

```bash
python3 scripts/d6_mission_state.py --state scripts/resultados/d6/mision.json init --scenario /tmp/swarm_hybrid_123_scenario.json
```

## Para cada etapa (ejemplo `pickup1`)

El orden obligatorio es: `pickup1`, `delivery1`, `pickup2`, `delivery2`. Sustituir `pickup1` por la etapa siguiente y usar su carpeta de salida diferente. **Detener robots antes de cada snapshot**.

```bash
STAGE=pickup1
OUT=scripts/resultados/d6/$STAGE
python3 scripts/prepare_hybrid_v2_d6_stage.py \
  --scenario /tmp/swarm_hybrid_123_scenario.json \
  --state scripts/resultados/d6/mision.json --stage "$STAGE" \
  --accept-namespaced-world-odom --radius 0.55 --margin 0.05 \
  --max-speed 1.20 --max-accel 1.00 --iterations 2 --output-dir "$OUT"
```

Esperar `CERTIFIED`. El informe nuevo muestra `assigned_names` para recogidas y `delivered_names` para entregas. Luego **retiming D5.1 y subdivisión para protocolo**, ambos SIN alterar la geometría:

```bash
python3 scripts/retime_hybrid_v2_d51.py \
 --report "$OUT/${STAGE}_live_report.json" --controls "$OUT/${STAGE}_live_controls.npz" \
 --stage "$STAGE" --max-speed 1.20 --max-accel 1.00 \
 --safety-factor 1.08 --subdivisions 4 --output-dir "$OUT/retimed"
python3 scripts/convert_d51_protocol_pieces.py \
 --report "$OUT/retimed/${STAGE}_retimed_report.json" \
 --controls "$OUT/retimed/${STAGE}_retimed_controls.npz" \
 --stage "$STAGE" --max-speed 1.20 --max-accel 1.00 \
 --output-dir "$OUT/protocol"
```

El conversor puede **rechazar** un plan: no relajar límites y no mover. Ejecutar un DRY RUN:

```bash
python3 scripts/run_hybrid_v2_stage_ros.py \
 --report "$OUT/protocol/${STAGE}_protocol_report.json" \
 --controls "$OUT/protocol/${STAGE}_protocol_controls.npz" \
 --stage "$STAGE" --scenario /tmp/swarm_hybrid_123_scenario.json
```

Solo si `PRECHECK OK` y `DRY RUN OK`, se ha comprobado el mundo, escenario y huellas, y hay supervisión presencial: ejecutar el MISMO comando con `--robot-radius 0.55 --operational-margin 0.05 --world-verified --execute`. Para medir, iniciar D.5 recorder en otra terminal con una carpeta **distinta por etapa** antes de ejecutar y analizar después.

**Ojo**: el plan de cada etapa se calcula a partir de la posición observada, no de la meta nominal de la etapa previa. Si `inicio diferente` o `path_follower` no detectado, NO ejecutar; resolver/replanificar como en D4/D5. No aumentar tolerancias.

## Confirmar una etapa

**NO confirmar por un simple estado `done`.** Confirmar tras comprobar en Gazebo:
- los tres controladores `done`, el robot detenido y ningún evento de choque;
- recogida física real y eliminación/marcación de las tres basuras asignadas (`pickup`) o entrega real de los tres objetos en depósitos correctos (`delivery`);
- las entidades de mundo y el escenario continúan siendo coherentes con el mapa usado para planificar la siguiente etapa.

```bash
python3 scripts/d6_mission_state.py --state scripts/resultados/d6/mision.json confirm \
 --stage "$STAGE" \
 --report "$OUT/protocol/${STAGE}_protocol_report.json" \
 --controls "$OUT/protocol/${STAGE}_protocol_controls.npz" \
 --confirm-physical
```

Este comando es un registro **manual**, no un detector físico. Si solo se siguió una trayectoria pero no existe aún mecanismo de recogida/depósito integrado, detener D.6 en esa etapa. Después repetir la sección anterior con `STAGE=delivery1`, `STAGE=pickup2`, `STAGE=delivery2` y sus respectivos `OUT`.

Para inspeccionar el estado:

```bash
python3 scripts/d6_mission_state.py --state scripts/resultados/d6/mision.json show
```

## Supuestos y límites

- La implementación presupone exactamente tres robots y tres basuras por ronda.
- La ocupación lógica `bridge.confirm_collected` se reconstruye desde las confirmaciones manuales. **No elimina modelos Gazebo**.
- Reportes D.4.1 mantienen `mission_stage`, `odom_xy`, radio/margen y escenario exigidos por el preflight D.4.2.
- D.5.1 sigue con límites 1.20 m/s y 1.00 m/s²; la temporalización no uniforme queda en pausa.
- `scenario_sha256` detecta modificaciones al archivo JSON entre etapas; no prueba por sí solo que Gazebo refleja ese archivo.
