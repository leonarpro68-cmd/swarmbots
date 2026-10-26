# D.4.1 — Planificación de recogida 1 desde odometría real

Este paquete **NO MOVILIZA ROBOTS**. Toma una fotografía de `/summit*/odom` y `/clock`, busca los vértices libres más próximos para el CBS y sustituye **únicamente** el extremo inicial métrico del problema SLSQP por la posición exacta observada. Vuelve a construir corredores y certifica desde cero. **Nunca traslada curvas ya certificadas**. Preserva las esperas finales y la metodología V2.

## Instalación

Fuera de Docker, desde `~/swarmbots`:

```bash
unzip -o ~/Descargas/swarm_hybrid_v2_D4_1_odom_replan.zip -d ~/swarmbots
```

Dentro de Docker, con Gazebo ejecutándose:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PYTHONPATH"
python3 -m pytest -q src/swarm_behavior/test/test_hybrid_v2_live_start.py
ros2 topic echo /summit0/odom --once
```

Verifique el valor de `header.frame_id` de **los tres** odom. **No asuma** que `odom` coincide con `map`: el planificador usa coordenadas del mundo y este prototipo no transforma TF. Debe demostrarse la identidad/transformación antes de usarlo para ejecución.

```bash
python3 scripts/replan_hybrid_v2_from_odom.py \
  --scenario /tmp/swarm_hybrid_123_scenario.json \
  --expected-frame odom \
  --radius 0.55 --margin 0.05 --iterations 2 \
  --output-dir scripts/resultados/d4_live
```

Reemplace el valor de `--expected-frame` por el marco real observado. El argumento `--scenario` debe ser el JSON **exacto** usado para generar el mundo de Gazebo en ejecución. **La coincidencia de nombre o seed no basta.**

Obtendrá `pickup1_live_report.json` y, **solo si es CERTIFIED**, `pickup1_live_controls.npz`.

El comando original D.4 seguirá siendo un preflight separado. No active `--execute` aún: falta verificar la equivalencia real de marcos, el estado/identidad del mundo, odometrías al momento de ejecución y supervisión frente a todos los obstáculos. Además, el radio 0.55 m es un valor experimental.

El esquema de CB​​S de nodo más cercano es una aproximación discreta; la diferencia de hasta `--max-snap` debe certificarse geométricamente en el primer tramo. Si no existe solución segura, se informa `REJECTED` y no se generan controles.
