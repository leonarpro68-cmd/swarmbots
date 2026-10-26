# D.4.1 — Aceptar odometría namespaced verificada en Gazebo

Este es un parche incremental para la instalación D.4.1 existente. **No** mueve robots, modifica la planificación SLSQP ni publica transforms TF.

## Por qué

En `world_demo`, se compararon manualmente posiciones de `/summitN/odom` y `gz model -m summitN -p` para N=0,1,2, y coincidieron horizontalmente. Los frames ROS reales son `summit0/odom`, `summit1/odom`, `summit2/odom`, no el frame literal `odom`.

Se agrega la opción explícita `--accept-namespaced-world-odom`, válida solamente para esta configuración comprobada. **No presume equivalencia de frames en otros mundos** ni certifica orientación, identidad de escenario o odometría futura. El programa verifica frame por robot y guarda `odom_frames` + `odom_frame_policy` en el reporte.

## Instalación

Fuera de Docker:

```bash
cd ~/swarmbots
unzip -o ~/Descargas/swarm_hybrid_v2_D4_1_frames.zip -d ~/swarmbots
```

Dentro de Docker, con Gazebo ejecutándose, ROS 2 y dependencias configuradas:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PYTHONPATH"
python3 -m pytest -q src/swarm_behavior/test/test_hybrid_v2_odom_frames.py
```

## Replanificación segura SIN movimiento

```bash
python3 scripts/replan_hybrid_v2_from_odom.py \
  --scenario /tmp/swarm_hybrid_123_scenario.json \
  --accept-namespaced-world-odom \
  --radius 0.55 \
  --margin 0.05 \
  --iterations 2 \
  --output-dir scripts/resultados/d4_live
```

Si CERTIFIED: `pickup1_live_report.json` y `pickup1_live_controls.npz`. Si REJECTED: revisar mensaje del JSON, no ejecutar comandos de movimiento.

**Requisito adicional pendiente:** demostrar que `/tmp/swarm_hybrid_123_scenario.json` es el JSON **exacto** que generó el mundo en ejecución. La equivalencia de odometría con poses Gazebo **no** demuestra esta identidad de escenario. No usar `--execute` hasta resolverla y completar comprobaciones de inicio fresco y orientación.

Para mundos que utilicen de verdad un frame compartido, el modo anterior `--expected-frame map` sigue disponible; las dos opciones son mutuamente excluyentes.
