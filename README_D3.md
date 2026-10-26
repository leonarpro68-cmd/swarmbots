# Hybrid V2 — D.3 adaptador de protocolo ROS 2 (primera parte)

Este parche añade un adaptador **sin ROS** que convierte los puntos de control certificados de D.2 a los mensajes JSON `swarmbots.timed_bezier.v1` usados en la fase híbrida V1. **No publica comandos**, ni altera launch ni `path_follower_grasp.py`.

## Instalación

Descomprimir en `~/swarmbots` (Ubuntu host). Dentro de Docker:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PYTHONPATH"
python3 -m pytest -q src/swarm_behavior/test/test_hybrid_v2_d3.py
```

## Uso de exportación en seco

D.2 debe haberse ejecutado con `--controls-dir scripts/resultados/controles_hybrid_v2` para guardar NPZ de etapas certificadas. Ejemplo:

```bash
python3 scripts/export_hybrid_v2_ros_protocol.py \
  --report scripts/resultados/mision_123_hybrid_v2_r055.json \
  --stage pickup1 \
  --controls scripts/resultados/controles_hybrid_v2/hybrid_v2_123_pickup1.npz \
  --frames odom odom odom \
  --start-time 1000 \
  --output-dir scripts/resultados/d3_dryrun
```

`--start-time 1000` es un marcador para **solo inspección**; NO se debe publicar. Para ejecución real se necesita `/clock` futuro leído del sistema, `frame_id` real por robot, validación del inicio desde odometría fresca, coordinación de armado, ausencia de otros publicadores y manejo de fallos/cancelación. `--frames odom odom odom` son ejemplos, no sustituyen los frame_id reales.

La exportación controla `CERTIFIED`, `dt_s`, horizonte compartido, finitud, continuidad y límites dinámicos mediante `decode_command`. **No recertifica la geometría**: se debe conservar intacta la relación entre JSON, NPZ, escenario, obstáculos y margen 0.05 m. Esta versión no se ha probado en Gazebo.

## Importante: caso del experimento de radio 0.55 m

El script original `experimento_radio_hybrid_v2.py` **no guardaba los NPZ**. Esta actualización añade `--controls-dir` y usa `mission.export_report(..., controls_dir=...)`; para generar esos datos hay que repetir la planificación una vez:

```bash
python3 scripts/experimento_radio_hybrid_v2.py \
  --seed 123 --radius .55 --margin .05 --iterations 2 \
  --output scripts/resultados/mision_123_hybrid_v2_r055.png \
  --json-output scripts/resultados/mision_123_hybrid_v2_r055.json \
  --controls-dir scripts/resultados/controles_hybrid_v2_r055
```

A continuación ejecutar el exportador usando la ruta real resultante en `controls_file` del JSON o:

```bash
python3 scripts/export_hybrid_v2_ros_protocol.py \
  --report scripts/resultados/mision_123_hybrid_v2_r055.json \
  --stage pickup1 \
  --controls scripts/resultados/controles_hybrid_v2_r055/hybrid_v2_123_pickup1.npz \
  --frames odom odom odom \
  --start-time 1000 \
  --output-dir scripts/resultados/d3_dryrun
```
