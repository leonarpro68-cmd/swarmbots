# SwarmBots Hybrid V2 — D.5: medición del seguimiento en Gazebo

Este paquete agrega **dos scripts**, sin modificar el ejecutor D.4.2, CBS, Bézier ni ningún controlador.

- `swarm_ws/scripts/record_hybrid_v2_d5.py`: observador ROS 2 pasivo. **Debe iniciarse antes** de `run_hybrid_v2_stage_ros.py --execute`. Escucha `/clock`, la odometría, los mensajes `hybrid_trajectory/command` y `hybrid_trajectory/status`; no publica comandos de movimiento.
- `swarm_ws/scripts/analyze_hybrid_v2_d5.py`: análisis offline del CSV; produce `tracking_comparison.png` y `metrics.json`.

## Instalación

Fuera de Docker:

```bash
cd ~/swarmbots
unzip -o ~/Descargas/swarm_hybrid_v2_D5_medicion.zip -d ~/swarmbots
```

Dentro de Docker, **con Gazebo y seguidores híbridos activos**:

### Terminal 1: observador (antes de ejecutar)
```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PYTHONPATH"
python3 scripts/record_hybrid_v2_d5.py \
  --output-dir scripts/resultados/d5_pickup1_seed123 \
  --sample-hz 10 \
  --timeout-s 240
```

### Terminal 2: preflight y ejecución

Usar el **plan live** válido y el comando D.4.2 habitual. Primero repetir preflight sin `--execute`; después, solo si sigue pasando y el mundo está verificado, lanzar:

```bash
python3 scripts/run_hybrid_v2_stage_ros.py \
  --report scripts/resultados/d4_live/pickup1_live_report.json \
  --controls scripts/resultados/d4_live/pickup1_live_controls.npz \
  --stage pickup1 \
  --scenario /tmp/swarm_hybrid_123_scenario.json \
  --robot-radius 0.55 \
  --operational-margin 0.05 \
  --world-verified --execute
```

**No vuelvas a ejecutar una misión con robots en sus metas usando el plan de recogida anterior.** Reinicia el mismo mundo con la distribución inicial y vuelve a generar `d4_live` desde odometría, o registra la siguiente misión que corresponda. No aumentes tolerancias para facilitar el seguimiento.

### Terminal 3: generar resultados

Después de finalizar la observación:

```bash
python3 scripts/analyze_hybrid_v2_d5.py \
  --input-dir scripts/resultados/d5_pickup1_seed123 \
  --min-separation 1.15
```

Genera:
```
scripts/resultados/d5_pickup1_seed123/
  odom_tracking.csv
  events.jsonl
  tracking_comparison.png
  metrics.json
```

## Métricas

- RMSE, media y máximo error XY (m) de cada robot durante el intervalo activo.
- Velocidad medida en odometría (m/s), duración observada y estados finales.
- Mínima distancia entre pares según muestras alineadas temporalmente (hasta 0.08 s de diferencia).
- Gráfico: curvas Bézier enviadas vs odometría real, y error de seguimiento en el tiempo.

## Limitaciones

- Se evalúan **muestras**, no distancias continuas ni colisiones exactas del SDF. La seguridad dinámica requiere comprobar también la geometría real y los obstáculos.
- Los tiempos del observador provienen de `/clock` y son los de las **mismas órdenes efectivamente publicadas**, no un plan reconstruido con una hora ficticia.
- Se debe iniciar el observador **antes** de ejecutar, porque la orden ROS utiliza QoS VOLATILE y puede perderse si se suscribe tarde.
- `A message was lost` u otras pérdidas de odometría reducen la calidad de las estimaciones.
- `complete` solo es verdadero cuando llegan tres estados `done` de la sesión observada.
