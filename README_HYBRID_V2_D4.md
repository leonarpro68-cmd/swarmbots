# SwarmBots Hybrid V2 — D.4: primera ejecución supervisada (etapa única)

**Alcance:** ejecutor ROS 2 separado, no modifica el optimizador ni `path_follower_grasp.py`. Usa el JSON y NPZ **certificados y correspondientes al mismo escenario**. Por defecto realiza solo un preflight, sin publicar movimientos.

## Instalación

Desde Ubuntu (fuera del contenedor):

```bash
cd ~/swarmbots
unzip -o ~/Descargas/swarm_hybrid_v2_D4_preflight_ros.zip -d ~/swarmbots
```

Dentro de Docker:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PYTHONPATH"
python3 -m py_compile scripts/run_hybrid_v2_stage_ros.py
```

## Preparar los cuatro archivos NPZ, solo si aún no existen

```bash
python3 scripts/experimento_radio_hybrid_v2.py \
  --seed 123 --radius 0.55 --margin 0.05 --iterations 2 \
  --output scripts/resultados/mision_123_hybrid_v2_r055.png \
  --json-output scripts/resultados/mision_123_hybrid_v2_r055.json \
  --controls-dir scripts/resultados/controles_hybrid_v2_r055
```

## Gazebo

Lanzar la simulación con **el mismo escenario de semilla 123 que aparece en el JSON**, el launch `swarm_cbs_grasp_base.launch.py` con `hybrid_enabled:=true` y `hybrid_max_speed:=1.20`. Evitar lanzar `run_swarmbots.sh` u otro coordinador al mismo tiempo. El SDF de mundo debe corresponder al mismo escenario.

**Atención: el launch actual elige poses aleatorias de los Summit por semilla, que pueden diferir de las posiciones offline del reporte.** En ese caso, D.4 aborta deliberadamente con mensaje de error de posición inicial. No se debe forzar tolerancia ni transformar arbitrariamente los controles: el siguiente trabajo será regenerar el plan para la odometría real o alinear de forma segura, como ya hace el coordinador V1.

## Prueba sin movimiento (recomendada primero)

```bash
python3 scripts/run_hybrid_v2_stage_ros.py \
  --report scripts/resultados/mision_123_hybrid_v2_r055.json \
  --controls scripts/resultados/controles_hybrid_v2_r055/hybrid_v2_123_pickup1.npz \
  --stage pickup1
```

Comprueba odometría reciente, velocidad casi cero, `frame_id`, reloj `/clock`, existencia de suscriptores, controles y tiempo del protocolo, y coincidencia geométrica del inicio. No publica comandos `start` sin la opción `--execute`.

## Ejecutar una sola etapa (solo en entorno aislado y tras revisar los controles físicos)

El programa admite `--execute` para publicar comandos y esperar tres ACK, estados `done`, y supervisar distancias reales entre robots. **No contiene todavía un validador en línea de colisiones contra todos los obstáculos del mundo, ni alinea las poses iniciales, ni lleva a cabo las cuatro etapas de forma automática**. Por eso esta versión es una integración inicial, no el cierre definitivo de D.4. Asegurar zona despejada, supervisión presencial y mecanismo de parada de la simulación.

```bash
python3 scripts/run_hybrid_v2_stage_ros.py \
  --report scripts/resultados/mision_123_hybrid_v2_r055.json \
  --controls scripts/resultados/controles_hybrid_v2_r055/hybrid_v2_123_pickup1.npz \
  --stage pickup1 --execute
```

**No ejecutar** si no coinciden escenario, SDF y posiciones iniciales, o si el radio circular de 0.55 m no ha sido contrastado con la colisión física del Summit. El valor 0.55 m sigue siendo experimental.

## Limitaciones / siguiente paso

- Validar huella real del Summit y ocupación real de depósitos, columnas y basuras.
- Añadir monitor de distancia robot-obstáculo con mapa dinámico por etapa.
- Unir planificación V2 a `PlanningBridge` desde la **odometría real** con alineación segura y objetivos de misión.
- Comprobar que el JSON certificado y NPZ proceden de idéntica corrida (actualmente se valida duración pero no hash criptográfico del plan).
- Registrar odometría real y comparar trayectorias ejecutadas con Bézier planificadas.
