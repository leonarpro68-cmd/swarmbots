# SwarmBots — Fase 5: seguimiento Bézier sincronizado en ROS 2 (experimental)

**Rama:** `feature/cbs-hybrid-refinement`. Continúa las Fases 1–4.

**Alcance:** primera integración en **ROS 2 Jazzy / Gazebo Harmonic** para **tres Summit**, únicamente **navegación hacia metas de recogida**, SIN acercamiento RGB-D, agarre, transporte, depósitos ni automatización de las dos rondas.

**Estado de verificación:** pruebas puras Python y pipeline de planificación ejecutados offline en el entorno de desarrollo. **Todavía NO probado con Gazebo real en el ordenador del usuario**. La ejecución en ROS debe verificarse y puede abortar si los controladores no respetan las tolerancias.

## Arquitectura

- `hybrid_refinement/timed_protocol.py`: protocolo JSON `swarmbots.timed_bezier.v1` sobre `std_msgs/String`. Verifica puntos Bézier, continuidad C², datos finitos, marcos y **cotas analíticas** de velocidad/aceleración. Curvas con seis puntos por pieza, duración compartida y `start_time_s` absoluto de `/clock`.
- `path_follower_grasp.py`: **extensión OPCIONAL** del seguidor existente. Sigue siendo el **único publicador de `cmd_vel`** del robot; genera feedforward de Bézier + error P mundial transformado al marco cuerpo de Summit mecanum y limita velocidad y aceleración del comando. Además, detiene el robot y publica `failed` ante error excesivo, odometría vencida, salto de reloj, parada de `/clock`, cambios de frame o conflicto con otros modos de control.
- `hybrid_refinement/ros_hybrid_demo.py`: coordinador ROS que espera las tres odometrías reales y descubrimiento DDS, alinea secuencialmente a vértices libres (comprobación de separación frente a los demás), ejecuta `PlanningBridge.plan_pickup` con el escenario real, construye corredores, optimiza, refina, escala temporalmente, publica las tres referencias con el **mismo tiempo de inicio futuro**, exige tres ACK `armed`, supervisa separación y zonas obstáculo según el grafo dinámico, y cancela a todos ante un fallo.
- `swarm_cbs_grasp_base.launch.py`: nuevo argumento `hybrid_enabled` (`false` por defecto). **La misión automática v1.1 no se modifica**.

Se usan tópicos relativos `/summitN/hybrid_trajectory/command` y `/summitN/hybrid_trajectory/status` (`std_msgs/String`), además de `/summitN/odom`, `/summitN/planned_path`, `/summitN/cmd_vel` y `/clock`. Los estados son `armed`, `running`, `done`, `failed`, `cancelled` e incluyen `session_id` para descartar respuestas antiguas.

**Limitaciones de seguridad:** la validación geométrica presupone el mapa/escenario cargado y que las coordenadas **numéricas** de odometría coinciden con el sistema mundial usado por el CBS, como en los ejecutores actuales. `frame_id` distinto cancela. Las tolerancias de seguimiento + supervisión disminuyen el riesgo, pero **parar robots al detectar un fallo no garantiza ausencia de colisión por inercia, latencia o errores no modelados**. Tampoco se han validado aquí masas, fricción, carga o límites de los motores del Summit. No usar con personas cerca, ni en robots físicos.

## Instalar y probar

Desde Ubuntu (host): descomprimir la entrega Fases 1–5 en la raíz `~/swarmbots`. Verificar primero `git status`; no sobrescribir modificaciones propias sin copia.

```bash
cd ~/swarmbots
unzip -o ~/Descargas/cbs_hybrid_fases1_a_5.zip -d ~/swarmbots
export PYTHONPATH="$PWD/swarm_ws/src/swarm_behavior:$PYTHONPATH"
python3 -m pytest -q swarm_ws/src/swarm_behavior/test/test_hybrid_*.py
```

**Resultado esperado en entorno de desarrollo:** `43 passed` (a octubre de 2026). Esto no comprueba ROS en Gazebo.

### Preparar ROS 2 Jazzy dentro de `swarm_dev`

```bash
# Host
cd ~/swarmbots
docker start swarm_dev  # solo si estuviese apagado
docker exec -it swarm_dev bash

# Contenedor, en el workspace montado por SwarmBots
source /opt/ros/jazzy/setup.bash
cd ~/swarm_ws
colcon build --symlink-install --packages-select swarm_behavior swarm_worlds
source install/setup.bash
python3 -c "import scipy, numpy, rclpy; print('Dependencias OK')"
```

Si tu Docker monta el workspace en otra ubicación, entrar al directorio real que contiene `src/` antes de compilar. **No usar `./run_swarmbots.sh` en esta prueba:** ese script inicia la misión v1.1 y compite por los objetivos de navegación.

### Preparación de escenario reproducible, una sola vez (contenedor)

```bash
cd ~/swarm_ws
python3 scripts/escenario_aleatorio_swarm.py --seed 123 \
  --output /tmp/swarm_hybrid_123_scenario.json
python3 scripts/swarm_cbs_grasp_prepare.py \
  --scenario /tmp/swarm_hybrid_123_scenario.json \
  --world "$(ros2 pkg prefix swarm_worlds)/share/swarm_worlds/worlds/tugbot_warehouse.sdf" \
  --output /tmp/swarm_hybrid_123_world.sdf
```

Abrir **tres terminales dentro del mismo contenedor** o sesiones equivalentes, siempre con ROS y `install/setup.bash` sourceados:

**Terminal A — Gazebo y seguidores con modo híbrido activado:**

```bash
ros2 launch swarm_worlds swarm_cbs_grasp_base.launch.py \
  seed:=123 \
  scenario:=/tmp/swarm_hybrid_123_scenario.json \
  world_sdf:=/tmp/swarm_hybrid_123_world.sdf \
  hybrid_enabled:=true
```

**Terminal B — bridge `/clock`:**

```bash
ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
```

**Terminal C — prueba de CBS + híbrido:**

```bash
ros2 run swarm_behavior swarm_hybrid_ros_demo \
  --scenario /tmp/swarm_hybrid_123_scenario.json \
  --iterations 2 --max-speed 0.40 --max-accel 0.35 \
  --tracking-error 0.16
```

**Qué debería ocurrir:** espera DDS y tres odometrías → alineación secuencial → CBS desde los vértices reales alineados → optimización/refinamiento → tres respuestas `armed` → inicio futuro simultáneo por reloj `/clock` → seguimiento → tres respuestas `done`, o cancelación general al detectarse un problema.

**Importante:** el CSV/JSON y PNG de Fases 3–4 son reportes offline. **No publicarlos directamente** en `cmd_vel` ni reenviar `planned_path` mientras haya una curva activa.

### Diagnóstico

```bash
ros2 topic echo /summit0/hybrid_trajectory/status
ros2 topic echo /summit0/odom --once
ros2 node list | grep path_follower
ros2 topic info -v /summit0/cmd_vel
```

Se esperan exactamente los nodos seguidores previstos; **ningún nodo adicional debe publicar `cmd_vel`**. Si la demo falla o se interrumpe, detener Gazebo y volver a iniciar un mundo limpio antes de repetir. No relanzar la misión automática sobre el mundo parcialmente recorrido.

## Pruebas pasadas y pendientes

- [x] Suite offline existente (30 casos) sin regresiones.
- [x] Pruebas del protocolo temporizado (13 casos): serialización, límites, continuidad, evaluación y cancelación.
- [x] Ensayo offline con escenario seed=123 de tres Summit: CBS → corredores → Bézier → refinamiento → mensajes temporizados aceptados por el validador. **No representa ejecución ROS.**
- [ ] Construcción `colcon` dentro de `swarm_dev`.
- [ ] Confirmar nombres/marcos de `/summitN/odom`, ACKs y salida de `cmd_vel`.
- [ ] Primera navegación real de tres Summit en Gazebo, sin colisión y sin abortos.
- [ ] Medir error de seguimiento, distancia real mínima, tiempos y paradas de emergencia.
- [ ] Después: integrar modo híbrido a los ejecutores de recogida/entrega conservando RGB-D, ACK físicos y fail-safe.

Si la demo falla con `Robot no alineado`, `Frame` o error de seguimiento, conservar el log exacto y ajustar SOLO tras verificar odometrías, TF y comportamiento de Gazebo. Evitar subir arbitrariamente umbrales de seguridad.
