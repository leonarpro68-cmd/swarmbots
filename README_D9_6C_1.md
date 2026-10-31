# SwarmBots D.9.6C.1 — Controlador en modo sombra (sin publicar comandos)

## Alcance

Nuevo archivo `swarm_ws/scripts/d96c1_shadow.py`, prueba `swarm_ws/scripts/tests/test_d96c1_shadow.py`. Los códigos D.8, D.9.4, D.9.5, D.9.6A y D.9.6B **no se sobrescriben**. No hay publicadores ROS (`create_publisher`) ni comandos reales de movimiento.

Lee `d95_preflight` y los NPZ convertidos D.9.4 mediante `d96a_readonly_follower.load_stage`, que comprueba hashes SHA-256. Se suscribe con QoS compatible con Jazzy a `/clock` y a `/summit0/odom`, `/summit1/odom`, `/summit2/odom`. Registra comandos **hipotéticos** `vx_body,vy_body` y motivos de bloqueo, con velocidad máxima y pendiente de comando acotadas. La ley candidata es `v_world = v_ref + kp * (p_ref - p_odom)`, transformada con yaw odométrico a ejes del Summit; **no implementa ni certifica el controlador definitivo**.

## Instalación

En el anfitrión, extraer en el directorio raíz del proyecto:

```bash
cd ~/swarmbots
unzip -o ~/Descargas/swarm_D9_6C_1_sombra_sin_movimiento.zip -d ~/swarmbots
```

Dentro del contenedor:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PWD/scripts:$PYTHONPATH"
python3 scripts/tests/test_d96c1_shadow.py
```

## Ejecución: se necesitan los robots, NO basta el mundo vacío

El mundo `gz sim -r -s empty.sdf` **solo sirve para D.9.6B**. El shadow necesita odometría real de 3 robots, y las posiciones iniciales deben coincidir con las referencias del `seed_100_run_007`. No ejecutes otra semilla esperando que las posiciones coincidan.

Con Gazebo y ROS bridge activos, verifica:

```bash
ros2 topic list | grep -E '/summit[012]/odom|/clock'
ros2 topic echo /summit0/odom --once
```

Con la escena **de la semilla 100 correctamente montada y robots detenidos**, ejecutar:

```bash
python3 scripts/d96c1_shadow.py \
  --run scripts/resultados/d8/seed_100_run_007 \
  --stage pickup1
```

Genera `d96c1_shadow/pickup1_shadow_report.json` y `pickup1_SHADOW_NO_COMMANDS.csv`. Estados importantes: `COMPLETE_SHADOW_ONLY`, `ABORT_NO_THREE_ODOM`, `BLOCKED_TRACKING_ERROR`, `BLOCKED_REFERENCE_FRAME_MISMATCH`, `BLOCKED_STALE_ODOM`.

**Advertencia:** sin un controlador que mueva los robots, el error de seguimiento crecerá respecto a la referencia: el modo sombra debe generar `BLOCKED_TRACKING_ERROR`. `COMPLETE_SHADOW_ONLY` solo significa que terminó el horizonte del reloj; NO certifica seguimiento ni puede autorizar ejecución. La odometría tiene que estar expresada en `map` para comparación directa; no se implementa TF en esta versión. Si aparece `odom` o `rb_*/odom`, no se debe cambiar `--reference-frame` a ciegas; es necesario transformar o verificar las poses primero.

## Siguiente validación

Revisar que el nodo reciba odometría y produzca comandos hipotéticos finitos y bien limitados cuando la referencia coincida con la posición observada; estudiar respuesta con seguimiento real bajo controlador D.8 sin ejecutar simultáneamente dos controladores. **No autorizar movimiento D.9.6C.2 hasta integrar TF, arranque coordinado y supervisión real.**
