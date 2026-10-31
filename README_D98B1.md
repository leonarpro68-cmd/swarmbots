# D.9.8B.1 — PREORIENTAR → ALINEAR RGB-D → AGARRAR (Gazebo)

## Qué se corrigió
El preflight D.9.8B comprobaba `grasp_radius=0.45 m` **antes** de que los Summit giraran y se aproximaran. Esto era prematuro. El protocolo real D8 en `prueba_alinear_agarrar_cbs.py` es:

1. Desde el **centro del Summit**, comprobar distancia a su basura: `0.55 <= d <= 2.2 m`; el robot debe estar detenido.
2. Girar sobre su sitio hasta el yaw `atan2(y_basura-y_robot, x_basura-x_robot)`, con `/summitN/local_turn/target_yaw` (`Float64`) y comprobar `/summitN/local_turn/result` (`Int8(1)`). Verificar deriva menor de 0.10 m y error yaw menor de 0.10 rad.
3. Activar `visual_grasp.py` con `/summitN/visual_grasp/target` (`PoseStamped`); esperar `/summitN/visual_grasp/result` (`Int8(1)`); el controlador RGB-D **sí aproxima el robot**.
4. **Inmediatamente antes de cerrar**, volver a leer la posición REAL de todas las basuras Gazebo; estimar punta con `palm_forward=.45`; validar `grasp_radius<=.45`, objeto asignado como el más cercano y margen >=.07 m. Confirmar robot parado.
5. Enviar `/summitN/gripper/grasp` (`Empty`), esperar `/summitN/gripper/result` (`Int8(1)`). El `grasp_manager` D8 solo responde 1 cuando su joint informa `attached`.

El script NO ejecuta regreso al vértice, NO crea ni modifica ledger D6, NO hace entregas, ni sustituye a D8. Ejecuta robot por robot. El ACK `Int8` no codifica nombre de basura, por eso se verifica geometría antes del grasp. Se debe verificar visualmente el objeto sujeto y hacer commit de la misión después.

## Paquete
- `swarm_ws/scripts/d98b1_turn_align_grasp.py` (nuevo; depende de `d98b_grasp_preflight.py` ya instalado, incluido aquí para facilitar uso)
- `swarm_ws/scripts/tests/test_d98b1_turn_align_grasp.py` (10 pruebas)

## Instalación
En el host:

```bash
unzip -o ~/Descargas/swarm_D98B1_giro_rgbd_agarre.zip -d ~/swarmbots
```

Dentro de Docker:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/scripts:$PWD/src/swarm_behavior:$PYTHONPATH"
python3 -m unittest discover -s scripts/tests -p test_d98b1_turn_align_grasp.py -v
```

## Vista previa, SIN MOVIMIENTO
Con **la misma sesión Gazebo** donde terminó pickup1_009 (no reiniciar ni mover robots), los tres parados y nodos D8 `swarm_path_follower_grasp`, `visual_grasp` y `grasp_manager` activos:

```bash
python3 scripts/d98b1_turn_align_grasp.py \
  --handoff scripts/resultados/d9_live/seed_100_pickup1_009/d98a/pickup1_handoff.json \
  --world world_demo \
  --robot summit0
```

Esperado: `READY_FOR_TURN_AND_RGBD_ONLY`. Ver `.../009/d98b/pickup1_turn_visual_grasp.json`. Mostrar ángulo objetivo, distancia centro-objeto y aproximación RGB-D requerida. Si `BLOCKED_OR_FAILED`, no continuar.

## Primera prueba ACTIVA: SOLO Summit0, EN GAZEBO
No dejar funcionando D97C watchdog/relay, D8 mission executor ni otro nodo que publique en `/summit0/cmd_vel`. El código exige exactamente **un** publicador `/summit0/cmd_vel`, el `path_follower` D8, y suscriptores para giro/visión/agarre. Comprueba en caso de duda: `ros2 topic info /summit0/cmd_vel -v`.

```bash
python3 scripts/d98b1_turn_align_grasp.py \
  --handoff scripts/resultados/d9_live/seed_100_pickup1_009/d98a/pickup1_handoff.json \
  --world world_demo \
  --robot summit0 \
  --execute-in-gazebo
```

Si se confirma, esperar `GRASP_ACK_RECEIVED_NO_LEDGER_COMMIT` y en `robot_results` `GRASP_MANAGER_ACK_NO_LEDGER_COMMIT`. Detenerse a verificar en Gazebo que `summit0` sujetó `trash_V2_verde`. NO repetir el comando exitoso, porque el objeto ya estaría sujeto. Si falla, revisar reporte antes de volver a ordenar agarre; especialmente si se recibió éxito parcial antes del error.

Para `summit1` y `summit2`, solo después de comprobar el primero, usar `--robot summit1`, `--robot summit2` por separado con el mismo handoff y **la misma sesión**. Evitar `--robot all` si summit0 ya agarró; está destinado a una sesión íntegra sin agarrar nada.

## Limitaciones
- Integración **sin prueba de Gazebo en el entorno del usuario**: solo 10 pruebas unitarias offline; puede requerir adaptación en ROS dependiendo de los nodos efectivamente activos.
- D8 `path_follower_grasp` tiene timeout de giro; `visual_grasp` tiene timeout para la aproximación y `grasp_manager` sólo acepta el ACK del joint.
- Si un proceso de control muere, no hay una protección independiente y certificada para robots físicos; **SIMULACIÓN SOLAMENTE**.
- El mundo/odom debe mantener la equivalencia global XY establecida en esta simulación; no se ha certificado un árbol TF global para todos los casos.
- No se ha certificado la entrega, regreso a vértice ni ledger de misión. Éstos siguen pendientes para la integración completa.
