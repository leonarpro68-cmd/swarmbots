# SwarmBots — misión física coordinada con CBS (v1.0)

> Estado de referencia: etiqueta Git `swarmbots-physical-v1.0`, commit `1ae53a0`.
> Esta guía documenta el flujo demostrado en conversaciones de validación. Revisa las opciones con `--help` en tu copia del repositorio antes de automatizar.

## Alcance

Sistema ROS 2 Jazzy / Gazebo Harmonic con **tres Summit XLS omnidireccionales**, **seis basuras** (dos de cada color) y **tres depósitos**. La misión se divide en dos rondas de recogida y entrega. La recogida y el transporte emplean planificación multiagente CBS y ejecución por pasos con barreras; la aproximación final emplea cámara de profundidad RGB-D, y el agarre usa pinza y Gazebo `DetachableJoint` con confirmaciones.

La misión es **supervisada**, no completamente desatendida: las entregas requieren aprobación humana y la posición final de cada basura debe verificarse visualmente.

## Componentes importantes

| Función | Ruta relativa al repositorio |
| --- | --- |
| Launch experimental Gazebo | `swarm_ws/src/swarm_worlds/launch/swarm_cbs_grasp_base.launch.py` |
| Generación de escenarios | `swarm_ws/scripts/escenario_aleatorio_swarm.py` |
| Preparación del SDF | `swarm_ws/scripts/swarm_cbs_grasp_prepare.py` |
| Coordinación integral | `swarm_ws/scripts/mision_fisica_completa.py` |
| Coordinación de rondas | `swarm_ws/scripts/mision_fisica_ronda1_v411.py`, `swarm_ws/scripts/mision_fisica_ronda2_v41.py` |
| Recogida CBS ronda 1 | `swarm_ws/scripts/ros_recogida_cbs_v2.py` |
| Recogida CBS ronda 2 | `swarm_ws/scripts/ros_recogida_ronda2_cbs_v2.py` (**contiene mejoras V3 en la etiqueta estable**) |
| Transporte CBS | `swarm_ws/scripts/ros_transporte_fisico_cbs_v2.py`, `swarm_ws/scripts/ros_transporte_ronda2_cbs_v2.py` |
| Alineación visual | `swarm_ws/src/swarm_behavior/swarm_behavior/visual_grasp.py` (mejoras V2) |
| Administrador de agarre | `swarm_ws/src/swarm_behavior/swarm_behavior/grasp_manager.py` (mejoras V2) |
| Validación por semilla | `swarm_ws/scripts/validar_semillas_swarm.py` |

Los nombres históricos `v2`, `v3` y `v41` son propios del proceso de desarrollo; la **etiqueta Git**, y no el nombre del archivo, identifica la versión de referencia.

## Preparación de una semilla

Los siguientes comandos se ejecutan **dentro del contenedor** `swarm_dev`. Sustituye `456` por la semilla deseada y usa una simulación nueva para cada intento.

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

python3 scripts/validar_semillas_swarm.py prepare \
  --seed 456 \
  --world-original "$(ros2 pkg prefix swarm_worlds)/share/swarm_worlds/worlds/tugbot_warehouse.sdf"
```

Esto prepara los archivos esperados bajo `/tmp/swarm_validacion/seed_456/`.

**Terminal 1: Gazebo**

```bash
ros2 launch swarm_worlds swarm_cbs_grasp_base.launch.py \
  seed:=456 \
  scenario:=/tmp/swarm_validacion/seed_456/scenario.json \
  world_sdf:=/tmp/swarm_validacion/seed_456/world.sdf
```

**Terminal 2: reloj simulado**

```bash
ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
```

**Terminal 3: misión y evaluación**

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/validar_semillas_swarm.py run --seed 456
```

Responde a las confirmaciones supervisadas solo después de comprobar Gazebo.

## Seguridad y reproducibilidad

- No reiniciar una misión fallida sobre un Gazebo parcialmente modificado.
- No borrar ni sobrescribir los registros anteriores; archivarlos primero.
- El validador se niega a reutilizar archivos de estado de una ejecución existente.
- El costo CBS (coste discreto) no equivale necesariamente a distancia física recorrida.
- La duración de pared incluye las pausas de autorización humana.
- `JOINT CONFIRMADO` y `DETACH CONFIRMADO` acreditan las confirmaciones recibidas; la ubicación correcta en el depósito requiere comprobación adicional.

## Resultados de la primera campaña funcional

| Semilla | Configuración | Resultado registrado | Tiempo de pared |
| --- | --- | --- | ---: |
| 123 | Referencia inicial | Dos rondas completadas | No registrado |
| 456 | Controladores anteriores | Fallo de agarre/alineación en segunda ronda | 270,27 s (hasta el fallo) |
| 456 | Controladores físicos V2 | Seis ACK de agarre y seis de liberación | 451,37 s |
| 789 | Recogida ronda 2 V2 | Timeout en `STEP_1` | 262,33 s (hasta el fallo) |
| 789 | Controladores V2 + ejecutor V3 | Seis ACK de agarre y seis de liberación | 440,54 s |
| 2026 | Controladores V2 + ejecutor V3 | Seis ACK de agarre y seis de liberación | 584,70 s |

**Limitación:** las configuraciones no son idénticas entre todas las semillas y no hay repeticiones suficientes para estimar una tasa de éxito estadística. Los resultados documentan **validación funcional exploratoria**, no prueba concluyente de robustez. Los resultados `completed_pending_visual_check` no equivalen a confirmación visual.

## Rutas de registros

`/tmp/swarm_validacion/seed_<SEMILLA>/mission_console.log` y `/tmp/swarm_validacion/seed_<SEMILLA>/run.json`.

Antes de reiniciar el contenedor o limpiar `/tmp`, copiar los escenarios y reportes fuera del contenedor. Los archivos bajo `/tmp` pueden ser efímeros.
