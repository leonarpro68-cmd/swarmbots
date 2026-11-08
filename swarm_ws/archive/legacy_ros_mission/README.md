# Ejecutor histórico CBS en vivo: etapas 6B–6E

**Estado: archivado; no se instala ni se ejecuta como misión ROS 2 actual.**

La misión 6E (`ros_full_mission.py`) dirigía tres Summit, planificaba CBS en línea,
retiraba modelos de basura mediante el servicio de Gazebo y coordinaba dos rondas
de recogida y entrega. Su flujo dependía del antiguo launch
`swarm_cbs_mission.launch.py`, que fue archivado antes de esta limpieza.
No usa los planes preparados certificados ni reproduce el protocolo D8 del pipeline vigente.

## Código conservado sin cambios

`swarm_planning/` conserva estos cinco archivos con sus bytes originales:

- `ros_full_mission.py` — fase 6E, orquestación de dos rondas.
- `ros_pickup_delivery.py` — fase 6D, entrega tras una ronda de recogida.
- `ros_pickup_removal.py` — fase 6C, retirada confirmada de modelos en Gazebo.
- `ros_pickup_executor.py` — fase 6B, navegación y barreras de llegada.
- `gazebo_removal.py` — utilidades de eliminación de modelos.

`test/` conserva cuatro pruebas propias de esos módulos. Los tests fueron
archivados junto con el código porque ya no se instalan sus dependencias.

## Código que permanece activo

- `ros_plan_helpers.py`: compartido con la demostración híbrida ROS.
- `ros_planning_probe.py`, `bridge.py`, `mission_core.py`: diagnóstico y núcleo
  del prototipo original, también utilizados por pruebas de regresión.
- `geometry.py`, `cbs.py`, `assignment.py`, `dynamic_map.py`, `scenario.py` y
  bibliotecas `hybrid_refinement` y `hybrid_v2`: no se modificaron.
- `swarm_path_follower`, `swarm_path_follower_grasp`, `visual_grasp` y
  `grasp_manager`: ejecutables utilizados por flujos actuales.

## Recuperación experimental

Los archivos archivados **no son directamente ejecutables** desde su carpeta,
porque utilizan importaciones relativas de paquete. Para reactivar el prototipo:

1. Restaurar los cinco módulos bajo
   `src/swarm_behavior/swarm_behavior/swarm_planning/`.
2. Restaurar las cuatro pruebas bajo `src/swarm_behavior/test/`.
3. Recuperar en `src/swarm_behavior/setup.py` la entrada
   `swarm_mission = swarm_behavior.swarm_planning.ros_full_mission:main`.
4. Reinstalar con `colcon build --packages-select swarm_behavior --symlink-install`
   y validar en ROS 2/Gazebo.
5. Los launch antiguos tienen requisitos de scripts faltantes: recuperar el
   ejecutor por sí solo **no** restaura la misión 7B original.

El instalador de este parche permite la reversión íntegra de estos cambios.
