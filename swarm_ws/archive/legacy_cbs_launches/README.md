# Launch históricos CBS: fases 7B y experimento de agarre

**Estado: archivado, no operativo con el workspace multimapa actual.**

Estos archivos proceden de los ensayos 7B.2 y 7B.3, además de un experimento
separado de pinza/percepción. Se guardan para referencia y recuperación; no
están instalados por `swarm_worlds` y no deben invocarse mediante `ros2 launch`.

| Archivo | Uso histórico | Por qué no se conserva como activo |
| --- | --- | --- |
| `swarm_cbs_scenario.launch.py` | Tres Summit + seis basuras + tres depósitos, sin CBS | Depende de `swarm_scenario_prepare.py`, que requiere `scripts/escenario_aleatorio_swarm.py` y `scripts/usar_escenario_swarm.py`: ausentes del ZIP actual |
| `swarm_cbs_mission.launch.py` | Misión CBS en vivo 6E | Hereda el preparador anterior y usa el ejecutor histórico `swarm_mission` |
| `swarm_cbs_grasp_base.launch.py` | Prueba aislada de pinza/cámara/agarre | Las instrucciones requieren `scripts/swarm_cbs_grasp_prepare.py` y el generador antiguo, ausentes en el ZIP actual; el mundo debe prepararse manualmente |
| `swarm_scenario_prepare.py` | Generación e inserción de objetos en Gazebo | Solo lo usan los lanzadores históricos anteriores |
| `swarm_mission_start.py` | Arranque de la misión 6E | Solo lo usa `swarm_cbs_mission.launch.py` |

## Qué permanece activo

- `src/swarm_worlds/launch/swarm_cbs_base.launch.py`: prueba aislada de **tres robots con `swarm_path_follower`**, sin CBS ni basura. Sigue disponible.
- `src/swarm_worlds/launch/supermarket_full_prepared.launch.py`: único launch principal recomendado para las misiones offline preparadas, válido para los siete mapas.
- El ejecutor `swarm_mission` también fue archivado posteriormente en `archive/legacy_ros_mission/`, junto con sus dependencias exclusivas y pruebas 6B–6E. No forma parte del paquete ROS 2 activo.

## Recuperación

El instalador del parche crea una copia de seguridad de los archivos antes de moverlos y permite revertirlos.
La recuperación de un launch histórico **no recupera sus scripts auxiliares faltantes**; por eso no equivale a habilitarlo para producción.

## Nota sobre instalados antiguos

Tras actualizar código fuente, reconstruir `swarm_worlds`. Si ROS sigue encontrando alguno de los launch archivados, puede tratarse de un artefacto residual en `install/`; no asumir que representa el código fuente actual.
