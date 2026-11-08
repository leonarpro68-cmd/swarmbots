# SwarmBots — ROS 2 Jazzy + Gazebo Harmonic

Workspace multimapa con 3 Summit, 6 basuras y 3 depósitos. Mapas:
`supermarket`, `tugbot_warehouse`, `colored_warehouse`, `pallet_warehouse`,
`parking_lot`, `rubber_factory` y `electrical_substation`.

## Iniciar Docker (Ubuntu anfitrión)

```bash
cd ~/swarmbots/swarm_ws/docker
xhost +local:docker
docker compose up -d swarm
docker exec -it swarm_dev bash
```

## Dentro de Docker

```bash
cd /home/dev/swarm_ws
source /opt/ros/jazzy/setup.bash
# Solo al crear el workspace o tras cambiar paquetes/launch de ROS:
colcon build --packages-select swarm_worlds --symlink-install
source install/setup.bash
```

## Preparar y ejecutar una misión

```bash
python3 scripts/supermarket_milp/run_offline_seed.py --seed 2026 --map rubber_factory
python3 scripts/supermarket_seed_pipeline/run_full_offline.py --seed 2026 --map rubber_factory
# Solo si aparece OFFLINE_FULL_STATIC_PASS_NOT_GAZEBO_CERTIFIED:
ros2 launch swarm_worlds supermarket_full_prepared.launch.py seed:=2026 map:=rubber_factory assignment_mode:=milp
```

El modo `fixed` usa `--assignment-mode fixed` en la planificación offline y
`assignment_mode:=fixed` en el launch. Los escenarios, resultados MILP y planes
se guardan separados por mapa y semilla. No cambies los hashes ni estados de
certificación para forzar una misión rechazada.

## Pruebas

```bash
python3 -m unittest discover -s tests -p 'test_*.py' -v
python3 -m unittest discover -s scripts/supermarket_milp/tests -p 'test_*.py' -v
```

El preflight ROS común reside en `scripts/supermarket_seed_pipeline/ros_preflight.py`.
La semilla 100 ya no tiene un launch ni un ejecutor especiales. Se conserva
`data/seed_100.json` únicamente como fixture de regresión A*/MILP.

Más detalles en `scripts/supermarket_seed_pipeline/README.md`.

## Launch de CBS históricos

Se conserva `swarm_cbs_base.launch.py` **solo** para probar tres Summit y sus seguidores de ruta, sin planificador CBS, basura ni pinza. Por ejemplo:

```bash
ros2 launch swarm_worlds swarm_cbs_base.launch.py world:=tugbot_warehouse
```

Los launch 7B.2/7B.3 (`swarm_cbs_scenario`, `swarm_cbs_mission`) y el experimento independiente `swarm_cbs_grasp_base` se archivaron en `archive/legacy_cbs_launches/` junto a sus dos auxiliares. Esos flujos requerían scripts de generación/preparación ya ausentes en el workspace y no forman parte del pipeline multimapa actual. Los documentos antiguos bajo `docs/cbs/` son históricos y pueden contener comandos obsoletos.

## Ejecutor CBS en vivo 6B–6E (archivado)

El antiguo `swarm_mission` (6E) y los ejecutores 6B/6C/6D se han archivado en
`archive/legacy_ros_mission/`. Su entrada `console_scripts` ya no se instala.
El núcleo compartido (`bridge`, `mission_core`, `ros_plan_helpers`, `ros_planning_probe`),
las demostraciones híbridas y sus bibliotecas permanecen en `src/swarm_behavior/`.
Esta limpieza **no afecta** a CBS offline, MILP, Bézier/V3 ni a la misión preparada.
Consulta `archive/legacy_ros_mission/README.md` para recuperar el prototipo.
