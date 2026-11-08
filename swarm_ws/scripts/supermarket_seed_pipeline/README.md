# SwarmBots — pipeline offline multimapa y ejecución preparada

Este módulo forma parte del workspace **SwarmBots instalado**. No se instala por separado. Se utiliza desde `/home/dev/swarm_ws` **dentro del contenedor Docker**.

El pipeline procesa un escenario y su asignación (`milp` o `fixed`) mediante CBS, auditoría de corredores, Bézier/Hybrid V2, refinamiento geométrico, reloj V3, verificaciones independientes y preflight estático D8. Las verificaciones **offline no certifican** seguimiento, agarre ni entrega física en Gazebo.

## Preparación del entorno

Desde Ubuntu anfitrión (el workspace se monta en Docker como `/home/dev/swarm_ws`):

```bash
cd ~/swarmbots/swarm_ws/docker
xhost +local:docker
docker compose up -d swarm
docker exec -it swarm_dev bash
```

Dentro del contenedor:

```bash
cd /home/dev/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

`install/setup.bash` requiere que el workspace haya sido compilado previamente. Si es una copia nueva y todavía no existe, ejecuta `colcon build --symlink-install` después de cargar `/opt/ros/jazzy/setup.bash`, y luego carga el overlay.

## Arquitectura de mapas

El mapa se selecciona mediante `--map` en los comandos offline y `map:=` en el launch. Los únicos archivos operativos por mapa son:

```text
config/maps/<nombre>/grafo.json
config/maps/<nombre>/obstaculos.json
```

Mapas disponibles en esta versión: `supermarket`, `tugbot_warehouse`, `colored_warehouse`, `pallet_warehouse`, `parking_lot`, `rubber_factory` y `electrical_substation`. La retícula de navegación tiene separación de 1.25 m. Los escenarios y resultados se guardan fuera de `config/maps/`, separados por mapa y semilla (salvo las rutas históricas conservadas para `supermarket`).

## Generar escenario y asignación A* + MILP

Ejemplo para `electrical_substation`, semilla 2030:

```bash
cd /home/dev/swarm_ws
python3 scripts/supermarket_milp/run_offline_seed.py \
  --seed 2030 --map electrical_substation
```

Si la asignación de esa semilla ya existe, el comando se detiene para no sobrescribirla. No uses `--overwrite` a menos que quieras recalcular explícitamente ese escenario y sus artefactos y hayas evaluado la compatibilidad de los planes guardados.

## Planificación completa offline

```bash
python3 scripts/supermarket_seed_pipeline/run_full_offline.py \
  --seed 2030 --map electrical_substation \
  --assignment-mode milp
```

Etapas del pipeline: `pickup1`, `delivery1`, `pickup2` y `delivery2`.

Para depurar sin esperar todas las optimizaciones, `run_full_offline.py` acepta `--until cbs` o `--until v3`; el valor predeterminado es `d8`. Las trayectorias solo se consideran preparadas para el siguiente paso cuando aparece:

```text
OFFLINE_FULL_STATIC_PASS_NOT_GAZEBO_CERTIFIED
```

En el modo `fixed`, primero debe existir el escenario de la semilla. La asignación `fixed` se construye automáticamente si falta:

```bash
python3 scripts/supermarket_seed_pipeline/run_full_offline.py \
  --seed 2030 --map electrical_substation \
  --assignment-mode fixed
```

Los planes y certificados se ligan a hashes de escenario, asignación, grafo y obstáculos. No combines artefactos de distintas semillas o mapas. Si se rechaza una etapa, consulta su reporte; no rebajes márgenes ni cambies el estado `PASS` manualmente.

## Ejecutar en Gazebo (solo con planificación completa)

```bash
ros2 launch swarm_worlds supermarket_full_prepared.launch.py \
  seed:=2030 map:=electrical_substation assignment_mode:=milp
```

El launch ejecuta verificaciones adicionales antes de iniciar la misión. La primera simulación de una nueva semilla debe supervisarse. Para otro mapa o semilla, usa los mismos valores en los tres comandos.

**Excepción histórica:** la semilla 100 de `supermarket` está protegida y tiene su launch específico `supermarket_full_experiment.launch.py`. No se la debe ejecutar mediante `supermarket_full_prepared.launch.py`.

## Resultados y recuperación

Los planes offline se almacenan normalmente en:

```text
scripts/supermarket_seed_pipeline/plans/<mapa>/seed_<N>/
  cbs_multigoal.json
  corridors_audit.json
  phase4/{pickup1,delivery1,pickup2,delivery2}/
  prepared/phase5_static_preflight.json
```

Para `supermarket`, por compatibilidad, la ruta omite `<mapa>/` y utiliza `plans/seed_<N>/`. El modo `fixed` añade un subdirectorio `fixed/` a los resultados de planificación. La ejecución Gazebo genera registros separados en `scripts/resultados/experiments/` (por mapa para los mundos no predeterminados).

El pipeline puede reutilizar certificados existentes si los hashes coinciden. Si hay una etapa parcial, conserva su carpeta para diagnóstico antes de volver a intentar esa etapa; **no elimines planes o reportes válidos automáticamente**.

## Pruebas automatizadas

```bash
python3 -m unittest discover -s tests -p 'test_*.py' -v
python3 -m unittest discover -s scripts/supermarket_milp/tests -p 'test_*.py' -v
```

Dependencias de Python utilizadas en el pipeline: `numpy`, `scipy` y `shapely`, entre otras dependencias de ROS 2 ya previstas en el contenedor. Se recomienda incluir `python3-shapely` en el Dockerfile para que las pruebas sean reproducibles tras reconstruir la imagen.


## Estructura unificada de mapas (noviembre 2026)

Los siete mapas (incluido `supermarket`) guardan escenarios en `config/scenarios/<mapa>/seed_N.json`, resultados MILP en `scripts/supermarket_milp/results/<mapa>/seed_N/` y planes preparados en `scripts/supermarket_seed_pipeline/plans/<mapa>/seed_N/`. Los planes anteriores de supermarket pueden contar con enlaces simbólicos de compatibilidad para rutas absolutas internas; no borrarlos hasta recalcular o archivar esos planes.
