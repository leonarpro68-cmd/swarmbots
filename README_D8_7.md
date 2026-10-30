# SwarmBots D.8.7 — coherencia del spawn con el escenario CBS

## Causa del fallo de la semilla 42

El generador `escenario_aleatorio_swarm.py` escogía tres posiciones iniciales
libres en la cuadrícula y comprobaba la planificación de dos rondas, pero
`swarm_cbs_grasp_base.launch.py` ignoraba esas posiciones. Introducía otros
robots mediante `_robot_specs_random(3, seed, (-8,8,-8,8))`, un muestreo
continuo que podía caer cerca de columnas o bordes de la cuadrícula libre.

Por eso el Summit 2 llegó a estar 0.856 m del nodo libre más próximo, mientras
`nearest_free_starts` exigía como máximo 0.750 m. Incrementar max_snap
relajaría un chequeo sin corregir el problema de fondo.

## Cambios D.8.7

1. `swarm_cbs_grasp_base.launch.py`: ahora usa `robots[].xy_m` y `robots[].grid`
   de `scenario.json`, después de verificar grafo libre, separación entre robots
   y separación respecto de objetos. Solo el yaw se toma de un RNG sembrado.
2. `run_swarm_seed_d8.py`: tras `READY` llama a un preflight adicional antes
   de la misión.
3. `d8_verify_initial_positions.py`: captura odometrías reales de los tres
   Summit y verifica que coincidan con el JSON dentro de 0.25 m, que sus frames
   tengan el nombre esperado y que estén detenidos. Escribe un archivo
   `spawn_verification.json` con evidencia para inspección.

**NO se alteran** CBS, SLSQP/Hybrid V2, la temporalización, velocidad,
aceleración, manejo RGB-D, DetachableJoint, D7 ni las oleadas de entrega.
No se cambia `max_snap=0.75` ni el certificado del plan.

## Instalación

En el host Ubuntu, dentro del directorio que contiene `swarm_ws`:

```bash
cd ~/swarmbots
cp swarm_ws/src/swarm_worlds/launch/swarm_cbs_grasp_base.launch.py \
  swarm_ws/src/swarm_worlds/launch/swarm_cbs_grasp_base.launch.py.bak_D8_6
cp swarm_ws/scripts/run_swarm_seed_d8.py \
  swarm_ws/scripts/run_swarm_seed_d8.py.bak_D8_6
unzip -o ~/Descargas/swarm_hybrid_v2_D8_7_spawn_scenario.zip -d ~/swarmbots
```

Dentro de Docker (sin otra simulación activa):

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select swarm_worlds --symlink-install
source install/setup.bash
python3 -m py_compile scripts/run_swarm_seed_d8.py \
  scripts/d8_verify_initial_positions.py \
  src/swarm_worlds/launch/swarm_cbs_grasp_base.launch.py
```

Probar de nuevo en una escena limpia:

```bash
python3 scripts/run_swarm_seed_d8.py \
  --seed 42 --auto-confirm --close-gazebo --execute
```

Se debe mostrar, después de READY:

`[D8] SPAWN OK: las tres odometrias coinciden con scenario.json (<=0.25 m)`

En el resultado también queda el JSON `spawn_verification.json`. Para la
semilla 42, Summit 2 debe aparecer inicialmente cerca de `(3.75, -21.25)`
(en el mundo Gazebo). El lanzador sigue aplicando preflight en vivo de CBS y
Hybrid V2 antes de ordenar movimiento.

**Validación efectuada:** sintaxis de los 3 scripts, pruebas unitarias del spawn
contra semillas 42, 123, 7, 2026, 100 y 789, validación de rechazo de poses
fuera de la cuadrícula y de odometrías incorrectas. No se ha ejecutado Gazebo
ni se certifican maniobras locales en esta máquina. La política de spawn cambia
las trayectorias que podrán planificarse en semillas antes exitosas, de modo
que conviene verificar 42 primero y luego una semilla de referencia.

## Reversión

Con Gazebo apagado, restaurar los dos `.bak_D8_6` previos. Recompilar
`swarm_worlds`. Si se revierte el launch, también debe revertirse el script D8;
el nuevo comprobador detectaría correctamente que las posiciones del spawn
aleatorio antiguo no coinciden con el JSON.
