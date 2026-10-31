# SwarmBots D.9.6C.2H.2B — corrección del GRAPH_WATCHDOG (solo sombra)

## Motivo

El reporte de `pickup1` terminó en `FAULT` únicamente por `GRAPH_WATCHDOG`,
tras una admisión inicial válida y sin detectar otros publicadores de velocidad.
El diseño anterior consultaba **11 tópicos del grafo ROS de forma bloqueante**
en el mismo hilo que recibía `/clock` y odometría, y marcaba el inicio de la
consulta como si esta ya hubiera terminado. Una consulta DDS lenta puede
provocar un falso vencimiento del watchdog del grafo o privar temporalmente
al supervisor de odometría/reloj.

Este diagnóstico es consistente con el informe recibido; sin los tiempos
individuales de cada consulta no podemos demostrar qué petición se retrasó.

## Cambios

- Se añade `GraphScanner`, **solo de lectura**, con nodo ROS 2 separado e
  hilo propio para consultar los 11 tópicos.
- El timestamp de una auditoría del grafo solo se actualiza cuando **todos**
  los tópicos han sido consultados con éxito.
- El hilo de suscripción a `/clock` y odometría no queda bloqueado por
  consultas del grafo.
- `GRAPH_WATCHDOG` se activa si han transcurrido más de **2,5 s de reloj
  monotónico** desde la última auditoría completa. `GRAPH_SCAN_SLOW`
  rechaza un barrido que exceda **2,0 s** o siga bloqueado más de 2,0 s.
- `GRAPH_QUERY_ERROR` rechaza errores al consultar DDS y
  `GRAPH_SCANNER_STOPPED` rechaza el fin inesperado del hilo.
- Se mantienen los watchdogs previos: reloj de 0,75 s y odometría de 0,4 s,
  así como detección acumulativa de publicadores competidores.
- Los reportes incluyen métricas del tiempo de barrido y detalles de la
  situación exacta del watchdog en caso de FAULT.

**Este umbral de grafo NO es adecuado como mecanismo único de exclusividad
para mover robots. Esta entrega conserva `execution_authorized=false`,
no crea publicadores, no tiene actuador de parada y nunca envía comandos.**

## Instalación

En el anfitrión (fuera de Docker):

```bash
cd ~/swarmbots
cp swarm_ws/scripts/d96c2h2b_guarded_shadow.py \
  swarm_ws/scripts/d96c2h2b_guarded_shadow.py.bak_graph_fix
unzip -o ~/Descargas/swarm_D9_6C_2H_2B_fix_graph_watchdog.zip -d ~/swarmbots
```

Dentro de Docker:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PWD/scripts:$PYTHONPATH"
python3 -m py_compile scripts/d96c2h2b_guarded_shadow.py
python3 scripts/tests/test_d96c2h2b_guarded_shadow.py -v
```

## Repetición de la prueba

Mantener **una sola** simulación Gazebo seed100, los Summit inmóviles,
puentes `/clock` y `/summit{0,1,2}/odom` y ningún seguidor D.8 activo.

El plan debe ser reciente. Crear una carpeta nueva (sustituir `_005` por otro
número si ya existe):

```bash
python3 scripts/d96c2b_live_pipeline.py \
  --scenario scripts/resultados/d8/seed_100_run_007/scenario.json \
  --stage pickup1 \
  --accept-namespaced-world-odom \
  --output scripts/resultados/d9_live/seed_100_pickup1_005
```

Inmediatamente después:

```bash
python3 scripts/d96c2h2b_guarded_shadow.py \
  --run scripts/resultados/d9_live/seed_100_pickup1_005 \
  --stage pickup1 \
  --config-report scripts/resultados/d9_live/d96c2h1_configuration_report.json \
  --warmup-seconds 4 \
  --observe-seconds 5
```

Archivos esperados en la carpeta `d96c2h2b_guard/` del plan:
`pickup1_guard_report.json` y `pickup1_hold_observations.csv`.

El resultado deseado es `PASS_CONTINUOUS_HOLD_SHADOW_ONLY`, estado final
`STOPPED` y cero comandos. En caso de fallo, revisar `faults`,
`fault_watchdog_details`, `ros_graph.last_scan_duration_wall_s`,
`ros_graph.max_scan_duration_wall_s`, `ros_graph.scan_error`.

## Alcance de pruebas

Se ejecutaron **39 pruebas unitarias locales** y validación sintáctica Python.
No tenemos en este entorno la ejecución activa de tu Gazebo/ROS 2; el
comportamiento de DDS bajo carga todavía debe verificarse en tu Docker.
La validación en reposo no autoriza la ejecución D9.2 con movimiento.
