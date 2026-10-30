# SwarmBots D.7.3 — Recogidas paralelas y entregas por oleadas

**Parche incremental sobre D.7.2 + D.7.1.** NO reemplaza CBS, Hybrid V2, path_follower, grasp_manager, visual_grasp, ni los scripts originales de manipulación.

## Cambios

- `run_hybrid_v2_mission_d7.py`: sustituye los bucles locales secuenciales por:
  - `parallel_grasps()` **original** en `agarres_paralelos_cbs.py`, lanzando las tres rutinas `prueba_alinear_agarrar_cbs.py` simultáneamente. Se realiza un preflight conjunto antes de mover cualquier robot; se verifica la separación de 3m entre objetivos (como en la rutina antigua), >=1.15m entre corredores ideales y los checks D6 (asignación, ubicación de basura, odometría y cercanía). No se continúa hasta tres ACK de alineación, unión y regreso. La evidencia geométrica adicional se revisa **después de que los tres regresaron**.
  - `release_waves()` adaptado de `entregas_paralelas_cbs_auto.py`: agrupa Summit por destinos, permite máximo uno por depósito en la misma oleada, y ejecuta simultáneamente los integrantes de la oleada. Los restantes esperan hasta que **todos** regresen y sus entregas sean verificadas. La lógica comprueba geométricamente segmentos de aproximación, robots estacionados y odometría fresca al entrar a cada oleada.
- `d7_parallel_delivery.py`: copia derivada **sin alterar** el script antiguo `entregas_paralelas_cbs_auto.py`, con callbacks de barrera y revalidación antes de cada oleada, sin confirmaciones por teclado.
- No altera la velocidad (1.20 m/s), aceleración (1.00 m/s²), temporalización, planificador ni el sistema de carpetas automáticas (`mision_001`, etc.).

## Instalación

En el anfitrión Ubuntu (el ZIP usa la estructura `swarm_ws/`):

```bash
cd ~/swarmbots
cp swarm_ws/scripts/run_hybrid_v2_mission_d7.py swarm_ws/scripts/run_hybrid_v2_mission_d7.py.bak_D7_2
unzip -o ~/Descargas/swarm_hybrid_v2_D7_3_oleadas_paralelas.zip -d ~/swarmbots
```

Si existe diferente carpeta compartida con Docker, extraer allí. NO sobrescribir el workspace dentro del contenedor si no persiste.

Dentro de Docker:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PYTHONPATH"
python3 -m py_compile scripts/run_hybrid_v2_mission_d7.py scripts/d7_parallel_delivery.py
python3 scripts/run_hybrid_v2_mission_d7.py --scenario /tmp/swarm_hybrid_123_scenario.json --auto-confirm
```

En un nuevo Gazebo **reiniciado**, escenario inicial verificado, tres Summit detenidos, los seis objetos en posiciones correctas y nodos habituales activos:

```bash
python3 scripts/run_hybrid_v2_mission_d7.py \
  --scenario /tmp/swarm_hybrid_123_scenario.json \
  --world-verified --auto-confirm --execute
```

Sigue solicitando **una sola confirmación inicial** (`SI`). D.7 genera una carpeta nueva automáticamente, sin pisar ejecuciones anteriores.

## Comportamiento esperado

| Asignación | Recogida | Entrega |
|---|---|---|
| 3 basuras | 3 agarres simultáneos, tras preflight | — |
| 3 depósitos distintos | — | 1 oleada: 3 Summit a la vez |
| 2 Summit a un depósito, 1 a otro | — | 2 oleadas: primero 2 simultáneos (distintos depósitos), luego 1 |
| 3 Summit al mismo depósito | — | 3 oleadas de 1 Summit |

Si los corredores locales no tienen separación suficiente, el programa **se detiene** antes de iniciar una maniobra incompatible: no fuerce los movimientos ni reduzca el umbral. El orden dentro de cada oleada se decide automáticamente a partir de las asignaciones y la geometría.

## Pruebas

```bash
python3 -m pytest -q tests/test_d7_parallel_policy.py
```

9 pruebas locales superadas: planificación de 3 depósitos distintos, 2+1, 3 en un depósito, rechazo de asignaciones/poses inválidas, cruces de corredores, ejecución de oleadas con procesos simulados, barreras y fallo ante falta de ACK. **No se ha realizado una prueba real con 3 robots simultáneos en Gazebo.**

## Precauciones

- El preflight usa aproximaciones geométricas de líneas rectas y distancias. **No equivale** a una certificación continua de la maniobra visual o del cuerpo/carga. Mantener supervisión visual, especialmente la primera vez.
- Los scripts antiguos de giro y RGB-D siguen controlando sus propios Summit. No lanzar otro controlador/ejecutor simultáneo.
- Ante fallo/timeout el coordinador detiene nuevos pasos y termina los procesos de la oleada; comprobar en Gazebo que los robots estén detenidos antes de intervenir.
- No relanzar sobre un ledger/escena parcialmente ejecutados. Inspeccionar uniones, basuras, robots y carpetas.
- Los ACK deben ir seguidos de evidencia geométrica en modo `--auto-confirm`, pero eso no demuestra por sí solo el estado físico exacto del `DetachableJoint` ni la seguridad completa de todas las maniobras.
