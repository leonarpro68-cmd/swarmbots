# SwarmBots — Asignación offline A* + MILP (multimapa)

Este directorio contiene el cálculo de costos A* y la asignación óptima de tareas según un **modelo proxy de tiempo**. No genera por sí solo rutas CBS, trayectorias Bézier/V3 ni certifica el comportamiento físico de Gazebo.

## Instalación y entorno

El módulo **ya forma parte de `swarm_ws/scripts/supermarket_milp/`**. No es necesario instalarlo desde un ZIP independiente. Utiliza los mapas centrales en `config/maps/<mapa>/grafo.json` y `obstaculos.json`.

Ejecuta los siguientes comandos desde `/home/dev/swarm_ws` dentro de Docker (con Python y dependencias instaladas):

```bash
python3 scripts/supermarket_milp/run_offline_seed.py \
  --seed 2030 --map electrical_substation
```

Esta interfaz genera un escenario reproducible con 3 Summit, 6 objetos y 3 depósitos, calcula paradas de depósito a partir de vecinos conectados del grafo, evalúa costos A* y resuelve MILP con una basura de cada color por ronda. El mapa predeterminado es `supermarket`; puedes seleccionar cualquiera de los mapas disponibles mediante `--map`.

**Protección de resultados:** si ya existe la asignación de la semilla, `run_offline_seed.py` se detiene sin sobrescribirla. No uses `--overwrite` sin revisar los planes CBS/Bézier que dependan de los mismos archivos.

## Modelo que resuelve

- Se asignan dos basuras diferentes a cada uno de los tres robots, una por ronda, usando las seis basuras exactamente una vez.
- Las variables binarias representan pares ordenados de recogida para cada robot; el modelo considera **90 parejas**.
- El objetivo minimiza la suma de los cuatro tiempos máximos por etapa (proxy basado en distancias A* y velocidad de referencia), con un pequeño desempate de distancia.
- La restricción de una basura de cada color por ronda está activada por el orquestador.
- Se verifica el costo proxy de la asignación por enumeración exhaustiva de las alternativas permitidas.

El resultado **no es un óptimo global de navegación física**: CBS, separación geométrica, Bézier/V3 y D8 se verifican después, en el pipeline de preparación.

## Flujo completo de trabajo

Una vez concluido A* + MILP:

```bash
python3 scripts/supermarket_seed_pipeline/run_full_offline.py \
  --seed 2030 --map electrical_substation --assignment-mode milp
```

Cuando termine con `OFFLINE_FULL_STATIC_PASS_NOT_GAZEBO_CERTIFIED`, podrás ejecutar con ROS 2 (dentro de Docker):

```bash
ros2 launch swarm_worlds supermarket_full_prepared.launch.py \
  seed:=2030 map:=electrical_substation assignment_mode:=milp
```

El preflight offline no sustituye la validación física en Gazebo. La misión debe supervisarse.

## Herramientas de bajo nivel y pruebas

`build_costs.py` y `solve_assignment.py` siguen disponibles para análisis manual de un escenario existente. Su uso separado exige proporcionar rutas consistentes con el mismo mapa y semilla. Las salidas MILP se guardan en `scripts/supermarket_milp/results/<mapa>/seed_<N>/`, salvo `supermarket`, que conserva la ruta histórica `results/seed_<N>/`.

```bash
python3 -m unittest discover \
  -s scripts/supermarket_milp/tests -p 'test_*.py' -v
```

Los resultados de la antigua semilla 100 del supermercado pueden conservarse como referencia histórica, pero no deben confundirse con planes ejecutables de una nueva semilla o un nuevo mapa.
