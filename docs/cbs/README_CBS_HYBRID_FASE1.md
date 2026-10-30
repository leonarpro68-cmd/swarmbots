# CBS + refinamiento híbrido — Fase 1 (adaptador offline)

**Rama prevista:** `feature/cbs-hybrid-refinement`, basada en `feature/swarmbots-auto-v1.1`.

Esta fase **no cambia el CBS actual**, ni los ejecutores de rondas, ni los
`path_follower`, ni los tópicos `cmd_vel`. Es un paso de preparación para
incorporar posteriormente los corredores híbridos, curvas Bézier y su
seguimiento temporizado, partiendo de las rutas producidas por el CBS real.

## Entrada y salida

- Entrada: `PickupPlan.paths`, `DeliveryPlan.paths` o `result['paths']` de
  `swarm_planning.cbs.solve`. Se requiere el **mismo grafo** (preferentemente
  `PlanningBridge.graph` durante misiones con basuras dinámicas).
- Valida: todos los vértices y aristas, segmentos libres en el grafo dinámico,
  separación entre centros durante cada movimiento sincronizado mediante
  `cbs.first_conflict` y persistencia de posiciones finales.
- Alarga exclusivamente al final los caminos más cortos con `WAIT` para
  obtener un horizonte común; conserva las esperas internas que devuelve CBS.
- Transforma cada vértice `(i,j)` a coordenadas métricas con
  `geometry.node_position` y produce intervalos `[t_k,t_{k+1}]` compartidos.
- Calcula **un tiempo NOMINAL mínimo por tick** a partir de la mayor
  transición espacial y `max_speed_m_s` (0.60 m/s como referencia del
  seguidor documentado). No equivale a factibilidad dinámica: la etapa Bézier
  deberá comprobar/acotar velocidad, aceleración y control.

### Demostración offline con el CBS real del repositorio

Desde la raíz del repositorio:

```bash
cd ~/swarmbots
export PYTHONPATH="$PWD/swarm_ws/src/swarm_behavior:$PYTHONPATH"
python3 -m swarm_behavior.swarm_planning.hybrid_refinement.demo_adapter \
    --out /tmp/swarm_cbs_hybrid_schedule.json
```

Se ejecuta la misma instancia de tres Summit usada en `cbs.py`: invoca CBS,
convierte sus trayectorias y guarda `/tmp/swarm_cbs_hybrid_schedule.json`.
No abre Gazebo ni publica comandos de movimiento.

### Pruebas

```bash
cd ~/swarmbots
export PYTHONPATH="$PWD/swarm_ws/src/swarm_behavior:$PYTHONPATH"
python3 -m pytest -q swarm_ws/src/swarm_behavior/test/test_hybrid_adapter.py
```

Si falta pytest en el contenedor, se puede instalar en el entorno de desarrollo
antes de repetir las pruebas; la biblioteca del adaptador requiere solamente
Python estándar y los módulos existentes de SwarmBots.

## Uso desde PlanningBridge

```python
from swarm_behavior.swarm_planning.bridge import PlanningBridge
from swarm_behavior.swarm_planning.hybrid_refinement import from_cbs_paths

bridge = PlanningBridge.from_json("/ruta/al/scenario.json")
plan = bridge.plan_pickup(starts)  # starts: vértices reales de planning_starts(...)
schedule = from_cbs_paths(plan.paths, graph=bridge.graph)
print(schedule.times_s)
print(schedule.steps[0].starts, schedule.steps[0].ends)
```

El adaptador debe invocarse cuando CBS terminó de planificar y el mapa dinámico
representa esa misma fase, **antes** de que la recogida cambie el mapa.

## Limitaciones y próximas fases

1. El resultado es un calendario nominal, **no una trayectoria ejecutable**.
   Cada movimiento (incluidos los diagonales) tiene el mismo tiempo inicial
   calculado; no garantiza límites de aceleración ni seguimiento.
2. Los ticks y el margen de CBS se mantienen sin alterar. Los intervalos
   `WAIT` permanecen inmóviles en la representación discreta.
3. Siguiente: corredores híbridos robot-robot / robot-obstáculo para el
   `DynamicTrashGraph` real, con rechazo si una partición espacial segura no
   existe. Las restricciones de los tres pares de robots se impondrán en
   cada intervalo de tiempo.
4. Después: Bézier, refinamiento iterativo, validación **continua** y
   sincronización mediante `/clock` con el controlador existente. No se
   conectará al movimiento hasta validar las restricciones físicas.
5. El fallback de la futura misión, si no hay solución refinada válida,
   debe ser el modo CBS discreto actual. Esta fase no cambia ese modo.
