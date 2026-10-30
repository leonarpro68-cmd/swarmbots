# Hybrid V2 — D.1, adaptador CBS real (offline)

## Alcance

- `cbs_adapter.py` recibe directamente un `PickupPlan` o `DeliveryPlan` de `PlanningBridge`, y el **DynamicTrashGraph de la etapa correspondiente**.
- Valida inicios, aristas, vértices y conflictos CBS; convierte `(i,j)` a metros y rellena con posiciones finales los tramos posteriores a la llegada.
- No inmoviliza los WAIT intermedios, respetando el experimento Python original.
- Usa radios del **grafo dinámico real**: columnas de `geometry.py`, basuras pendientes con `TRASH_EXCLUSION_M`, depósitos con `DEPOSIT_RADIUS_M`. No supone un radio fijo de 0.4 para toda basura; evita confundir el experimento de referencia con el escenario vivo.
- `plan_v2` llama a `pipeline_c2.run`, sin modificar SLSQP; no devuelve controles cuando el certificado se rechaza.
- `demo_d1.py` ejecuta una prueba con el CBS real. **Sus inicios de prueba son vértices de ejemplo**, no posiciones originales de Gazebo. En D.2 se usará la misma secuencia de inicios y cuatro rondas que `plot_mission_routes.py`.

## Instalación

Descomprimir en la raíz del repositorio `~/swarmbots` (fuera de Docker) y ejecutar dentro del contenedor:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PYTHONPATH"
python3 -m pytest -q src/swarm_behavior/test/test_hybrid_v2*.py
python3 -m swarm_behavior.swarm_planning.hybrid_v2.demo_d1 --seed 123 --output scripts/resultados/hybrid_v2_d1_seed123.json
```

Para ejecutar además la optimización del escenario de prueba (más costosa; puede rechazarse justificadamente):

```bash
python3 -m swarm_behavior.swarm_planning.hybrid_v2.demo_d1 --seed 123 --solve --iterations 2 --output scripts/resultados/hybrid_v2_d1_solve_seed123.json
```

**Cuidado:** El grafo dinámico cambia cuando `confirm_collected(name)` actualiza una basura. Para cada planificación, llamar `adapt_plan(plan, bridge.graph)` **antes** de confirmar nuevas recogidas. En una entrega, usar el grafo después de confirmar las basuras ya retiradas.

## Estado y límites

Esta etapa valida la interfaz CBS→Hybrid V2 con un plan real de recogida, pero **no** demuestra todavía que cada nueva misión sea factible con los corredores originales ni que SLSQP certifique todas las etapas. La integración offline de cuatro paneles será D.2. La conexión a controladores ROS corresponde a D.3–D.4. Ningún `launch` o publicador de velocidad ha sido modificado.
