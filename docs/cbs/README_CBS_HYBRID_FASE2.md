# SwarmBots – CBS + corredores híbridos (Fase 2)

**Rama local prevista:** `feature/cbs-hybrid-refinement` (derivada de `feature/swarmbots-auto-v1.1`).

Esta entrega es **offline**. No reemplaza el algoritmo CBS ni los nodos de ROS 2,
no publica `cmd_vel`, no altera el ciclo de recogida/entrega y no lanza Gazebo.
**No genera todavía curvas Bézier** ni comandos ejecutables.

## Dependencias y archivos

Se apoya en el adaptador instalado en la Fase 1 (`adapter.py`) y en la
geometría del paquete actual `swarm_planning`:

- `geometry.py`: límites (-14,14) x (-24,24), columnas circulares, radio de
  seguridad del Summit + margen (`CLEARANCE_WALL_M ≈ 0.76145 m`).
- `cbs.py`: margen entre centros `MIN_SEPARATION ≈ 1.52290 m` y comprobación
  de conflicto en cada tick mediante interpolación **sincronizada**.
- `dynamic_map.py`: basuras todavía pendientes (círculos con exclusión
  `TRASH_EXCLUSION_M`), depósitos (radio de exclusión `DEPOSIT_RADIUS_M`) y
  columnas. El conjunto de obstáculos se obtiene del **mismo grafo** que
  usó CBS. Los depósitos tienen el radio configurado en el mapa dinámico;
  no se atribuye a esta simplificación una garantía de colisión frente al SDF.

No depende de NumPy, SciPy ni Shapely para construir corredores: se emplea
geometría 2D con Python estándar. `pytest` es necesario solo para las pruebas,
`matplotlib` únicamente para generar la imagen diagnóstica opcional.

Archivos nuevos:

```text
swarm_ws/src/swarm_behavior/swarm_behavior/swarm_planning/hybrid_refinement/
    corridors.py
    demo_corridors.py
swarm_ws/src/swarm_behavior/test/test_hybrid_corridors.py
docs/cbs/README_CBS_HYBRID_FASE2.md
```

También se amplía `hybrid_refinement/__init__.py` para exportar las funciones.

## Qué hace exactamente

1. Toma un `DiscreteSchedule` obtenido de rutas CBS por `from_cbs_paths`.
2. Obtiene la geometría del **mismo** `cbs.Graph` o `DynamicTrashGraph`. Si
   el tipo de grafo es desconocido, falla de forma conservadora.
3. Por cada tick y par de robots `(0,1)`, `(0,2)`, `(1,2)` calcula la
   distancia mínima entre los **segmentos completos**. Construye dos
   semiplanos perpendiculares al vector de puntos más próximos, separados
   por `MIN_SEPARATION`.
4. Para cada columna, basura pendiente y depósito, construye semiplanos
   tangentes a los **círculos de exclusión ya inflados**.
5. Interseca semiplanos y límites del mundo para obtener un polígono convexo
   por robot e intervalo. Comprueba área positiva y contención íntegra del
   segmento original en cada polígono.
6. Si dos segmentos son espacialmente demasiado próximos, subdivide **todos**
   los ticks en el mismo número de intervalos (1, 2, 4, 8, ... hasta 64 por
   defecto), sin modificar posiciones o duración global. Esto es importante
   porque CBS valida la separación **en tiempos iguales**, no entre todos
   los puntos de dos aristas visitados en instantes diferentes.
7. Si ninguna subdivisión es válida, retorna `CorridorError`. **Nunca** reduce
   el radio, el margen ni fuerza un resultado no factible.
8. Marca los intervalos `WAIT` para que la futura optimización mantenga la
   posición constante durante esos intervalos. Los corredores por sí solos
   NO obligan a quedarse inmóvil: esa será una restricción explícita de Bézier.

Un corredor seguro en un subintervalo solo significa que **cualquier par de
puntos** en los corredores correspondientes mantiene la separación espacial
mínima. Para garantizar lo mismo en una trayectoria Bézier, la próxima fase
exigirá que **todos sus puntos de control** estén dentro del polígono convexo,
y sincronizará la ejecución de los robots.

### Modelo de seguridad

Los obstáculos considerados son los documentados en `geometry.py` y
`dynamic_map.py`. No se analiza automáticamente la malla 3D ni los
colliders del mundo Gazebo, ni obstáculos dinámicos no registrados.
La factibilidad de la trayectoria con velocidad, aceleración, orientación,
seguimiento y posibles errores de odometría **no está demostrada** en esta fase.
El tiempo calculado por Fase 1 sigue siendo **nominal**.

## Ejecución offline

Desde la raíz del repositorio:

```bash
cd ~/swarmbots
export PYTHONPATH="$PWD/swarm_ws/src/swarm_behavior:$PYTHONPATH"
```

### Prueba estática con el CBS real

```bash
python3 -m swarm_behavior.swarm_planning.hybrid_refinement.demo_corridors \
  --mode static \
  --out /tmp/swarm_corridors_static.json \
  --png /tmp/swarm_corridors_static.png \
  --plot-tick 1
```

**Resultados de referencia en el ZIP provisto:** 3 robots, 8 ticks CBS,
4 subintervalos/tick, 96 corredores geométricos, 10 columnas circulares,
separación entre centros de 1.5229 m. Este ejemplo obliga a subdividir
pese a que CBS no detecta conflictos sincronizados.

### Prueba con basuras pendientes y depósitos (grafo dinámico)

```bash
python3 -m swarm_behavior.swarm_planning.hybrid_refinement.demo_corridors \
  --mode pickup \
  --out /tmp/swarm_corridors_pickup.json
```

Se llama a `PlanningBridge.plan_pickup()` para obtener rutas CBS de recogida;
no son rutas creadas a mano. La prueba utiliza los inicios `(8,18)`,
`(14,18)` y `(11,14)` y los objetos que define `DynamicTrashGraph` por defecto.
Resultados de referencia: 3 robots, 8 ticks CBS, 1 subintervalo/tick,
24 corredores, 19 círculos de exclusión (10 columnas, 6 basuras y
3 depósitos). Este es un **escenario de integración offline**, no la
confirmación de los seis agarres de una misión real en Gazebo.

Para un escenario de misión exportado en JSON, utilizar:

```bash
python3 -m swarm_behavior.swarm_planning.hybrid_refinement.demo_corridors \
  --mode pickup --scenario /ruta/scenario.json \
  --starts '8,18;14,18;11,14' \
  --out /tmp/swarm_corridors_scenario.json
```

Los vértices de `--starts` deben ser válidos para el escenario y esa fase.

### Pruebas unitarias

```bash
python3 -m pytest -q \
  swarm_ws/src/swarm_behavior/test/test_hybrid_adapter.py \
  swarm_ws/src/swarm_behavior/test/test_hybrid_corridors.py
```

El resultado esperado en el entorno con ambas fases es `14 passed`.

## Salida JSON y conexión futura

El tipo `swarmbots_hybrid_safe_corridors_v1` incluye:

- `status: geometry_only_not_executable`;
- robots, ticks originales, subintervalos/tick, tiempo nominal;
- lista de obstáculos (centros y radios inflados);
- `corridors[substep][robot]` con segmento original, tiempos, indicador
  `wait`, polígono y cada desigualdad `normal · p <= offset_m`;
- `schedule` original serializado (en la herramienta de demostración).

La Fase 3 utilizará estas **desigualdades lineales** como restricciones de
los puntos de control Bézier. No se debe enviar el JSON a `path_follower`
ni usarlo como trayectoria ROS 2 todavía.

## Integración prevista

Tras validar optimización Bézier (grado 5, continuidad C²) y comprobar
límites físicos del Summit, se añadirá una interfaz de seguimiento temporal
al controlador actual. Se mantendrán las barreras/confirmaciones de
agarre y entrega; la integración solo moverá los segmentos de navegación.
Si falta un corredor o una optimización válida, la misión debe retroceder
al **CBS discreto existente**, no ejecutar una trayectoria insegura.
