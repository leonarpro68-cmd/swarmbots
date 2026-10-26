# SwarmBots – Fase 3: CBS → corredores híbridos → trayectorias Bézier C²

**Rama prevista:** `feature/cbs-hybrid-refinement`.
**Alcance:** simulación algorítmica **offline** (no ROS 2, no Gazebo, no publicación `cmd_vel`).

Esta fase complementa las Fases 1 y 2, sin modificar ni el algoritmo CBS ni los
controladores actuales de SwarmBots.

## Archivos que incorpora

- `swarm_planning/hybrid_refinement/bezier_optimizer.py`: problema cuadrático,
  restricciones, evaluador Bézier, validación y escalamiento temporal.
- `swarm_planning/hybrid_refinement/demo_bezier.py`: prueba integrada usando rutas
  **producidas por el CBS del repositorio**; modos `pickup` y `static`.
- `test/test_hybrid_bezier.py`: pruebas sobre recogida real y caso estático.
- `package.xml`: nueva dependencia de ejecución `python3-scipy` (ya existía NumPy).
- Esta documentación.

La importación de Fases 1/2 continúa independiente de SciPy: solo
`bezier_optimizer` y `demo_bezier` requieren NumPy y SciPy.

## Modelo matemático

Si hay `m` intervalos seguros de longitud común `dt`, para cada robot `i`
se construye una curva Bézier grado 5 por intervalo:

```text
f[i,k](u) = Σ_j B[j,5](u) P[i,k,j],   0 <= u <= 1, j=0...5
J[i] = Σ_k ∫_[k dt,(k+1)dt] ||d² f[i,k](t) / dt²||² dt
```

Se resuelve por robot un problema cuadrático con:

1. **Corredores convexos:** los seis puntos de control de cada pieza pertenecen
   al polígono del corredor en ese subintervalo. Al estar el polígono contenido
   en todos los semiplanos robot–robot, robot–obstáculo y paredes, la envolvente
   convexa de Bézier asegura esa contención para **todo el intervalo**.
2. **Continuidad C²** en cada junta: posiciones, velocidades y aceleraciones
   continuas. Los subintervalos tienen el mismo `dt` por construcción.
3. **Extremos:** inicio y meta CBS exactos, con velocidad y aceleración cero.
4. **Anclas CBS:** por defecto conserva también cada vértice original al final de
   cada tick, incluyendo los waypoints de misión. La opción experimental
   `--free-tick-waypoints` relaja estas anclas intermedias pero **no** los WAIT.
5. **WAIT:** si CBS marca el tick como espera, **los seis puntos de control** de
   cada pieza del tick se fijan al waypoint. La curva no se mueve ni recorre
   ninguna distancia durante esa espera.

Una solución inicial universalmente factible usa, para cada pieza,
`[a,a,a,b,b,b]` (smoothstep quintic). Se prueba contra TODAS las restricciones
antes de llamar a SLSQP. Las restricciones de igualdad redundantes (causadas
por WAIT consecutivos, etc.) se reducen mediante QR con pivoteo, evitando la
condición de más igualdades que variables que presenta SLSQP.

El programa valida por separado el resultado final. Si SLSQP no encuentra un
punto factible o intenta aumentar el costo, se recupera la semilla factible.
Una trayectoria que superó validación pero cuya optimización no convergió puede
conservarse como **candidato offline**, con `solver_success=false`. No debe
interpretarse como óptima.

## Escalamiento temporal común

El plan inicial hereda el tiempo nominal de la Fase 1. Aplicamos un factor
**común a los tres robots**, de forma que no se altera su sincronización:

```text
L = max(1, cota_velocidad / velocidad_admisible,
         sqrt(cota_aceleracion / aceleracion_admisible))
L <- L * safety_factor (por defecto 1.001)
T_escalado = L * T_nominal
```

Las cotas de rapidez y aceleración se obtienen de la **envolvente convexa de las
curvas Bézier derivadas**, no de un muestreo que pudiera omitir picos. Son cotas
conservadoras (podrían sobreestimar los máximos reales).

Los valores `.60 m/s` y `.50 m/s²` son **parámetros ilustrativos de traslación**,
no restricciones verificadas del modelo físico Summit. Siguen pendientes la
orientación, límite de giro, posibilidad de acelerar con cargas, odometría,
latencias, márgenes frente a seguimiento imperfecto, y actualización dinámica
de obstáculos.

## Pruebas

Desde la raíz del proyecto (instala `python3-scipy` en el mismo entorno de
Python que ejecutará las pruebas si todavía no existe):

```bash
cd ~/swarmbots
export PYTHONPATH="$PWD/swarm_ws/src/swarm_behavior:$PYTHONPATH"
python3 -c 'import numpy, scipy; print("NumPy y SciPy disponibles")'
python3 -m pytest -q \
  swarm_ws/src/swarm_behavior/test/test_hybrid_adapter.py \
  swarm_ws/src/swarm_behavior/test/test_hybrid_corridors.py \
  swarm_ws/src/swarm_behavior/test/test_hybrid_bezier.py
```

Resultado en el entorno usado para preparar esta entrega: **22 passed**.

### Recogida con grafo dinámico

```bash
python3 -m swarm_behavior.swarm_planning.hybrid_refinement.demo_bezier \
  --mode pickup \
  --out /tmp/swarm_hybrid_bezier_pickup.json \
  --png /tmp/swarm_hybrid_bezier_pickup.png
```

Referencia obtenida en pruebas offline: 3 robots, 8 piezas por robot,
24 corredores, costo de aceleración (por robot) de
`8.3786→0.5451`, `5.2366→0.5471`, `3.1420→1.0310`;
los tres solucionadores marcaron éxito. Tiempo nominal `23.57 s`;
escalado ilustrativo `.60 m/s`, `.50 m/s²`: **56.39 s**. Ese
escalado es conservador y **no** una medición del Summit en Gazebo.

### CBS estático con cruces temporales

```bash
python3 -m swarm_behavior.swarm_planning.hybrid_refinement.demo_bezier \
  --mode static --maxiter 40 \
  --out /tmp/swarm_hybrid_bezier_static.json
```

Referencia: 3 robots, 32 piezas por robot (8 ticks × 4 subintervalos).
Los controles permanecieron factibles y el costo disminuyó, pero SLSQP
alcanzó el límite de iteraciones con `solver_success=false` para los tres
robots. **No afirmar optimalidad para ese caso.** Este solver denso aún no
es adecuado para planificaciones extensas en línea; se estudiará un solver
QP sparse/segmentación local antes de ejecutar misiones largas.

## Estado de integración

El JSON contiene los puntos de control en `map`, resultados de SLSQP, tiempos
nominales y escalados, y estados explícitos
`status='offline_bezier_not_ros_executable'`.

**No publicarlo en `nav_msgs/Path` como si contuviera una trayectoria temporal.**
`nav_msgs/Path` no codifica la temporización continua por waypoint en el
seguidor existente. Sigue pendiente desarrollar un controlador sincronizado
con `/clock`, `/odom` y un único publicador `cmd_vel`, además de introducir
verificación de factibilidad de las trayectorias y un fallback CBS discreto.

La siguiente fase del postprocesamiento será refinamiento iterativo de los
corredores a partir de las propias curvas, revalidando siempre seguridad,
factibilidad y costo, seguida de la implementación ROS 2.
