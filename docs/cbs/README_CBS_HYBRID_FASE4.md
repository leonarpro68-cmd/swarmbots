# SwarmBots — Fase 4: refinamiento iterativo de corredores híbridos (offline)

**Rama:** `feature/cbs-hybrid-refinement` (encima de Fases 1–3).
**Estado:** experimental, **no ejecutable en ROS 2**, sin cambios en CBS ni `cmd_vel`.

## Implementación

* `hybrid_refinement/iterative_refinement.py`: reconstruye en cada subintervalo la envolvente convexa de **32 muestras** de la curva Bézier previa; no añade los puntos de control al conjunto usado para la separación.
* Para cada pareja de robots, calcula los puntos más próximos de sus envolventes y genera una bisectriz de máximo margen con la separación física conservadora del CBS (aprox. 1.523 m en la configuración actual). Generaliza a los **3 pares de 3 Summit**.
* Para cada obstáculo circular (ya inflado según el modelo de `DynamicTrashGraph`) calcula un semiplano tangente orientado al espacio libre. Mantiene las cuatro restricciones de pared. Se forma así un nuevo polígono convexo por robot y subintervalo.
* `bezier_optimizer.py` admite ahora `initial_trajectory=` como punto inicial SLSQP cuando sus controles permanecen dentro de la nueva partición. Si no, usa una semilla Bézier C² con extremos de las piezas optimizadas previamente (no los subintervalos rectos originales).
* Se minimiza de nuevo la **integral nominal del cuadrado de aceleración**, con continuidad C², anclas CBS por defecto y WAIT absolutamente fijos. Cada candidato se valida por **sus seis puntos de control** en cada corredor, no solo por las 32 muestras. Si falla la geometría, convergencia, factibilidad o no mejora el objetivo más que la tolerancia, se retiene el plan anterior.
* Un factor temporal **único** recalcula una duración escalada conservadora para todos los robots después de cada iteración aceptada. Esto no es un modelo de control dinámico completo de los Summit.

## Ejecución sin ROS/Gazebo

En el repositorio local, tras aplicar el parche incremental:

```bash
cd ~/swarmbots
export PYTHONPATH="$PWD/swarm_ws/src/swarm_behavior:$PYTHONPATH"
python3 -m pytest -q \
  swarm_ws/src/swarm_behavior/test/test_hybrid_adapter.py \
  swarm_ws/src/swarm_behavior/test/test_hybrid_corridors.py \
  swarm_ws/src/swarm_behavior/test/test_hybrid_bezier.py \
  swarm_ws/src/swarm_behavior/test/test_hybrid_iterative_refinement.py
```

Demostración con rutas de recogida producidas por CBS y mapa dinámico:

```bash
python3 -m swarm_behavior.swarm_planning.hybrid_refinement.demo_iterative \
  --mode pickup --iterations 5 --samples 32 \
  --out /tmp/swarm_hybrid_fase4.json \
  --png /tmp/swarm_hybrid_fase4.png
```

Para demostrar mejora cuando el optimizador inicial tuvo un presupuesto insuficiente:

```bash
python3 -m swarm_behavior.swarm_planning.hybrid_refinement.demo_iterative \
  --mode pickup --initial-maxiter 1 --maxiter 150 --iterations 5 \
  --out /tmp/swarm_hybrid_fase4_warmup.json
```

**Nota de interpretación:** con `--initial-maxiter 150`, el problema de recogida es relativamente simple y el refinamiento puede terminar con `converged` sin aceptar ninguna iteración: la Fase 3 ya había alcanzado un costo cercano al óptimo dentro de los corredores originales. No se debe inventar una mejora si no la hay. En la prueba con una inicialización truncada a una iteración SLSQP, el costo pasa aprox. de **2.32280** a **2.12321** después de una reconstrucción y reoptimización, sin relajar ninguna espera ni restricción geométrica. Los números son **offline** y específicos de ese caso.

El JSON contiene costos por robot, historial de iteraciones aceptadas, tiempos escalados, solución Bézier final, semiplanos y polígonos de los corredores finales, y `status=offline_validated_not_ros_executable`.

## Diferencias y límites frente al trabajo de Hönig et al.

La reconstrucción se inspira en la sección VII-D del artículo y en el ZIP `experimento_hibrido_refinamiento_paper.zip`. A diferencia del experimento original, se preservan los WAIT como posiciones constantes y no se acepta un candidato que incremente el objetivo. Esto protege el calendario del CBS a costa de restringir el espacio de soluciones. A diferencia del problema 3D de cuadricópteros, el modelo actual es 2D, con obstáculos circulares y límites traslacionales de velocidad y aceleración. No modela plenamente cinemática mecanum, orientación, cargas, incertidumbre de localización, errores de seguimiento ni latencias ROS 2.

**No utilizar este JSON para controlar los robots.** La Fase 5 debe definir interfaz temporizada y un controlador de referencia a partir del reloj compartido `/clock`, sin publicar desde dos nodos simultáneamente en `cmd_vel`; además se requieren pruebas en Gazebo, control de desviación y detención segura.
