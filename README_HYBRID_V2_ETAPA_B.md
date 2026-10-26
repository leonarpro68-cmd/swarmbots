# Hybrid V2 — Etapa B (regresión reproducible)

No modifica nodos ROS 2, launch, ni controladores. Incluye copia de tres artefactos de referencia aportados por el usuario (`reporte.json`, `controles_slsqp.npz` y `controles_slsqp_refinados.npz`).

## Cambios

- Corrección en `slsqp.py`: ya no depende de `NODES` del experimento original; usa `node_paths` pasado como argumento.
- Parámetro `maxiter` configurable en `solve_all` (por defecto 450, como el original).
- `regression_b.py`: validación de costos, longitudes aproximadas, continuidad C0/C1/C2, holguras muestreadas y comparación con resultados registrados.
- Pruebas ejecutando realmente `solve_all` sobre una ruta externa y comparando datos de referencia.

## Pruebas rápidas dentro de Docker

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PYTHONPATH"
export HYBRID_V2_REFERENCE_DIR="$PWD/src/swarm_behavior/test/data/hybrid_v2_reference"
python3 -m pytest -q src/swarm_behavior/test/test_hybrid_v2.py src/swarm_behavior/test/test_hybrid_v2_stage_b.py
```

Resultado comprobado en el entorno de desarrollo: `5 passed`.

## Informe contra artefactos originales (sin resolver SLSQP)

```bash
python3 -m swarm_behavior.swarm_planning.hybrid_v2.regression_b \
 --reference-dir "$HYBRID_V2_REFERENCE_DIR" \
 --output ~/swarm_ws/scripts/resultados/hybrid_v2_entrega_r2_regresion.json
```

Los costos de referencia son **6.008490019568636** (inicial) y **5.551884565077389** (refinado). La comprobación de holguras es muestreada, no una certificación física de colisiones.

## Reoptimización (costosa, optativa)

```bash
python3 -m swarm_behavior.swarm_planning.hybrid_v2.regression_b \
 --reference-dir "$HYBRID_V2_REFERENCE_DIR" \
 --output ~/swarm_ws/scripts/resultados/hybrid_v2_entrega_r2_resuelto.json \
 --solve --iterations 5 --maxiter 450
```

**Pendiente**: esta reproducción completa del optimizador, incluidos los 5 refinamientos, no fue ejecutada hasta su finalización en el entorno de preparación. El reporte de referencia confirma consistencia de lectura/costo y las pruebas pequeñas confirman que el optimizador funciona con rutas externas; no prueba todavía que la nueva implementación reproduzca la optimización completa dentro de una tolerancia.
