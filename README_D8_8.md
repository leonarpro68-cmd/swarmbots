# SwarmBots D.8.8 — Replanificación CBS compatible con margen Hybrid V2

## Problema reproducido

Semilla 42, `delivery1`, odometría inicial de la ejecución `seed_42_run_008`:

```
No hay separacion espacial respecto al obstaculo (d=0.884, r=0.896)
```

CBS permite un segmento con separación 0.884 m respecto al centro de una basura todavía pendiente. Hybrid V2 necesita 0.896 m (radio de exclusión del mapa dinámico, 0.846 m, más margen operativo de 0.05 m). La diferencia de 0.012 m fuerza correctamente el REJECTED.

## Solución

Se conservan las rutas normales y los parámetros. Solo si una optimización falla **exactamente por falta de separación espacial respecto al obstáculo**, se activa un intento adicional:

1. El grafo CBS se filtra temporalmente eliminando vértices y aristas a menor distancia que los radios de exclusión **ya existentes** más el margen de Hybrid V2 (sin duplicar inflación).
2. CBS calcula una nueva trayectoria; no se cambia el estado de la misión ni la asignación de basuras.
3. Se reconstruyen los corredores y optimizan las curvas quínticas con los límites originales.
4. La nueva solución se acepta únicamente con certificado de Hybrid V2 `CERTIFIED`.
5. El grafo original se restaura al terminar la tentativa, incluso en caso de excepción. Si no hay alternativa certificada, el programa sigue rechazando la ejecución.

No se cambia `max_snap`, velocidad, radio físico, margen 0.05, política de múltiples seguidores, D.7.3, D.8.5, D.8.7, ni agarres/depositos.

## Archivos

- `swarm_ws/scripts/prepare_hybrid_v2_d6_stage.py` — intento de recuperación conservadora.
- `swarm_ws/scripts/d88_margin_graph.py` — filtro temporal de grafo por separación geométrica.
- `swarm_ws/scripts/tests/test_d88_offline.py` — regresión con la semilla 42 y odometría del fallo.

## Instalación

Fuera de Docker:

```bash
cd ~/swarmbots
cp swarm_ws/scripts/prepare_hybrid_v2_d6_stage.py swarm_ws/scripts/prepare_hybrid_v2_d6_stage.py.bak_D8_8
unzip -o ~/Descargas/swarm_hybrid_v2_D8_8_margen_cbs.zip -d ~/swarmbots
```

Dentro de Docker, con Gazebo detenido:

```bash
cd ~/swarm_ws
python3 -m py_compile scripts/prepare_hybrid_v2_d6_stage.py scripts/d88_margin_graph.py
```

No es necesario `colcon build`: son scripts que se ejecutan directamente, no se modifica un paquete ROS instalado.

Prueba offline (opcional, sin mover robots):

```bash
PYTHONPATH="$PWD/src/swarm_behavior:$PWD/scripts" python3 scripts/tests/test_d88_offline.py
```

Reprobar en mundo limpio:

```bash
python3 scripts/run_swarm_seed_d8.py --seed 42 --auto-confirm --close-gazebo --execute
```

Si se reproduce la misma situación, se espera:

```
[D8.8] CBS incompatible con reserva Hybrid V2: ...
[D8.8] ALTERNATIVA CERTIFIED; aristas excluidas: 10
CERTIFIED
```

El resto de la misión todavía debe comprobarse en Gazebo; el test offline no valida agarres ni dinámica real.

## Evidencia local

La regresión `swarm_ws/scripts/tests/test_d88_offline.py` sobre la geometría de la semilla 42, trayectorias CBS, y odometría reportada:

- Reprodujo exactamente `d=0.884, r=0.896`.
- Eliminó 10 aristas de un grafo temporal, sin eliminar nodos.
- Encontró una alternativa CBS; la optimización completa con `iterations=2, radius=0.55, margin=0.05` devolvió `CERTIFIED`.
- Certificado: `min_obstacle_extra_clearance_m ≈ 0.0600 m` (frente a mínimo de 0.0500 m).
- Verificó restauración del grafo y escritura de `delivery1_live_report.json` + `delivery1_live_controls.npz`.
