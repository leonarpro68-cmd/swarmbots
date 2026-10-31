# SwarmBots D.9.8C.6 — Puente CBS → Bézier quíntico por fases (OFFLINE)

Esta entrega **NO** autoriza controlar Gazebo y **NO** declara certificación Hybrid V2 + D.9.2. Convierte las tres fases discretas D.9.8C.5 en controles Bézier quínticos C2, con robot estacionado representado por un segmento constante y velocidades cero en cada waypoint. El avance conservador se detiene en cada nodo; **no** es todavía el suavizado temporal óptimo D.9.2 y no garantiza evitación continua de obstáculos o movimientos transportando cargas.

## Instalación

En el host:

```bash
unzip -o ~/Descargas/swarm_D98C6_bezier_tres_fases_offline.zip -d ~/swarmbots
```

En Docker, desde `~/swarm_ws`:

```bash
python3 -m unittest discover -s scripts/tests -p test_d98c6_bezier.py -v
```

## Ejecución (sin Gazebo)

```bash
python3 scripts/d98c6_bezier_phase_preview.py \
 --cbs scripts/resultados/d9_live/seed_100_pickup1_009/d98c1/delivery1_wave_parking_cbs_preview.json \
 --snapshot scripts/resultados/d9_live/seed_100_pickup1_009/d98c1/delivery1_live_snapshot.json \
 --segment-duration 8
```

Genera `delivery1_bezier_three_phases_preview.json` y `delivery1_bezier_three_phases_preview.npz` junto al JSON CBS. Los controles se almacenan por clave `<phase>_<summit>_controls` y los tiempos `<phase>_<summit>_durations_s`. Los límites evaluados son velocidad ≤1.2 m/s, aceleración ≤1.0 m/s² y separación conservadora ≥1.15 m, *en la geometría interrobot prevista*, usando derivadas Bernstein y una extensión Lipschitz entre muestras.

**Limitaciones importantes:** las rutas parten de posiciones CBS discretizadas y estados posteriores a fases son predichos, no medidos. Falta verificar obstáculos a lo largo de todos los segmentos, introducir puntos de giro/entrega y carga, conectar el optimizador D.9.2 con robots estacionados, certificar conversiones D.9.4/D.9.5 por fase y recalcular con odometría fresca cuando se reabra Gazebo. Además, validar carga mediante evidencia persistente de unión y configurar restauración del escenario, antes de la ejecución de transporte.

En las pruebas locales con `d98c5_preview_test.json` y `delivery1_live_snapshot.json`, la vista previa de tres fases superó los tres límites con `--segment-duration 8`, pero no indica que tal resultado siga vigente tras restaurar Gazebo.
