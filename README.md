# D.8.10 — Cinemática de trayectorias Hybrid V2

Este parche sustituye **únicamente** `swarm_ws/scripts/plot_mission_d8.py` (D.8.9).

## Instalación

Desde el host:

```bash
cd ~/swarmbots
cp swarm_ws/scripts/plot_mission_d8.py swarm_ws/scripts/plot_mission_d8.py.bak_D8_9
unzip -o ~/Descargas/swarm_hybrid_v2_D8_10_cinematica.zip -d ~/swarmbots
```

Dentro de Docker:

```bash
cd ~/swarm_ws
export PYTHONPATH="$PWD/src/swarm_behavior:$PWD/scripts:$PYTHONPATH"
python3 -m py_compile scripts/plot_mission_d8.py
python3 scripts/plot_mission_d8.py --run scripts/resultados/d8/seed_100_run_001 --kinematics
```

Se generan `mission_hybrid_v2.png` (las rutas) y `mission_hybrid_v2_cinematica.png` (cuatro filas: pickup1, delivery1, pickup2 y delivery2; columnas de rapidez y magnitud de aceleración). Se muestra cada Summit y los límites de referencia 1.20 m/s y 1.00 m/s². Para generar solamente un PNG de rutas, omitir `--kinematics`.

Parámetros optativos: `--kinematics-output RUTA.png`, `--samples-per-piece 50`, `--max-speed 1.20` y `--max-accel 1.00`.

Se usan **derivadas analíticas** de cada Bézier quíntica y el `dt_s` del NPZ de protocolo: `v = 5(P_{k+1}-P_k)/dt`, `a = 20(P_{k+2}-2P_{k+1}+P_k)/dt²` como puntos de control de derivadas. El tiempo reinicia en cada etapa. Las muestras incluyen los extremos de cada pieza, por lo que las discontinuidades entre piezas no quedan ocultas. Las curvas representan **trayectorias planificadas**, no velocidades/odometrías realmente medidas en Gazebo; no incluyen aproximaciones RGB-D ni giros/retornos locales.

Este cambio no modifica la planificación, el seguidor, el controlador ni el lanzamiento D.8. La comparación con límites es visual; la certificación sigue haciéndose con el planificador original.
