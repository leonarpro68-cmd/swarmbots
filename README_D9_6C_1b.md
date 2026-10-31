# D.9.6C.1b — Validación de alineación inicial para modo sombra

Este parche **reemplaza solamente** `swarm_ws/scripts/d96c1_shadow.py` y añade pruebas. Conserva el comportamiento seguro por defecto: odometría con `summitN/odom` no equivale por definición a `map`, por lo que se rechaza. **No crea publishers ROS, no crea transformaciones TF y no controla ningún robot.**

## Instalación
Desde Ubuntu, fuera del contenedor:
```
cd ~/swarmbots
cp swarm_ws/scripts/d96c1_shadow.py swarm_ws/scripts/d96c1_shadow.py.bak_D96C1b
unzip -o ~/Descargas/swarm_D9_6C_1b_alineacion_sombra.zip -d ~/swarmbots
```
Dentro del contenedor:
```
cd ~/swarm_ws
python3 -m py_compile scripts/d96c1_shadow.py
python3 scripts/tests/test_d96c1b_alignment.py
```

## Prueba
Con Gazebo y los tres puentes odométricos y `/clock` funcionando, y el escenario `seed_100_run_007` cargado:
```
export PYTHONPATH="$PWD/src/swarm_behavior:$PWD/scripts:$PYTHONPATH"
python3 scripts/d96c1_shadow.py --run scripts/resultados/d8/seed_100_run_007 --stage pickup1 --gazebo-aligned-shadow
```

La bandera solo habilita una **hipótesis experimental para cálculos sin ejecución**: valida que las coordenadas iniciales de las tres odometrías difieran como máximo 0.10 m de `scenario.json`, que las referencias Bézier inicien en esos puntos, que la orientación recibida sea finita y los marcos sean exactamente `summitN/odom` y `summitN/base_footprint`. No establece que las orientaciones de `summitN/odom` y `map` coincidan, ni que la relación de marcos permanezca fija al moverse. **No usar esta hipótesis para movimiento real**. La ejecución futura requerirá una transformación verificada (TF2 u otra medición calibrada de la pose global).

El script sigue avanzando la referencia mientras los robots permanecen quietos, por lo que es esperable `BLOCKED_TRACKING_ERROR` después del comienzo. El estado final `COMPLETE_SHADOW_ONLY` describe la terminación de observación, no aprobación del control.

En el reporte verificar `gazebo_alignment_opt_in`, `initial_alignment_checked`, `initial_alignment_ok`, `initial_alignment_errors_m`, `shadow_status_counts`, `publisher_count=0` y `commands_sent=0`.
