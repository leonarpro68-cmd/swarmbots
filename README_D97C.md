# D.9.7C — Primer seguidor ROS 2 de tres Summit (solo Gazebo)

Requiere D9.7A, D9.6A, D9.4, D9.5, D9.6C.2B instalados. NO modifica D8. Es una integración experimental; el watchdog NO garantiza parada si él mismo o Gazebo fallan. Los robots deben estar alejados de obstáculos y otros controladores detenidos. Utiliza SOLO una instancia de Gazebo.

## Instalar y ensayar código

Desde el host: `unzip -o ~/Descargas/swarm_D97C_tres_summit_bezier.zip -d ~/swarmbots`

Dentro de Docker:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/scripts:$PWD/src/swarm_behavior:$PYTHONPATH"
python3 -m unittest discover -s scripts/tests -p test_d97c_common.py -v
```

## Antes de mover

1. Gazebo seed100 **solo base**, puentes odom y /clock.
2. Parar followers de D8; verificar que `ros2 topic info /summitN/cmd_vel -v` diga cero publishers para N=0,1,2.
3. Generar un plan fresco con `d96c2b_live_pipeline.py` usando una carpeta nueva (ver el comando abajo).
4. Comprobar espacio libre y supervisar la simulación. No usar con robots reales.

```bash
python3 scripts/d96c2b_live_pipeline.py \
 --scenario scripts/resultados/d8/seed_100_run_007/scenario.json \
 --stage pickup1 --accept-namespaced-world-odom \
 --output scripts/resultados/d9_live/seed_100_pickup1_006
python3 scripts/d97c_follower.py --run scripts/resultados/d9_live/seed_100_pickup1_006 --stage pickup1
```

Primera salida: `INERT_PREVIEW`.

## Terminal A — watchdog de 3 robots (produce velocidades; se apaga al finalizar)

```bash
python3 scripts/d97c_watchdog3.py --enable-sim-relay --max-seconds 35 \
 --output scripts/resultados/d9_live/d97c_relay_first.json
```

## Terminal B — ejecutar un fragmento de 0,60 segundos de tiempo de simulación

**Inmediatamente** tras el arranque del watchdog, dentro de los 120 s de simulación desde planificación:

```bash
python3 scripts/d97c_follower.py \
 --run scripts/resultados/d9_live/seed_100_pickup1_006 \
 --stage pickup1 --execute-in-gazebo
```

El primer ensayo se limita a 0,60 s de referencia, **no a 8,7 s**. Al salir el ejecutor ordena ceros. Una vez completado revisar informes, odometría final y watchdog antes de hacer un seguimiento completo. El relay termina por tiempo; no lo mate mientras haya movimiento.

La ejecución completa REQUIERE repetir desde un plan fresco y ejecutar con `--full-pickup1` añadido. No realizar hasta revisar el ensayo corto y sus informes.

Salida: `RUN/d97c/pickup1_d97c_report.json`, `.csv`; y `d97c_relay_first.json`. `FINISHED_TRAJECTORY_NOT_MISSION` NO significa recogida de basura: solo movimiento geométrico.
