# SwarmBots D9.7C.1 — Cierre coordinado y pickup1 completo

Actualiza **solamente** `scripts/d97c_follower.py` y `scripts/d97c_watchdog3.py`; incluye prueba `scripts/tests/test_d97c1_protocol.py`. No modifica D8, otros launch ni las trayectorias. Exclusivamente Gazebo Harmonic.

## Cambios
- Mensaje de cierre `std_msgs/String`, tópico `/d97c/session_end`, valor exacto `D97C_STOP_ALL`.
- El follower, cuando concluye sin fallos, publica comandos cero y comprueba velocidades de los **tres** Summit debajo de 0,03 m/s en ocho observaciones; solo entonces manda la señal de cierre.
- El relay mantiene el corte de 0,30 s mientras está RUNNING. Después del cierre, publica solo ceros y requiere ocho observaciones de velocidad menor a 0,03 m/s antes de terminar `NORMAL_END_STOP_CONFIRMED`.
- Si no se confirma la detención en 2 s, `END_STOP_NOT_CONFIRMED`. Si hay silencio sin cierre, `INPUT_TIMEOUT` se mantiene.
- En seguimiento completo, verifica que el error respecto al punto final de cada referencia sea <= 0,12 m antes de declarar la trayectoria terminada.
- **No** certifica parada ante la caída del relay ni la ausencia de colisiones con obstáculos. Solo simulación, no robots físicos.

## Instalación
Host:
```
cd ~/swarmbots
cp swarm_ws/scripts/d97c_follower.py swarm_ws/scripts/d97c_follower.py.bak_pre_D97C1
cp swarm_ws/scripts/d97c_watchdog3.py swarm_ws/scripts/d97c_watchdog3.py.bak_pre_D97C1
unzip -o ~/Descargas/swarm_D97C1_cierre_coordinado_pickup1.zip -d ~/swarmbots
```
Dentro de Docker:
```
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/scripts:$PWD/src/swarm_behavior:$PYTHONPATH"
python3 -m unittest discover -s scripts/tests -p test_d97c1_protocol.py -v
python3 -m unittest discover -s scripts/tests -p test_d97c_common.py -v
```

## Procedimiento
1. Ejecuta **una sola instancia** de Gazebo con semilla 100, tres Summit y los puentes `/clock` + odometría; apaga seguidores D8 y cualquier relay previo. Revisa `/summitN/cmd_vel`: 0 publicadores.
2. Como Summit 0 avanzó durante el ensayo, genera un **nuevo** plan desde odometría actual. Por ejemplo, `seed_100_pickup1_007`, cambiando sufijo si ya existe:
```
python3 scripts/d96c2b_live_pipeline.py \
 --scenario scripts/resultados/d8/seed_100_run_007/scenario.json \
 --stage pickup1 --accept-namespaced-world-odom \
 --output scripts/resultados/d9_live/seed_100_pickup1_007
```
Verifica `PASS_READ_ONLY_LIVE_REPLAN` y `INERT_PREVIEW`.
3. Primer ensayo recomendado (0,60 s) para comprobar protocolo nuevo:
Terminal A:
```
python3 scripts/d97c_watchdog3.py --enable-sim-relay --max-seconds 35 \
 --output scripts/resultados/d9_live/d97c1_relay_short.json
```
Terminal B (tras empezar relay, sin esperar a que termine):
```
python3 scripts/d97c_follower.py --run scripts/resultados/d9_live/seed_100_pickup1_007 \
 --stage pickup1 --execute-in-gazebo
```
Espera salida A. Deben aparecer `SHORT_CHECKOUT_COMPLETED`, `final_stop_confirmed:true`, `end_signal_sent:true`, y en el relay `NORMAL_END_STOP_CONFIRMED`, `normal_end:true`.
4. Para `pickup1` COMPLETO, como ya hubo movimiento, **genera otro plan nuevo** (p. ej. `seed_100_pickup1_008`) repitiendo D9.6C.2B desde posiciones actuales. Si Gazebo cambió o pasó más de 120 s de simulación, replanifica de nuevo. Con escenario despejado, ejecuta relay con informe `d97c1_relay_full.json` en terminal A, y en terminal B:
```
python3 scripts/d97c_follower.py \
 --run scripts/resultados/d9_live/seed_100_pickup1_008 \
 --stage pickup1 --execute-in-gazebo --full-pickup1
```
Solicita movimiento real en Gazebo con los TRES Summit; no mueve basuras ni ejecuta tareas de grasp. No inicies ejecutor D8 al mismo tiempo.
5. Envía ambos JSON (follower en carpeta del run y relay). Para el reporte CSV y resultados reales de movimiento se necesita comprobar seguimiento completo y separación. Si falla, NO ejecutes otra prueba completa hasta revisar causas.

## Observaciones de seguridad
- La detección de publicadores rivales y el relay son **software best-effort**; no hay lease DDS exclusivo ni watchdog físico de Gazebo ante crash del propio relay.
- Se vigilan odometrías y marcos, velocidad 1,20 m/s de comandos y separación geométrica >=1,15 m mediante `gate`; certificaciones offline siguen siendo necesarias.
- Gazebo debe estar visible y los tres robots deben poder desplazarse sin obstáculos. Mantén acceso a pausar simulación.
