# D.9.7B.2 — ensayo corto del relay de parada (solo Gazebo)

Este código es una **prueba de actuación, no un sistema de seguridad certificado**. Los modelos y ejecutor D.8 no se modifican. El relay es un proceso ROS separado: escucha `/d97b2/summit0/request`, publica `/summit0/cmd_vel` y deja de enviar velocidades no nulas al perder la petición durante 0.30 segundos. Publica ceros a 20 Hz. Si **el relay también muere**, no existe protección independiente del propio relay; NO considerar probado el timeout interno de Gazebo.

Primero detener la misión D.8 y sus seguidores; usar mundo semilla 100 y robot Summit 0 alejado de obstáculos. El puente `bridge_summit0` debe estar funcionando, y el reloj `/clock` debe avanzar. NO iniciar `d97b1_zero_actuator.py` en paralelo ni otra fuente de `/summit0/cmd_vel`.

Instalar: `unzip -o ~/Descargas/swarm_D97B2_ensayo_parada.zip -d ~/swarmbots` (desde anfitrión). Dentro de Docker:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m unittest discover -s scripts/tests -p test_d97b2_watchdog.py -v
ros2 topic info /summit0/cmd_vel -v
```

**Terminal A** (iniciar primero):

```bash
python3 scripts/d97b2_stop_watchdog.py --active-sim-only \
  --output scripts/resultados/d9_live/d97b2_stop_test.json
```

**Terminal B**, únicamente después de ver al nodo relay con `ros2 node list | grep d97b2_independent_zero_watchdog` y comprobar en `ros2 topic info /summit0/cmd_vel -v` que es el ÚNICO publicador:

```bash
python3 scripts/d97b2_short_pulse.py --send-short-pulse
```

La fuente publica 0.08 m/s durante 0.6 s, después termina deliberadamente. El relay debería pasar a publicar solamente ceros después de 0.30 s sin petición, quedándose activo. Esperar a que termine Terminal A; leer el reporte.

```bash
cat scripts/resultados/d9_live/d97b2_stop_test.json
```

Si aparece `PASS_RELAY_TIMEOUT_STOP_OBSERVED_SIM_ONLY`, significa que se observaron movimiento, corte del pulso y velocidad medida final inferior a 0.03 m/s. Es una validación del relay, **NO un ensayo de muerte del relay ni un E-stop independiente**. Estados `INCONCLUSIVE_*` o `REJECT_*` requieren revisión antes de D.9.2. Para interrumpir: `Ctrl+C` en Terminal B; conservar A abierto para que ordene ceros. Si A deja de funcionar, cerrar publicación no garantiza que Gazebo se detenga: pausar Gazebo o reiniciar la simulación y evitar nuevos comandos.
