# D9.6B fix para ROS 2 Jazzy

Corrige la importación inexistente de `rclpy.qos.qos_profile_clock`.

La suscripción a `/clock` usa `QoSProfile(depth=10, reliability=BEST_EFFORT, durability=VOLATILE)`.
No añade publicadores ni modifica la ejecución D8.

Instalar desde host:

```
cd ~/swarmbots
cp swarm_ws/scripts/d96b_clock_readonly.py swarm_ws/scripts/d96b_clock_readonly.py.bak_qos
unzip -o ~/Descargas/swarm_D9_6B_fix_qos_jazzy.zip -d ~/swarmbots
```

Dentro de Docker:

```
cd ~/swarm_ws
python3 -m py_compile scripts/d96b_clock_readonly.py
python3 scripts/d96b_clock_readonly.py --run scripts/resultados/d8/seed_100_run_007 --stage pickup1
```

Requiere Gazebo y puente `/clock` en ejecución. Comprueba `ros2 topic echo /clock --once` antes de iniciar.
