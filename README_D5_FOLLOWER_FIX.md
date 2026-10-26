# D.5: permitir observador sin duplicar seguidores

El ejecutor anterior exigía `get_subscription_count()==1`, lo que rechazaba el observador D.5.
Ahora examina `get_subscriptions_info_by_topic()` y exige exactamente un endpoint `path_follower` en el namespace del Summit con tipo `std_msgs/msg/String`. Los observadores adicionales no bloquean el preflight. Se conservan todas las demás verificaciones de seguridad.

## Instalación (host Ubuntu)

```bash
cd ~/swarmbots
unzip -o ~/Descargas/swarm_hybrid_v2_D5_follower_guard_fix.zip -d ~/swarmbots
```

## Dentro de Docker

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PYTHONPATH"
python3 -m py_compile scripts/run_hybrid_v2_stage_ros.py
ros2 topic info /summit0/hybrid_trajectory/command -v
```

Con el observador y Gazebo activos, repetir el preflight habitual SIN `--execute`:

```bash
python3 scripts/run_hybrid_v2_stage_ros.py \
 --report scripts/resultados/d4_live/pickup1_live_report.json \
 --controls scripts/resultados/d4_live/pickup1_live_controls.npz \
 --stage pickup1 \
 --scenario /tmp/swarm_hybrid_123_scenario.json
```

Si falla por inicio diferente, regenerar el plan desde odometría. No usar `--execute` hasta que el preflight y la coherencia del escenario sean correctos.
