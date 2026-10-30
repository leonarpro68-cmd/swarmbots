# Fase 5.1: hasta 1.20 m/s + PNG CBS vs Bézier

Cambios sobre Fase 5: parámetros sincronizados en coordinador, protocolo y launch.
No se eliminan verificaciones de colisión, odometría, WAIT ni seguimiento.
El factor temporal `--time-safety-factor` es >=1: nunca ejecuta una referencia
más rápida que las cotas matemáticas configuradas.

## Ejecución aislada (dentro de swarm_dev)

Compilar los paquetes modificados:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select swarm_behavior swarm_worlds
source install/setup.bash
python3 -c 'import matplotlib; print("matplotlib OK")'
```

**Terminal 1:** iniciar el mismo escenario, pero configurar máximo del seguidor:

```bash
ros2 launch swarm_worlds swarm_cbs_grasp_base.launch.py \
  seed:=123 scenario:=/tmp/swarm_hybrid_123_scenario.json \
  world_sdf:=/tmp/swarm_hybrid_123_world.sdf \
  hybrid_enabled:=true hybrid_max_speed:=1.20
```

**Terminal 2:** puente `/clock` si no está funcionando desde el lanzamiento.

**Terminal 3:** (sin `run_swarmbots.sh` simultáneo):

```bash
ros2 run swarm_behavior swarm_hybrid_ros_demo \
  --scenario /tmp/swarm_hybrid_123_scenario.json \
  --iterations 2 \
  --max-speed 1.20 --max-accel 1.00 \
  --time-safety-factor 1.08 \
  --tracking-error 0.16 \
  --output-dir /tmp/swarm_hybrid_results
```

El coordinador guarda ANTES de mandar la orden:

- `/tmp/swarm_hybrid_results/hybrid-<ID>_cbs_vs_bezier.png`
- `/tmp/swarm_hybrid_results/hybrid-<ID>_plan.json`

Son archivos **dentro del contenedor**. Para abrirlos en el host:

```bash
docker cp swarm_dev:/tmp/swarm_hybrid_results ~/swarmbots/resultados/hybrid_fase51
```

La salida PNG muestra la **etapa de recogida** de la demo aislada, no las
cuatro fases de la misión completa. El JSON distingue planificación de ejecución.

## Advertencias

1. 1.20 m/s es el techo del comando de velocidad, no una garantía de velocidad física ni de reducir a la mitad el tiempo de misión.
2. La factibilidad dinámica solo comprueba velocidad/aceleración cartesiana; no demuestra límites de ruedas ni control de Gazebo.
3. Si la prueba falla o aumenta el error de seguimiento, detenerla y bajar velocidad/aceleración. Empezar con pruebas aisladas sin manipulación.
4. No ejecutar con otros controladores publicando `cmd_vel`.
