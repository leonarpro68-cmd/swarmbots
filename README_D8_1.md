# D.8.1: confirmación Gazebo y cierre seguro

Parche sobre los archivos originales incluidos en `swarmbots.zip`; tocar solamente:
- `swarm_ws/src/swarm_behavior/swarm_behavior/grasp_manager.py`
- `swarm_ws/src/swarm_worlds/launch/swarm_cbs_grasp_base.launch.py`

## Cambios
- `attach_timeout` sube de 2.5 a 5.0 s (en el nodo y explícitamente en el launch); `attach_retry` continúa en 0.35 s.
- Ante timeout de attach o detach, la operación queda **incierta y bloqueada**. El gestor mantiene la suscripción al estado del joint para registrar un ACK tardío; si llega, queda una alerta, no una autorización automática de transporte.
- Un nuevo `grasp` o `release` en estado incierto se rechaza con resultado -1. No se abre la pinza automáticamente.
- Evita `rclpy.shutdown()` doble al apagar.

## Instalación en host Ubuntu (ajustar Descargas)
```bash
cd ~/swarmbots
cp swarm_ws/src/swarm_behavior/swarm_behavior/grasp_manager.py swarm_ws/src/swarm_behavior/swarm_behavior/grasp_manager.py.bak_D8_1
cp swarm_ws/src/swarm_worlds/launch/swarm_cbs_grasp_base.launch.py swarm_ws/src/swarm_worlds/launch/swarm_cbs_grasp_base.launch.py.bak_D8_1
unzip -o ~/Descargas/swarm_hybrid_v2_D8_1_grasp_timeout.zip -d ~/swarmbots
```

Dentro de Docker:
```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
python3 -m py_compile src/swarm_behavior/swarm_behavior/grasp_manager.py src/swarm_worlds/launch/swarm_cbs_grasp_base.launch.py
colcon build --packages-select swarm_behavior swarm_worlds --symlink-install
source install/setup.bash
```

No modificar una ejecución en curso. Cerrar Gazebo y volver a iniciarlo con D.8; las modificaciones requieren recrear los tres grasp_manager.

## Verificación
Con el nuevo Gazebo abierto:
```bash
for i in 0 1 2; do ros2 param get /grasp_manager_summit$i attach_timeout; done
```
Debe dar `Double value is: 5.0` en los tres.

Ejecutar primero una misión de prueba supervisada:
```bash
python3 scripts/run_swarm_seed_d8.py --seed 123 --auto-confirm --execute
```

Nota: ni la ampliación de timeout ni el ACK tardío prueban el estado físico del joint. Si aparece `ESTADO INCIERTO`, `ACK TARDIO`, o una etapa falla, detener y revisar logs de Gazebo. No activar reintentos ciegos, no marcar la carga como recogida y no reanudar sobre el estado parcial.

### Alcance de las pruebas
Las cuatro pruebas suministradas son estáticas/de regresión de código y se superaron. No se ha validado una ejecución integral real dentro del Docker del usuario.
