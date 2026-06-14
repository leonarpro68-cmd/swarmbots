# swarmbots

Simulación de **swarm robotics** sobre **ROS 2 Humble** + **Gazebo Fortress**: un enjambre de N **Summit XLS omnidireccionales (mecanum)** patrullando un almacén, sobrevolado por N **drones X3** que despegan solos y quedan en hover.

![Enjambre de Summits y drones X3 en el warehouse](captura.png)

## Qué incluye

- **Mundo warehouse** (`tugbot_warehouse`, por defecto): almacén MovAi con estanterías, carros, pallets y estación de carga. El edificio está vendorizado con colisiones primitivas para mantener RTF ≈ 1.
- **Summit XLS mecanum** (`summit0…N-1`): cinemática omnidireccional real (física, no bypass) con `MecanumDrive` + inyección de fricción direccional `fdir1` en el SDF. LiDAR 2D, odometría y joint states por robot.
- **Drones X3** (`drone0…N-1`): quadrotor vendorizado con `MulticopterVelocityControl`; despegue automático por altitud real y hover estable ~3.5 m sobre el anillo de summits.
- **Comportamiento de enjambre** (`swarm_behavior`, solo Summit por ahora): navegación reactiva go-to-goal con evasión de obstáculos por LiDAR (campos potenciales, aprovecha el strafe mecanum) y modo líder-seguidor. Destino fijado en vivo desde RViz con la herramienta "2D Goal Pose", sobre un mapa estático del almacén. _Implementado; validación en Gazebo en curso._
- Variantes secundarias: Summit XL skid-steer individual, enjambre de minibots y mundo `empty_arena`.

## Requisitos

- Ubuntu 22.04 con ROS 2 Humble, Gazebo Fortress (`ign gazebo` 6.x) y `ros-humble-ros-ign*`
- GPU NVIDIA recomendada (los `gpu_lidar` rinden mucho mejor)
- Alternativa portable: Docker + NVIDIA Container Toolkit (ver `swarm_ws/docker/`)

## Uso rápido

```bash
./swarm_ws/scripts/sim_native.sh 3      # build + warehouse con 3 summits y 3 drones
```

Equivale a:

```bash
cd swarm_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to swarm_description swarm_worlds summit_xl_description
source install/setup.bash
ros2 launch swarm_worlds sim_summit.launch.py n_robots:=3
```

Mover robots (en otra terminal, tras los mismos `source`):

```bash
ros2 topic pub /summit0/cmd_vel geometry_msgs/Twist '{linear: {x: 0.3, y: 0.2}}'   # avance + strafe
ros2 topic pub /drone0/cmd_vel  geometry_msgs/Twist '{linear: {x: 0.3}}'           # dron (frame del cuerpo)
ros2 topic pub /summit0/cmd_vel geometry_msgs/Twist '{}'                            # frenar (los comandos persisten)
```

Argumentos útiles del launch:

| Argumento | Por defecto | Descripción |
|---|---|---|
| `n_robots` | `3` | Nº de Summits |
| `n_drones` | `-1` | Nº de drones (`-1` = igual que `n_robots`) |
| `world` | `tugbot_warehouse` | Mundo (`empty_arena` disponible) |
| `center_x` / `center_y` | `0` / `18` | Centro del círculo de spawn |
| `headless` | `false` | Servidor sin GUI (usa `--headless-rendering`) |

## Comportamiento de enjambre + RViz

Con la sim ya corriendo, en otras dos terminales (tras los mismos `source`):

```bash
# Controladores: todos los summits van al mismo punto y esquivan obstáculos
ros2 launch swarm_behavior swarm_behavior.launch.py n_robots:=3 goal_x:=0.0 goal_y:=8.0
# …o modo líder-seguidor (summit0 va al punto, el resto le siguen):
ros2 launch swarm_behavior swarm_behavior.launch.py n_robots:=3 mode:=follow

# RViz con el mapa del almacén y control interactivo del destino
ros2 launch swarm_behavior rviz.launch.py n_robots:=3
```

En RViz pulsa **"2D Goal Pose"** y clica en el mapa para fijar el destino del enjambre en vivo. El mapa estático se genera sin simular con `python3 swarm_ws/scripts/make_warehouse_map.py` (rasteriza las colisiones del mundo a un occupancy grid).

## Tópicos por robot

`/<ns>/cmd_vel`, `/<ns>/odom`, `/<ns>/scan` (solo summits), `/<ns>/joint_states`, `/<ns>/tf` — con `<ns>` = `summit0…`, `drone0…`.

## Estructura

```
swarm_ws/
├── docker/                      # Dockerfile + compose (respaldo de portabilidad)
├── scripts/                     # sim_native.sh (primario), make_warehouse_map.py, build/run/sim.sh (Docker)
└── src/
    ├── swarm_worlds/            # mundos, launches del enjambre, mapas, modelos vendorizados (X3, warehouse)
    ├── swarm_behavior/          # navegación reactiva go-to-goal + RViz (enjambre Summit)
    ├── swarm_description/       # minibot diff-drive
    ├── summit_xl_description/   # Summit portado a Fortress (omni + skid-steer)
    └── robotnik_*/              # paquetes externos Robotnik
```

Más detalle técnico (decisiones de diseño, lecciones aprendidas) en [CLAUDE.md](CLAUDE.md).
