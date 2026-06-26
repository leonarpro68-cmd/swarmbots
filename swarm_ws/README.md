# swarm_ws

Workspace de **swarm robotics** sobre **ROS 2 Jazzy** + **Gazebo Harmonic**, containerizado con Docker + NVIDIA Container Toolkit.

## Requisitos del host

- Docker Engine + plugin `compose`
- NVIDIA Container Toolkit (`nvidia-ctk`) configurado para Docker
- X11 (Linux con servidor X corriendo)

## Estructura

```
swarm_ws/
├── docker/
│   ├── Dockerfile           # ROS 2 Jazzy + Gazebo Harmonic + ros_gz
│   ├── docker-compose.yml   # X11, GPU NVIDIA, bind mount del workspace
│   └── entrypoint.sh        # source de ROS y del overlay
├── src/
│   ├── swarm_description/   # URDFs / Xacros / meshes (vacío)
│   └── swarm_worlds/        # mundos .sdf / .world      (vacío)
└── scripts/
    ├── build.sh             # docker compose build
    ├── run.sh               # arranca/atacha al contenedor con X11+GPU
    └── colcon_build.sh      # colcon build --symlink-install
```

## Uso

```bash
# 1. Construir la imagen (la primera vez ~10-15 min)
./scripts/build.sh

# 2. Abrir un shell en el contenedor (X11 + GPU listos)
./scripts/run.sh

# 3. Dentro del contenedor: compilar el workspace
cd /home/dev/swarm_ws && colcon build --symlink-install
source install/setup.bash
```

## Validaciones rápidas dentro del contenedor

```bash
# ROS 2
ros2 doctor --report

# Gazebo Harmonic
gz sim --version              # debe imprimir 8.x
gz sim -v 4 shapes.sdf       # GUI con GPU

# Puente ros_gz
ros2 run ros_gz_bridge parameter_bridge --help
```

## Notas

- El contenedor corre como usuario `dev` con el mismo UID/GID del host para que los archivos creados desde dentro queden con tus permisos.
- `network_mode: host` simplifica DDS multi-robot; cambia a `bridge` si necesitas aislar tráfico.
- Para correr sin GPU, elimina el bloque `deploy.resources` de `docker/docker-compose.yml`.
