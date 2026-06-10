# AGENTS.md

Swarm robotics project: ROS2 Humble + Gazebo Fortress, containerized.

## First-time setup

```bash
./swarm_ws/scripts/build.sh         # build Docker image (~10-15min)
./swarm_ws/scripts/run.sh           # interactive shell with X11+GPU
```

Container name: `swarm_dev`, image: `swarmbots/humble-fortress:latest`.

## Scripts run from host; colcon runs inside container

| Host command | What it does |
|---|---|
| `./scripts/run.sh` | Attach to running container or start new one with X11+GPU |
| `./scripts/build.sh` | `docker compose build` (sets UID/GID from host) |
| `./scripts/colcon_build.sh` | Runs `colcon build --symlink-install` inside container |

Inside container after colcon build, always `source install/setup.bash` — the entrypoint auto-sources it if present.

## Package dirs

- `src/swarm_description/` — URDF/Xacro, meshes, robot models
- `src/swarm_worlds/` — world files (`.sdf` / `.world`)

Both are empty (only `.gitkeep`). ROS 2 packages go here.

## Validation inside container

```bash
ros2 doctor --report                 # ROS diagnostics
ign gazebo --version                 # should print 6.x
ign gazebo -v 4 shapes.sdf          # GUI with GPU
ros2 run ros_ign_bridge parameter_bridge --help  # ROS↔Gazebo bridge
```

## Key constraints

- **Docker-first**: everything must work inside the container. Never assume host packages.
- **ROS2 Humble only**: no ROS1, no ROS2 rolling. Use `ros-humble-*` packages.
- **Gazebo Fortress (Ignition 6)**: primary simulator. `ign` commands, not `gz`.
- **No GPU?** Remove `deploy.resources.devices` block from `docker-compose.yml`.
- **Multi-robot DDS**: set `ROS_DOMAIN_ID` env var (default `0`). `network_mode: host` simplifies this.
- **X11**: `xhost +local:docker` is handled by `run.sh` via trap.
- **URI**: after colcon build, run `source install/setup.bash` else nodes won't be found.
- **xacro → URDF**: robot models in `.xacro` that compile to valid URDF; include inertias, collisions, sensors, Gazebo plugins.
- **Worlds**: `.world` (SDF/XML) with lighting, ground plane, ≥1 obstacle.

## .gitignore excludes

`build/`, `install/`, `log/`, `__pycache__/`, `.vscode/`, `.idea/`, `docker/docker-compose.override.yml`, `.env`. Commit only `src/` and config.

## Gazebo plugin rule

Use only officially documented Gazebo plugins. Never invent plugin names.

## No git repo yet

The workspace skeleton exists but has never been committed. Initialize with `git init` at root before first commit.
