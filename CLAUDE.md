# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

# Asesor — Swarm Robotics / ROS2 Jazzy / Gazebo Harmonic & Isaac Sim / Docker

## Estado actual del repo

> **MIGRACIÓN Humble/Fortress → Jazzy/Harmonic: HECHA a nivel de código (2026-06-25), SIN validar en vivo.** Se migraron todos los nombres de plugin (`libignition-gazebo-*-system.so` → `gz-sim-*-system`, `ignition::gazebo::systems::*` → `gz::sim::systems::*`), la CLI (`ign sdf -p`/`ign topic` → `gz sdf -p`/`gz topic`), las env vars (`IGN_GAZEBO_RESOURCE_PATH` → `GZ_SIM_RESOURCE_PATH`), el wrapper de launch (`ros_ign_gazebo`/`ign_gazebo.launch.py`/`ign_args` → `ros_gz_sim`/`gz_sim.launch.py`/`gz_args`), el bridge (`ros_ign_bridge` → `ros_gz_bridge`, tipos `ignition.msgs.*` → `gz.msgs.*`), la fricción mecanum (`xmlns:ignition`/`ignition:expressed_in` → `xmlns:gz='http://gazebosim.org/schema'`/`gz:expressed_in`) y Docker (base `jazzy-desktop`, `gz-harmonic`, `ros-jazzy-ros-gz*`). **PENDIENTE en la nueva máquina**: instalar el stack (`sudo apt install gz-harmonic ros-jazzy-ros-gz ros-jazzy-xacro ...`), migrar la caché de Fuel (`cp -r ~/.ignition/fuel/* ~/.gz/fuel/`) y validar end-to-end. Hoy en `isa` solo está ROS 2 Jazzy base (sin `ros_gz` ni `xacro`), por eso no se pudo compilar ni lanzar.

**Funcionaba end-to-end en el stack viejo (vía nativa)**: `./swarm_ws/scripts/sim_native.sh 3` abre Gazebo (warehouse `tugbot_warehouse` por defecto) y un **enjambre de N Summit XLS omnidireccionales (mecanum)** en círculo — `/summitN/scan`, `/summitN/odom`, `/summitN/cmd_vel` (incluido strafe lateral en Y) — más **N drones X3 que despegan solos y sobrevuelan el enjambre en hover** (`/droneN/cmd_vel`, `/droneN/odom`). Validado en `isa` (Ubuntu 22.04, NVIDIA 580, Fortress 6.16.0, ROS 2 Humble) con RTF ≈ 0.99 **antes de la migración**. También disponibles: enjambre de minibots (`sim.launch.py`), Summit XL skid-steer individual (`spawn_summit.launch.py`) y el mundo `empty_arena` (`world:=empty_arena center_x:=0 center_y:=0`).

Estructura:

- `swarm_ws/docker/` — Pipeline Docker (Dockerfile + docker-compose + entrypoint). **No se ha construido la imagen**; en `isa` no hace falta porque la vía nativa cubre todo. Existe como fallback de portabilidad.
- `swarm_ws/scripts/` — `sim_native.sh` (host, **primaria**; ahora compila también `swarm_behavior`), `make_warehouse_map.py` (genera el occupancy grid estático del warehouse sin simular), `build.sh` + `run.sh` + `colcon_build.sh` + `sim.sh` (Docker, secundarias).
- `swarm_ws/src/swarm_description/` — paquete propio: `urdf/minibot.urdf.xacro` (diff-drive + LiDAR 2D, plugins **nativos Harmonic**: `gz-sim-diff-drive-system`, `gz-sim-joint-state-publisher-system`).
- `swarm_ws/src/swarm_worlds/` — paquete propio: `worlds/tugbot_warehouse.sdf` (**mundo por defecto**, almacén MovAi adaptado) y `worlds/empty_arena.sdf` (suelo + sol + 2 cajas; ambos con plugins de mundo, **incluido `Sensors` que va aquí UNA VEZ**, no por robot) + `launch/sim.launch.py` que spawnea N minibots en círculo y monta los bridges ROS↔Gz + `models/x3_uav/` (quadrotor X3 de Open Robotics **vendorizado** — meshes con URIs relativas, sin dependencia de Fuel/internet) + `models/warehouse/` (edificio del almacén **vendorizado con colisiones primitivas**) + `launch/spawn_drone.launch.py`.
- `swarm_ws/src/swarm_behavior/` — paquete propio (Python): comportamiento de enjambre. Nodo `go_to_goal` (campos potenciales holonómico, aprovecha el strafe mecanum) + `launch/swarm_behavior.launch.py` (1 controlador por summit, modos `goal`/`follow`) + `launch/rviz.launch.py` (RViz con mapa estático + control del destino por "2D Goal Pose"). **Implementado, aún sin validar en Gazebo vivo.**
- `swarm_ws/src/{robotnik_common,robotnik_sensors,summit_xl_description,summit_xl_control}/` — paquetes externos Robotnik. **Summit ya portado a Fortress** en dos variantes propias: `robots/summit_xl_omni.urdf.xacro` (XLS mecanum/omni — **el robot del enjambre**, vía `sim_summit.launch.py`) y `robots/summit_xl_noarm.urdf.xacro` (XL skid-steer 4 ruedas con DiffDrive de joints agrupados 2L+2R, vía `spawn_summit.launch.py`). Siguen **sin portar** (y no se usan): `summit_xl_base.gazebo.xacro` (plugins Classic), `all_sensors.urdf.xacro` (Classic), `ros2_control.urdf.xacro` (declara joints de brazo inexistentes), `summit_xl_control/launch/*.launch` (XML ROS 1).

### Lecciones aprendidas (no repetir)
- El `MecanumDrive` de **Fortress no publica odometría**; por eso se añadió un `OdometryPublisher` aparte (que da la **pose mundial** del spawn, no cero). **⚠️ REGRESIÓN EN HARMONIC (2026-06-25):** el `MecanumDrive` de Harmonic **SÍ publica odom/tf por su cuenta** (dead-reckoning en frame del spawn) en los tópicos por defecto `/model/<ns>/odometry` y `/model/<ns>/tf`. Como el `OdometryPublisher` usa esos mismos tópicos, **colisionan**: el bridge recibe DOS publicadores y la odom sale **alternando mundo/relativa** (`gz topic -i -t /model/<ns>/odometry` muestra 2 publishers) → el comportamiento recibe basura y los robots "bailan"/no llegan. **Fix:** redirigir la odom/tf del MecanumDrive a tópicos propios no usados (`<odom_topic>/model/${ns}/mecanum_odom</odom_topic>`, `<tf_topic>/model/${ns}/mecanum_tf</tf_topic>`) para dejar `/model/<ns>/odometry` solo al OdometryPublisher. Lo mismo aplicaría a DiffDrive si se le pone OdometryPublisher al lado. Diagnóstico: leer `/<ns>/odom` con el robot quieto; si alterna entre la pose mundo y (0,0), es esto.
- La fricción direccional mecanum (`<fdir1 gz:expressed_in="...">`, namespace `xmlns:gz='http://gazebosim.org/schema'`) **no es expresable en URDF** — el parser de sdformat pierde el atributo. Solución: pipeline xacro → URDF → `gz sdf -p` → inyectar `fdir1` con regex → spawn por `-file` (ver `sim_summit.launch.py`). Patrón de rodillos en X: FL/BR `1 -1 0`, FR/BL `1 1 0`, con `mu1=1.0`/`mu2=0.0` (copiado del demo `mecanum_drive.sdf` de gz-sim8).
- Al convertir URDF→SDF, los links unidos por joints fijos se fusionan (lumping) en el link raíz: el "chassis" del SDF resultante se llama `base_footprint`, no `base_link` (importa para `expressed_in`).
- En Fortress, el plugin `Sensors` va **en el `<world>` del SDF**, NO en el `<gazebo>` del URDF del robot. Ponerlo en el URDF y spawnear N>1 robots dispara SIGINT a Gazebo.
- SDF en Fortress soporta hasta `version="1.8"`. La 1.9 es Garden/Harmonic — Gazebo sale "limpio" (exit 0) sin warning claro.
- `<gz_frame_id>` en sensores es tag de Harmonic; en Fortress se omite.
- Para `IncludeLaunchDescription(gz_sim.launch.py)`, pasar `gz_args` como **string ya resuelto**, no como lista de substituciones — esto último confunde el wrapper.
- `catkin_pkg` rechaza emails sin TLD válido (ej. `dev@local`). Usar uno real.
- **Spawnear un robot dentro de una colisión estática hunde el RTF de TODA la sim** (a ~0.06): la interpenetración profunda dispara el solver de contactos de DART en cada paso. Los modelos Fuel pueden tener cajas de colisión mucho mayores que su visual (las `shelf_big` del warehouse miden 2.1×18×6 m — pasillos enteros). Antes de elegir un punto de spawn, comprobar las cajas de colisión de los `<include>`, no solo los visuales. Para diagnosticar RTF bajo: `gz topic -e -t /world/<mundo>/stats` y bisecar el mundo quitando includes.
- Los `sleep` de **reloj de pared no sirven para secuencias dependientes del tiempo de sim** (el RTF puede ser <1): el despegue de los drones corta la subida leyendo la altitud real por odometría (`gz topic -e ... | awk`), no con `sleep 4`.
- En headless, añadir `--headless-rendering` a `gz sim -s`: sin él los `gpu_lidar` caen a render por software (libEGL "failed to create dri2 screen" en el log y GPU sin uso en `nvidia-smi`).
- `model://` en mundos requiere exportar `GZ_SIM_RESOURCE_PATH` con el dir `models` del paquete (lo hace `sim_summit.launch.py` vía `os.environ` antes de lanzar Gazebo).

## Comandos comunes

Hay **dos vías** para correr la sim. La máquina objetivo lleva ROS 2 Jazzy + Gazebo Harmonic + drivers NVIDIA → la vía nativa es la rápida; Docker queda como respaldo de portabilidad. (Tras la migración, en `isa` falta instalar `ros-jazzy-ros-gz` + `gz-harmonic` + `ros-jazzy-xacro` para poder compilar/lanzar.)

### Vía A — nativa en el host (recomendada en `isa`)
```bash
./swarm_ws/scripts/sim_native.sh 3    # compila y lanza ros2 launch en el host
```
Equivale a:
```bash
cd swarm_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select swarm_description swarm_worlds
source install/setup.bash
ros2 launch swarm_worlds sim.launch.py n_robots:=3
```

### Vía B — Docker (portable)
- Construir imagen: `./swarm_ws/scripts/build.sh` (requiere `docker compose` v2 + NVIDIA Container Toolkit)
- Shell en contenedor: `./swarm_ws/scripts/run.sh`
- Simular: `./swarm_ws/scripts/sim.sh 3`

### Mover robots (cualquier vía, en otra terminal)
```bash
source /opt/ros/jazzy/setup.bash && source ~/swarmbots/swarm_ws/install/setup.bash
ros2 topic pub /robot0/cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}, angular: {z: 0.3}}'
```

## Convenciones del minibot
- Namespace por robot: `robot0`, `robot1`, … (asignado por el launch)
- Tópicos ROS ya remapeados al namespace: `/<ns>/cmd_vel`, `/<ns>/odom`, `/<ns>/scan`, `/<ns>/joint_states`, `/<ns>/tf`
- Frame prefix: `<ns>/base_footprint`, `<ns>/odom`, `<ns>/lidar_link`

---

## Enjambre Summit XLS omni en Fortress (HECHO — 2026-06-09, primario)

**Validado headless en `isa` con 3 robots**: spawn en círculo (radio ≥3.5 m, mirando al centro), `/summitN/scan` publica, avance X (2.4 m en 5 s a 0.5 m/s), **strafe lateral Y** (2.39 m con deriva frontal de 0.2 mm) y giro (+114° en 4 s a 0.5 rad/s) — la cinemática mecanum funciona de verdad (física, no bypass). Los comandos `cmd_vel` **persisten** hasta recibir otro (mandar Twist en cero para frenar).

```bash
./swarm_ws/scripts/sim_native.sh 3                          # build + enjambre con GUI
ros2 launch swarm_worlds sim_summit.launch.py n_robots:=3 [headless:=true]
ros2 topic pub /summit0/cmd_vel geometry_msgs/Twist '{linear: {x: 0.3, y: 0.2}}'
```

Piezas: `summit_xl_description/robots/summit_xl_omni.urdf.xacro` (base XLS + 4 ruedas mecanum con macro propio + LiDAR 2D + `MecanumDrive` + `OdometryPublisher` + JSP) y `swarm_worlds/launch/sim_summit.launch.py` (genera URDF y SDF por robot en `/tmp/swarm_summit/`, inyecta `fdir1`, spawnea N con bridges). Namespaces `summit0…N-1`, mismos tópicos/frames que el minibot.

## Enjambre de drones X3 sobrevolando (HECHO — 2026-06-09)

Quadrotor **X3 UAV** vendorizado en `swarm_worlds/models/x3_uav/` (descargado de Fuel una vez; URIs de meshes relativas → autocontenido y Docker-safe). `sim_summit.launch.py` ahora spawnea también **N drones** (`n_drones:=-1` → igual que `n_robots`), namespaces `drone0…N-1`, que **despegan solos y quedan en hover ~3.5 m** sobre el anillo de summits. Validado con 3+3: hover estable (deriva <0.1 mm en 8 s), `/droneN/cmd_vel` y `/droneN/odom` por bridge, Summits intactos.

Cómo vuela: a cada model.sdf se le inyectan los plugins nativos Fortress `MulticopterMotorModel` (×4) + `MulticopterVelocityControl` + `OdometryPublisher` (config copiada del demo `multicopter_velocity_control.sdf` de ign-gazebo6; los links internos del modelo se llaman `X3/rotor_N`). **El controlador no actúa hasta recibir el primer Twist** → los drones spawnean en el suelo (1 m por fuera del anillo) y un `ExecuteProcess` por dron espera su odometría, manda subida 0.7 m/s **hasta que la odometría supera 2.2 m** (corte por altitud real, robusto a RTF<1; con la latencia del sondeo quedan a ~3.4–3.9 m) y luego Twist cero = hover. El `cmd_vel` del dron es velocidad en frame del cuerpo (z sube/baja); Twist cero = hover, persiste como en los Summits.

```bash
ros2 launch swarm_worlds sim_summit.launch.py n_robots:=3              # 3 summits + 3 drones
ros2 topic pub /drone0/cmd_vel geometry_msgs/Twist '{linear: {x: 0.3, z: 0.2}}'
# dron extra con la sim corriendo (no lanza Gazebo; reusa sim_summit vía importlib):
ros2 launch swarm_worlds spawn_drone.launch.py name:=drone_extra x:=1.0 y:=2.0
```

## Mundo warehouse por defecto (HECHO — 2026-06-10)

`worlds/tugbot_warehouse.sdf` (del zip `tugbot_warehouse.zip` del usuario, mundo demo del Tugbot de MovAi): almacén con estanterías, carros, pallets y estación de carga. Adaptaciones: se quitó el `<include>` del robot Tugbot, `max_step_size` 0.01→0.004 (igual que el demo multicóptero) y el edificio se sustituyó por `models/warehouse/` vendorizado. Es el **mundo por defecto** de `sim_summit.launch.py` (args: `world:=tugbot_warehouse|empty_arena`, `center_x`/`center_y` = centro del círculo de spawn, por defecto **(0, 18)** = zona norte abierta).

**Validado headless en `isa` 3+3 (2026-06-10): RTF 0.99**, summits sobre el suelo del almacén (caja de colisión propia, tope z≈0.009), mecanum OK (avance+strafe simultáneos con ratio exacto), drones hover estable (deriva 2e-5 m/8 s), `/summit0/scan` ve la geometría del almacén (~8 m).

Detalles importantes:
- **Edificio vendorizado** (`models/warehouse/`): la colisión original era UNA malla STL (3192 tris) con AABB 30×50×12.6 m que envuelve todo el interior; se reemplazó por primitivas (caja de suelo con tope a z=0.099 en frame del modelo + 4 paredes a x=±15/y=±25 de altura completa, los drones no pueden escapar). Visuales intactos (el lidar raya contra visuales, no colisiones). También elimina la dependencia de Fuel para el edificio.
- Los demás modelos (shelf, shelf_big, cart, pallets, charging_station) **siguen viniendo de Fuel** (en Harmonic cacheados en `~/.gz/fuel/`; antes en Fortress era `~/.ignition/fuel/`. Primera ejecución necesita internet o la caché ya poblada — `cp -r ~/.ignition/fuel/* ~/.gz/fuel/` migra la caché vieja. En Docker habría que vendorizarlos también o montar la caché).
- **NO mover el centro de spawn sin comprobar las colisiones**: las `shelf_big` tienen cajas de 2.1×18×6 m (shelf_big_2/3/4 cubren y∈[-22,-4] en x≈{4.7..6.8, -1..1.1, -6.9..-4.8}). Spawnear dentro hunde el RTF a ~0.06 y atrapa a los robots (así se descubrió: el centro anterior (1,-6.5) caía dentro de shelf_big_3).

## Comportamiento de enjambre + RViz (HECHO y VALIDADO EN VIVO — drones añadidos 2026-06-25)

Paquete `swarm_behavior` (ament_python). **Un mismo goal mueve summits Y drones.** Validado en vivo (Jazzy/Harmonic, headless): un clic de "2D Goal Pose" reubica todo el enjambre; summits convergen por el suelo y los drones llegan al mismo (x,y) en capas de altura separadas (drone0 z=3.5, drone1 z=4.5, …). Cambio dinámico de goal confirmado (publicar `/goal_pose` mueve a todos). Pendiente de afinar fino en GUI: alineación `/map` vs nubes LiDAR en RViz y ganancias de evasión de los summits.

**Nodo `go_to_goal`** (uno por summit, dentro de su namespace): controlador por **campos potenciales holonómico**. Lee `odom` (frame mundo) + `scan`, publica `cmd_vel`. Suma atracción al goal + repulsión de cada rayo < `influence_radius` (1.5 m), rota el vector a frame cuerpo y manda `vx, vy` (**usa el strafe mecanum para esquivar**) + `wz` para encarar el avance (mantiene el FOV de 270° mirando adelante). Parada de seguridad si obstáculo frontal < 0.4 m. Escucha `/goal_pose` (PoseStamped) → reubica el goal en vivo.

**Formación en ANILLO + latch (2026-06-25)** — resuelve el "baile" cuando varios summits comparten goal: apuntar todos al mismo punto exacto hace que el anti-colisión (repulsión LiDAR, se ven entre sí) les impida amontonarse y oscilen sin converger. Solución: cada summit no va al centro sino a su **slot en un anillo** alrededor del goal (ángulo `2π·robot_index/n_robots`, radio = `formation_spacing/(2·sin(π/n))`; con `formation_spacing=2.0` > influence, en reposo no se repelen). Además un **latch de llegada con histéresis** (`arrive_hysteresis=0.6`): al entrar en `goal_tol` se queda QUIETO y no se reactiva hasta que el goal se aleje > `goal_tol+hist` (evita entrar/salir de tolerancia = bailar). El launch pasa `robot_index`, `n_robots`, `formation_spacing`. Validado en vivo: 3 summits rodean el goal y quedan inmóviles (lecturas idénticas a +6 s); goal dinámico re-converge. **Requería el fix de odom de arriba** (sin él, odom basura = no llegan).

**Nodo `drone_go_to_goal`** (uno por dron, dentro de su namespace): go-to-goal **3D** sin LiDAR (vuela por encima de los obstáculos). Lee `odom`, publica `cmd_vel`. Atracción horizontal al mismo goal (x,y) que los summits, rotada a **frame cuerpo** (verificado empíricamente 2026-06-25: el `cmd_vel` del dron es body frame con yaw, igual que el summit) + control de altitud `vz = k_z·(cruise_z − z)` que **siempre se republica** para sostener el hover. Cada dron cruza a `cruise_z + i·drone_layer` (capas, def 3.5 + i·1.0 m) para no colisionar. Escucha `/goal_pose` igual que los summits.

**Modos** (`swarm_behavior.launch.py`, solo afectan a summits):
- `mode:=goal` (def): todos los summits al mismo punto `goal_x/goal_y`.
- `mode:=follow`: `summit0` va al punto; el resto **persiguen la odom de summit0** a `standoff` m (líder-seguidor, **sin SLAM** — la odom ya está en frame mundo). NO es "uno mapea y los demás navegan el mapa" (eso sería SLAM+Nav2 multi-robot, fase futura).
- Los **drones siempre van al goal** en ambos modos. Args extra: `n_drones` (-1 = igual que n_robots, 0 = sin drones), `cruise_z`, `drone_layer`.

**RViz** (`rviz.launch.py`): frame fijo `map`. Como la odom de cada robot está en frame mundo, todos los `summitN/odom` coinciden con `map` → **TF estático identidad `map→summitN/odom`** por robot. El `/tf` está namespaced (`/summitN/tf`) → se relé al `/tf` global con `topic_tools relay` (idem `/tf_static`). El `frame_id` del scan es el nombre escopado del sensor (`summitN/base_footprint/lidar`, NO existe en el URDF) → se ata por **identidad** a `summitN/lidar_link` (mismo punto físico, ya colocado por robot_state_publisher). El mapa estático se sirve con `nav2_map_server` + `nav2_lifecycle_manager` (autostart) en `/map`.

**Mapa estático** (`scripts/make_warehouse_map.py` → `swarm_worlds/maps/warehouse.{pgm,yaml}`, 640×1040 @ 0.05 m): generado **sin simular**. `gz sdf -p` NO resuelve los `<include>` desde CLI (callback vacío) → se parsea a mano: poses del mundo + colisiones de cada model.sdf (warehouse vendorizado + modelos MovAi del cache de Fuel), componiendo mundo∘link∘colisión y rasterizando box/cylinder/mesh-AABB. **Filtro de altura clave** (`Z_BAND=0.20–0.70 m`): excluye suelo (tope ~0.01 m) y pallets/rieles bajos, incluye estanterías/paredes/cajas altas. Regenerar tras tocar el mundo: `python3 scripts/make_warehouse_map.py && colcon build --packages-select swarm_worlds`.

```bash
# Terminal 1: sim   ·   Terminal 2: comportamiento   ·   Terminal 3: RViz
./swarm_ws/scripts/sim_native.sh 3
ros2 launch swarm_behavior swarm_behavior.launch.py n_robots:=3 [mode:=follow] [goal_x:=0 goal_y:=8]
ros2 launch swarm_behavior rviz.launch.py n_robots:=3 [map:=/ruta/otro.yaml]
```
En RViz: botón **"2D Goal Pose"** → clic en el mapa = nuevo destino del enjambre (en `goal` todos, en `follow` el líder).

## Summit XL skid-steer en Fortress (HECHO — 2026-06-09, secundario)

**Validado headless en `isa`**: spawn OK, `/summit/scan` publica, `/summit/cmd_vel` a 0.5 m/s durante 6 s → odometría avanza 3.02 m, `/summit/joint_states` reporta las 4 ruedas. Sin errores en el log de Gazebo.

```bash
ros2 launch swarm_worlds spawn_summit.launch.py            # con GUI
ros2 launch swarm_worlds spawn_summit.launch.py headless:=true
ros2 topic pub /summit/cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}}'
ros2 topic echo /summit/scan --once
```

Decisiones de diseño (por si hay que retocarlo):
- `summit_xl_noarm.urdf.xacro` es un robot **nuevo y limpio** (no parte de `summit_xl_std.urdf.xacro`): incluye solo `summit_xl_base.urdf.xacro` + `rubber_wheel.urdf.xacro` (ninguno trae plugins Classic) y añade LiDAR 2D propio. Se descartó incluir `all_sensors.urdf.xacro` (helios/zed2 con plugins Classic) y `ros2_control.urdf.xacro` (declara joints `arm_*` que no existen sin brazo → gz_ros2_control fallaría, y tiene tópicos `model/summit/...` hardcodeados de una sesión anterior).
- Skid-steer 4 ruedas resuelto con DiffDrive nativo y **tags `<left_joint>`/`<right_joint>` repetidos** (2L+2R). Funciona en Fortress sin truco adicional. `wheel_separation` = 2×0.218 = 0.436, `wheel_radius` = 0.11.
- Las meshes cargan vía `file://$(find summit_xl_description)/...` (resuelto por xacro a ruta absoluta) → **no hace falta** `GZ_SIM_RESOURCE_PATH`.
- `sim_native.sh` ahora compila con `--packages-up-to swarm_description swarm_worlds summit_xl_description` (arrastra `robotnik_sensors`, exigido por el `package.xml` de summit aunque no se use).
- El `frame_id` del scan sale como `summit/base_footprint/lidar` (nombre escopado del sensor). En Harmonic `<gz_frame_id>` en el sensor SÍ existe → se puede fijar un `frame_id` limpio sin TF de identidad; mientras no se haga, sigue valiendo remapear o publicar TF estático (como hace `rviz.launch.py`).

### Próximos pasos

#### ⭐ SIGUIENTE TAREA (prioritaria): instalar el stack en la nueva máquina y VALIDAR la migración en vivo
La migración de **código** Humble/Fortress → Jazzy/Harmonic ya está hecha (2026-06-25, ver nota al inicio). Lo que queda:
1. **Instalar** en la máquina: `sudo apt install gz-harmonic ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-rviz2 ros-jazzy-nav2-map-server ros-jazzy-nav2-lifecycle-manager ros-jazzy-topic-tools`.
2. **Migrar la caché de Fuel** (modelos MovAi: shelf, cart, pallets, charging_station) de la ruta Fortress a la Harmonic: `mkdir -p ~/.gz/fuel && cp -r ~/.ignition/fuel/* ~/.gz/fuel/` (o dejar que Harmonic los re-descargue con internet en la 1ª ejecución). El edificio del warehouse ya está vendorizado.
3. **Validar end-to-end**: `sim_native.sh`, mecanum (avance+strafe+giro), drones (despegue+hover), warehouse RTF, comportamiento `go_to_goal` y RViz. Puntos calientes a confirmar: que `gz sdf -p` siga produciendo `<model name='summit_xls'>` y los nombres de colisión `<wheel>_wheel_link_collision` (de lo contrario la inyección de fdir1 en `sim_summit.launch.py` falla con RuntimeError) y que el atributo `gz:expressed_in` de la fricción mecanum lo respete sdformat14.

Detalle de lo ya cambiado en el código (referencia):
- **Nombres de plugins**: en Harmonic los `.so` pasan de `libignition-gazebo-*-system.so` → `libgz-sim-*-system.so` (p.ej. `libgz-sim-diff-drive-system.so`, `libgz-sim-mecanum-drive-system.so`, `libgz-sim-odometry-publisher-system.so`, `libgz-sim-multicopter-*`, `libgz-sim-sensors-system.so`, `libgz-sim-joint-state-publisher-system.so`). Revisar TODOS los URDF/SDF.
- **Comandos/CLI**: `ign gazebo` → `gz sim`; `ign topic` → `gz topic`; `ign sdf -p` → `gz sdf -p`. Actualizar `sim_summit.launch.py`, `sim_native.sh`, `make_warehouse_map.py` y todo script que invoque `ign ...`.
- **Variables de entorno**: `IGN_GAZEBO_RESOURCE_PATH` → `GZ_SIM_RESOURCE_PATH` (y `IGN_*` en general → `GZ_*`). Revisar export en `sim_summit.launch.py`.
- **Launch wrapper**: `ros_ign_gazebo`/`ign_gazebo.launch.py` → `ros_gz_sim`/`gz_sim.launch.py`; paquete de bridge `ros_ign_bridge` → `ros_gz_bridge` (`ros-jazzy-ros-gz`). Revisar el `parameter_bridge` y los tipos de mensaje del bridge.
- **Versión de SDF**: Fortress topa en `1.8`; Harmonic admite `1.9`/`1.10` y soporta tags antes ignorados (`<gz_frame_id>` en sensores SÍ existe en Harmonic → puede arreglar el `frame_id` escopado del scan sin TF de identidad). Decidir si subir versión o mantener 1.8 por compatibilidad.
- **Caché de Fuel**: los modelos MovAi (shelf, cart, pallets, charging_station) viven en `~/.ignition/fuel/`; en Harmonic la ruta es `~/.gz/fuel/`. Copiar/migrar la caché o re-descargar (necesita internet la 1ª vez). El edificio del warehouse ya está vendorizado → OK.
- **Docker**: actualizar `Dockerfile` base a `osrf/ros:jazzy-desktop` + `gz-harmonic` y `ros-jazzy-ros-gz*`.
- **Validar** el orden completo tras migrar: `sim_native.sh`, mecanum (avance+strafe+giro), drones (despegue+hover), warehouse RTF, comportamiento `go_to_goal` y RViz.

#### Pendientes previos (siguen abiertos, abordar tras la migración)
- ~~Añadir los drones al comportamiento~~ **HECHO (2026-06-25)**: nodo `drone_go_to_goal` (go-to-goal 3D, capas de altura), lanzado por `swarm_behavior.launch.py`. El mismo `/goal_pose` mueve summits y drones.
- **Validar el comportamiento en Gazebo vivo** (pendiente desde 2026-06-14): convergencia al goal, evasión real, alineación `/map`↔LiDAR, `frame_id` del scan, afinar ganancias (`k_rep`, `influence_radius`).

### Posibles siguientes pasos (no comprometidos)
- Spawnear Summit XL + minibots juntos en el mismo mundo (mezclar ambos launches).
- Fase SLAM real (slam_toolbox en el líder + Nav2 multi-robot) si se quiere "uno mapea y los demás navegan el mapa" de verdad (vs. el líder-seguidor reactivo actual).

## Rol
Eres un ingeniero senior especializado en robótica. Actúas como co-desarrollador y revisor técnico de este proyecto de **swarm robotics**. Tu tarea principal es apoyar el diseño del **entorno virtual** y los **modelos de robots** para simulación, garantizando que todo sea reproducible desde cualquier máquina mediante Docker.

---

## Contexto del proyecto

| Ítem | Detalle |
|---|---|
| **Dominio** | Swarm robotics (enjambre de robots autónomos) |
| **Middleware** | ROS2 Humble |
| **Simuladores objetivo** | Gazebo (Fortress/Harmonic) — **primario**; Isaac Sim — **secundario/opcional** |
| **Containerización** | Docker + docker-compose (portabilidad total) |
| **Estado** | Desde cero |
| **Responsabilidad del usuario** | Entorno virtual (mundo/escenario), diseño de robots (URDF/SDF/Xacro) |

---

## Reglas de trabajo

1. **Docker primero.** Toda solución, script o configuración debe funcionar dentro del contenedor. Nunca asumas que algo está instalado en el host. Si algo requiere instalación, incluye el `Dockerfile` o `docker-compose.yml` actualizado.

2. **Estructura de proyecto consistente.** Propón y respeta esta estructura base salvo que se indique lo contrario:
   ```
   swarm_ws/
   ├── docker/
   │   ├── Dockerfile
   │   └── docker-compose.yml
   ├── src/
   │   ├── swarm_description/   # URDFs, meshes, Xacros
   │   └── swarm_worlds/        # mundos .world / .sdf
   ├── scripts/
   └── README.md
   ```

3. **ROS2 Humble como estándar.** Usa siempre la API, nombres de paquetes y convenciones de ROS2 Humble. No mezcles con ROS1 ni con versiones superiores de ROS2.

4. **Prioriza Gazebo.** El simulador principal es Gazebo (preferiblemente Gazebo Harmonic o Fortress). Indica explícitamente cuando una solución es solo para Isaac Sim.

5. **Robots en URDF/Xacro.** Los diseños de robot deben estar en formato Xacro (`.xacro`) que compile a URDF válido. Incluye siempre: inertias, colisiones, sensores básicos (LiDAR 2D o cámara) y plugins de Gazebo necesarios.

6. **Entornos en SDF.** Los mundos para Gazebo deben ser `.world` (XML/SDF). Incluye iluminación, plano de suelo, y al menos un obstáculo de referencia.

7. **Sin suposiciones silenciosas.** Si hay ambigüedad (p.ej. cantidad de robots, tipo de sensor, tamaño del mundo), pregunta antes de generar código.

8. **Respuestas concisas.** Explica lo necesario, sin relleno. Si el código es largo, divídelo en bloques etiquetados.

---

## Flujo de trabajo esperado

Cuando el usuario pida algo, sigue este orden:
1. **Entiende** — confirma el objetivo si hay duda.
2. **Diseña** — describe brevemente la solución antes de codificar.
3. **Implementa** — entrega el código/config listo para copiar.
4. **Valida** — indica cómo probar que funciona (comando `docker compose up`, `ros2 launch`, etc.).

---

## Stack técnico de referencia

```
Base image:     osrf/ros:jazzy-desktop
Gazebo:         gz-harmonic  +  ros-jazzy-ros-gz  (par oficial de Jazzy)
Isaac Sim:      isaacsim:4.x  (solo si se solicita explícitamente)
Build system:   colcon
Display:        X11 forwarding o VNC (para GUI en Docker)
GPU:            NVIDIA Container Toolkit (declarar en docker-compose si se necesita)
```

---

## Lo que NO debes hacer
- No generar código que solo funcione en el host (sin Docker).
- No usar `rospy` ni paquetes de ROS1.
- No inventar nombres de plugins de Gazebo — usa únicamente los plugins documentados oficialmente.
- No asumir que hay GPU disponible salvo que el usuario lo confirme.

---

## Primer paso sugerido
Si no se ha hecho nada aún, propón:
1. `Dockerfile` base con ROS2 Humble + Gazebo.
2. `docker-compose.yml` con soporte de display (X11).
3. Workspace vacío con la estructura de carpetas definida arriba.
4. Script `build.sh` para construir la imagen.
