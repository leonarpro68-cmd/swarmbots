"""Spawn N Summit XLS (mecanum/omni) + N drones X3 into Gazebo Harmonic.

Summits: el plugin MecanumDrive necesita fricción direccional en las ruedas
(<fdir1 gz:expressed_in=...>), que no es expresable en URDF. Por eso
aquí se hace: xacro → URDF → `gz sdf -p` → inyección de fdir1 → spawn del
SDF resultante. El URDF (sin fdir1) se sigue usando para robot_state_publisher.

Drones: al model.sdf vendorizado (x3_uav) se le inyectan los plugins nativos
MulticopterMotorModel (x4) + MulticopterVelocityControl + OdometryPublisher
(config copiada del demo multicopter_velocity_control.sdf de gz-sim8).
Despegan solos al arrancar y quedan en hover ~2.3 m sobre el anillo de
summits. El controlador no actúa hasta recibir el primer Twist, por eso
spawnean en el suelo y un proceso por dron manda subida y luego hover.

Usage:
    ros2 launch swarm_worlds sim_summit.launch.py                        # N Summit+gripper, sin drones (default)
    ros2 launch swarm_worlds sim_summit.launch.py n_robots:=1            # 1 Summit+gripper
    ros2 launch swarm_worlds sim_summit.launch.py n_drones:=3            # reactivar drones (aparcados por defecto)
    ros2 launch swarm_worlds sim_summit.launch.py headless:=true
"""
import math
import os
import random
import re
import shutil
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

GEN_DIR = "/tmp/swarm_summit"

# Direcciones de rodillo mecanum (patrón X, igual que el demo
# mecanum_drive.sdf de gz-sim8). FL/BR vs FR/BL alternados.
_FDIR = {
    "front_left":  "1 -1 0",
    "back_right":  "1 -1 0",
    "front_right": "1 1 0",
    "back_left":   "1 1 0",
}


def _generate_sdf(ns_str: str, xacro_path: str, gripper: bool = False,
                  trash_list: str = ""):
    """xacro → URDF → SDF con fdir1 inyectado. Devuelve (urdf_str, sdf_path).

    trash_list: nombres de los modelos de basura (separados por espacio) para
    generar un DetachableJoint por objeto en el gripper. Deben coincidir con
    los nombres de los modelos inyectados en el mundo.
    """
    os.makedirs(GEN_DIR, exist_ok=True)

    xacro_cmd = ["xacro", xacro_path, f"robot_ns:={ns_str}"]
    if gripper:
        xacro_cmd.append("gripper:=true")
        xacro_cmd.append(f"trash_list:={trash_list}")
    urdf_str = subprocess.run(
        xacro_cmd,
        check=True, capture_output=True, text=True,
    ).stdout
    urdf_path = os.path.join(GEN_DIR, f"{ns_str}.urdf")
    with open(urdf_path, "w") as f:
        f.write(urdf_str)

    sdf_str = subprocess.run(
        ["gz", "sdf", "-p", urdf_path],
        check=True, capture_output=True, text=True,
    ).stdout

    # Namespace XML para el atributo gz:expressed_in
    sdf_str = sdf_str.replace(
        "<model name='summit_xls'>",
        "<model name='summit_xls' xmlns:gz='http://gazebosim.org/schema'>",
        1,
    )

    # Inyectar fdir1 dentro del <ode> de la colisión de cada rueda
    for wheel, fdir in _FDIR.items():
        tag = f"<fdir1 gz:expressed_in='base_footprint'>{fdir}</fdir1>"
        pattern = re.compile(
            rf"(<collision name='{wheel}_wheel_link_collision'>.*?<ode>)(.*?)(</ode>)",
            re.S,
        )
        sdf_str, n_subs = pattern.subn(rf"\1\2{tag}\3", sdf_str)
        if n_subs != 1:
            raise RuntimeError(
                f"fdir1 injection failed for '{wheel}' ({n_subs} matches). "
                "¿Cambió el formato de salida de 'gz sdf -p'?"
            )

    sdf_path = os.path.join(GEN_DIR, f"{ns_str}.sdf")
    with open(sdf_path, "w") as f:
        f.write(sdf_str)
    return urdf_str, sdf_path


# ---- Basura aleatoria (semilla reproducible) ----------------------------
# PRISMAS 0.10 x 0.10 base x 0.20 ALTO, 0.3 kg. La pinza del Summit va a la
# altura del chasis (dedos en z ~= 0.10-0.16; centro del hueco z ~= 0.127
# porque base_link cuelga a wheel_radius=0.127); un cubo de 0.10 en el suelo
# solo llegaba a z=0.10 y la pinza pasaba por encima. Con 0.20 de alto la pieza
# llega al hueco de los dedos y se ve agarrada. El transporte es cinematico
# (grasp_manager teleporta la pieza flotando de pie), asi que la mayor relacion
# alto/ancho (2:1) no la hace volcar en marcha; el agarre por proximidad
# (approach_offset) no la toca al acercarse. 3 colores para variedad visual.
# Inercia caja HxWxD: Ixx=Iyy=(1/12)m(0.10^2+0.20^2), Izz=(1/12)m(0.10^2+0.10^2).
_TRASH_H = 0.20  # ALTO de la pieza (m); la pose z = _TRASH_H/2 (apoyado en suelo)
_TRASH_TYPES = [
    ("<box><size>0.10 0.10 0.20</size></box>",
     "<ixx>1.25e-3</ixx><iyy>1.25e-3</iyy><izz>5.0e-4</izz>", "0.2 0.6 0.3"),
    ("<box><size>0.10 0.10 0.20</size></box>",
     "<ixx>1.25e-3</ixx><iyy>1.25e-3</iyy><izz>5.0e-4</izz>", "0.3 0.4 0.7"),
    ("<box><size>0.10 0.10 0.20</size></box>",
     "<ixx>1.25e-3</ixx><iyy>1.25e-3</iyy><izz>5.0e-4</izz>", "0.7 0.4 0.2"),
]


# Caja de aparición por defecto (fallback; en runtime la calcula _launch_setup
# desde center_x/center_y y area_half). Warehouse vaciado => suelo abierto.
_SPAWN_BOX = (-8.0, 8.0, -8.0, 8.0)


def _random_poses(rng, n, avoid, box=_SPAWN_BOX, min_sep=0.5, avoid_clear=0.8):
    """n poses (x,y) por rejection sampling: separadas entre si (min_sep) y de
    las posiciones a evitar (avoid_clear). rng es un random.Random ya sembrado
    => reproducible. Devuelve [(x, y)] (puede ser <n si la zona se satura)."""
    xmin, xmax, ymin, ymax = box
    placed = []
    attempts = 0
    while len(placed) < n and attempts < n * 300 + 100:
        attempts += 1
        x = rng.uniform(xmin, xmax)
        y = rng.uniform(ymin, ymax)
        if any(math.hypot(x - ax, y - ay) < avoid_clear for ax, ay in avoid):
            continue
        if any(math.hypot(x - px, y - py) < min_sep for px, py in placed):
            continue
        placed.append((x, y))
    return placed


def _trash_poses(n, seed, avoid, box=_SPAWN_BOX):
    """[(name, x, y, type_idx)] de basuras. Mismo seed => mismas poses."""
    pts = _random_poses(random.Random(seed + 101), n, avoid, box=box)
    return [(f"trash_{i}", x, y, i % len(_TRASH_TYPES))
            for i, (x, y) in enumerate(pts)]


def _deposit_poses(n, seed, avoid, box=_SPAWN_BOX):
    """[(name, x, y)] de depositos. Stream de rng separado (seed+10007) pero
    reproducible con el mismo seed. Mas separados entre si (min_sep 2 m) y
    evitando robots + basuras (avoid)."""
    pts = _random_poses(random.Random(seed + 10007), n, avoid, box=box,
                        min_sep=2.0, avoid_clear=1.0)
    return [(f"deposit_{i}", x, y) for i, (x, y) in enumerate(pts)]


def _robot_specs_random(n, seed, box, avoid=(), min_sep=2.0):
    """[(ns, x, y, yaw)] de robots en poses y yaw aleatorios (sembrados).
    Stream de rng propio (seed+20011). Separados entre si (min_sep) y de
    'avoid'. yaw uniforme en [-pi, pi]."""
    rng = random.Random(seed + 20011)
    pts = _random_poses(rng, n, list(avoid), box=box,
                        min_sep=min_sep, avoid_clear=min_sep)
    return [(f"summit{i}", x, y, rng.uniform(-math.pi, math.pi))
            for i, (x, y) in enumerate(pts)]


def _deposit_model_sdf(name, x, y, radius=0.5):
    """SDF de un deposito: disco plano visual, ESTATICO y SIN colision (el
    robot lo atraviesa; solo marca la zona de descarga)."""
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0.01 0 0 0</pose>
      <link name="link">
        <visual name="v">
          <geometry><cylinder><radius>{radius}</radius><length>0.02</length></cylinder></geometry>
          <material><ambient>0.9 0.25 0.2 0.5</ambient><diffuse>0.9 0.25 0.2 0.5</diffuse></material>
        </visual>
      </link>
    </model>"""


def _trash_model_sdf(name, x, y, type_idx):
    """SDF de un modelo de basura dinamico (para inyectar en el <world>)."""
    geom, inertia, color = _TRASH_TYPES[type_idx]
    return f"""
    <model name="{name}">
      <pose>{x:.3f} {y:.3f} {_TRASH_H / 2:.3f} 0 0 0</pose>
      <link name="link">
        <inertial><mass>0.3</mass>
          <inertia>{inertia}<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="c"><geometry>{geom}</geometry>
          <surface><friction><ode><mu>0.4</mu><mu2>0.4</mu2></ode></friction></surface></collision>
        <visual name="v"><geometry>{geom}</geometry>
          <material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse></material></visual>
      </link>
    </model>"""


_MOTOR_PLUGIN = """
      <plugin filename="gz-sim-multicopter-motor-model-system"
              name="gz::sim::systems::MulticopterMotorModel">
        <robotNamespace>{ns}</robotNamespace>
        <jointName>X3/rotor_{n}_joint</jointName>
        <linkName>X3/rotor_{n}</linkName>
        <turningDirection>{dir}</turningDirection>
        <timeConstantUp>0.0125</timeConstantUp>
        <timeConstantDown>0.025</timeConstantDown>
        <maxRotVelocity>800.0</maxRotVelocity>
        <motorConstant>8.54858e-06</motorConstant>
        <momentConstant>0.016</momentConstant>
        <commandSubTopic>gazebo/command/motor_speed</commandSubTopic>
        <motorNumber>{n}</motorNumber>
        <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
        <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
        <motorSpeedPubTopic>motor_speed/{n}</motorSpeedPubTopic>
        <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
        <motorType>velocity</motorType>
      </plugin>"""

_ROTOR_CFG = """
          <rotor>
            <jointName>X3/rotor_{n}_joint</jointName>
            <forceConstant>8.54858e-06</forceConstant>
            <momentConstant>0.016</momentConstant>
            <direction>{d}</direction>
          </rotor>"""

_CONTROL_PLUGIN = """
      <plugin filename="gz-sim-multicopter-control-system"
              name="gz::sim::systems::MulticopterVelocityControl">
        <robotNamespace>{ns}</robotNamespace>
        <commandSubTopic>gazebo/command/twist</commandSubTopic>
        <enableSubTopic>enable</enableSubTopic>
        <comLinkName>X3/base_link</comLinkName>
        <velocityGain>2.7 2.7 2.7</velocityGain>
        <attitudeGain>2 3 0.15</attitudeGain>
        <angularRateGain>0.4 0.52 0.18</angularRateGain>
        <maximumLinearAcceleration>2 2 2</maximumLinearAcceleration>
        <rotorConfiguration>{rotors}
        </rotorConfiguration>
      </plugin>
      <plugin filename="gz-sim-odometry-publisher-system"
              name="gz::sim::systems::OdometryPublisher">
        <dimensions>3</dimensions>
        <odom_frame>{ns}/odom</odom_frame>
        <robot_base_frame>{ns}/base_footprint</robot_base_frame>
      </plugin>"""


def _generate_drone_sdf(ns_str: str) -> str:
    """model.sdf vendorizado + plugins de vuelo con namespace. Devuelve ruta."""
    os.makedirs(GEN_DIR, exist_ok=True)
    worlds_share = get_package_share_directory("swarm_worlds")
    model_sdf = os.path.join(worlds_share, "models", "x3_uav", "model.sdf")
    with open(model_sdf) as f:
        sdf_str = f.read()

    motors = "".join(
        _MOTOR_PLUGIN.format(ns=ns_str, n=n, dir=d)
        for n, d in [(0, "ccw"), (1, "ccw"), (2, "cw"), (3, "cw")]
    )
    rotors = "".join(
        _ROTOR_CFG.format(n=n, d=d) for n, d in [(0, 1), (1, 1), (2, -1), (3, -1)]
    )
    plugins = motors + _CONTROL_PLUGIN.format(ns=ns_str, rotors=rotors)

    # Los meshes del modelo vendorizado tienen URIs relativas; al copiar el
    # SDF a /tmp hay que volverlas absolutas.
    sdf_str = sdf_str.replace(
        "<uri>meshes/", f"<uri>{os.path.join(worlds_share, 'models', 'x3_uav', 'meshes')}/"
    )
    assert sdf_str.count("</model>") == 1
    sdf_str = sdf_str.replace("</model>", plugins + "\n    </model>")

    sdf_path = os.path.join(GEN_DIR, f"{ns_str}.sdf")
    with open(sdf_path, "w") as f:
        f.write(sdf_str)
    return sdf_path


def _spawn_for_drone(ns_str: str, x: float, y: float, yaw: float):
    sdf_path = _generate_drone_sdf(ns_str)
    twist_topic = f"/{ns_str}/gazebo/command/twist"
    # El controlador no hace nada hasta el primer Twist: esperar a que el
    # dron esté en la sim (su odometría aparece), despegar y quedar en hover.
    # La subida se corta por ALTITUD real (odom z >= 2.2 m), no por sleep de
    # reloj de pared: con mundos pesados el RTF puede ser << 1 y un sleep
    # fijo dejaría al dron a centímetros del suelo.
    odom_topic = f"/model/{ns_str}/odometry"
    takeoff = (
        f"for i in $(seq 1 60); do "
        f"  gz topic -l 2>/dev/null | grep -q '{odom_topic}' && break; sleep 1; "
        f"done; sleep 1; "
        f"gz topic -t '{twist_topic}' -m gz.msgs.Twist -p 'linear: {{z: 0.7}}'; "
        f"for i in $(seq 1 300); do "
        f"  z=$(gz topic -e -t '{odom_topic}' -n 1 2>/dev/null "
        f"      | awk '/position {{/{{f=1}} f&&/z:/{{print $2; exit}}'); "
        f"  case \"$z\" in ''|*[!0-9.eE+-]*) z=0;; esac; "
        f"  awk -v z=\"$z\" 'BEGIN{{exit !(z>=2.2)}}' && break; "
        f"  sleep 1; "
        f"done; "
        f"gz topic -t '{twist_topic}' -m gz.msgs.Twist -p 'linear: {{z: 0.0}}'"
    )
    return [
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", ns_str,
                "-file", sdf_path,
                "-x", str(x), "-y", str(y), "-z", "0.1",
                "-Y", str(yaw),
            ],
            output="screen",
        ),
        ExecuteProcess(cmd=["bash", "-c", takeoff], name=f"takeoff_{ns_str}", output="screen"),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            namespace=ns_str,
            name=f"bridge_{ns_str}",
            output="screen",
            arguments=[
                f"{twist_topic}@geometry_msgs/msg/Twist]gz.msgs.Twist",
                f"/model/{ns_str}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            ],
            remappings=[
                (twist_topic,                  "cmd_vel"),
                (f"/model/{ns_str}/odometry",  "odom"),
            ],
        ),
    ]


def _spawn_for_robot(ns_str: str, x: float, y: float, yaw: float, gripper: bool = False,
                     trash_list: str = "", trash_names=None):
    desc_share = get_package_share_directory("summit_xl_description")
    xacro_path = os.path.join(desc_share, "robots", "summit_xl_omni.urdf.xacro")
    urdf_str, sdf_path = _generate_sdf(ns_str, xacro_path, gripper, trash_list)

    bridge_args = [
        f"/model/{ns_str}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        f"/model/{ns_str}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        f"/model/{ns_str}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        f"/model/{ns_str}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        f"/model/{ns_str}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        # IMU del chasis
        f"/model/{ns_str}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        # Odometria de ENCODERS (dead-reckoning del MecanumDrive, frame del
        # spawn). Es la odom realista (deriva); /odom sigue siendo el
        # ground-truth del OdometryPublisher.
        f"/model/{ns_str}/mecanum_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
    ]
    bridge_remaps = [
        (f"/model/{ns_str}/cmd_vel",      "cmd_vel"),
        (f"/model/{ns_str}/odometry",     "odom"),
        (f"/model/{ns_str}/scan",         "scan"),
        (f"/model/{ns_str}/joint_states", "joint_states"),
        (f"/model/{ns_str}/tf",           "tf"),
        (f"/model/{ns_str}/imu",          "imu"),
        (f"/model/{ns_str}/mecanum_odom", "encoder_odom"),
    ]
    # La camara solo existe con la pinza (esta en el macro del gripper).
    if gripper:
        bridge_args.append(
            f"/model/{ns_str}/camera@sensor_msgs/msg/Image[gz.msgs.Image")
        bridge_remaps.append((f"/model/{ns_str}/camera", "camera"))

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=ns_str,
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": urdf_str,
                "frame_prefix": f"{ns_str}/",
                "use_sim_time": True,
            }],
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", ns_str,
                "-file", sdf_path,
                "-x", str(x), "-y", str(y), "-z", "0.15",
                "-Y", str(yaw),
            ],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            namespace=ns_str,
            name=f"bridge_{ns_str}",
            output="screen",
            arguments=bridge_args,
            remappings=bridge_remaps,
        ),
    ]

    # Pinza: spawner del gripper_controller contra el controller_manager
    # namespaceado por robot (/<ns>/controller_manager). El gz_ros2_control lo
    # arranca al cargar el modelo; el spawner reintenta hasta que esta listo.
    # Timeouts largos: con el controller_manager inicializando el hardware al
    # arrancar Gazebo, los defaults (5 s) expiran de forma intermitente y el
    # controlador queda cargado pero sin configurar/activar (dedos sin
    # responder). --controller-manager-timeout espera al CM; --switch-timeout
    # da margen a la activacion.
    if gripper:
        nodes.append(Node(
            package="controller_manager",
            executable="spawner",
            name=f"spawn_gripper_{ns_str}",
            arguments=["gripper_controller",
                       "--controller-manager", f"/{ns_str}/controller_manager",
                       "--controller-manager-timeout", "60",
                       "--switch-timeout", "30"],
            output="screen",
        ))

        # Detach inicial de la basura: los DetachableJoint del gripper NACEN
        # ADJUNTADOS (limitacion de gz-sim), asi que la basura se mueve pegada
        # al robot desde el spawn. Publicamos 'detach' a todos los objetos
        # (varias rondas, por si gz-transport pierde la 1a en el descubrimiento)
        # para soltarlos; el grasp_manager luego los re-adjunta al agarrar.
        if trash_names:
            detach_cmds = " ; ".join(
                f"gz topic -t /{ns_str}/grasp/{obj}/detach -m gz.msgs.Empty -p ''"
                for obj in trash_names)
            nodes.append(ExecuteProcess(
                cmd=["bash", "-c",
                     f"sleep 6; for i in 1 2 3; do {detach_cmds} ; sleep 1.5; done"],
                name=f"detach_trash_{ns_str}",
                output="screen",
            ))

    return nodes


_NVIDIA_EGL_JSON = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"


def _force_nvidia_render():
    """Forzar render por la NVIDIA (Optimus). No-op si no hay NVIDIA o si el
    usuario ya fijo las variables (para no pisar una config manual)."""
    if shutil.which("nvidia-smi") is None:
        return
    # GLX (viewport del GUI) -> offload a la NVIDIA.
    os.environ.setdefault("__NV_PRIME_RENDER_OFFLOAD", "1")
    os.environ.setdefault("__GLX_VENDOR_LIBRARY_NAME", "nvidia")
    # EGL (render de sensores del servidor) -> usar SOLO el vendor NVIDIA, sin
    # intentar Mesa/dri2 sobre la PCI de la NVIDIA.
    if os.path.exists(_NVIDIA_EGL_JSON):
        os.environ.setdefault("__EGL_VENDOR_LIBRARY_FILENAMES", _NVIDIA_EGL_JSON)


def _launch_setup(context, *args, **kwargs):
    n_robots = int(LaunchConfiguration("n_robots").perform(context))
    headless = LaunchConfiguration("headless").perform(context).lower() in ("true", "1")
    world = LaunchConfiguration("world").perform(context)
    cx = float(LaunchConfiguration("center_x").perform(context))
    cy = float(LaunchConfiguration("center_y").perform(context))
    gripper = LaunchConfiguration("gripper").perform(context).lower() in ("true", "1")
    n_trash = int(LaunchConfiguration("n_trash").perform(context))
    n_deposits = int(LaunchConfiguration("n_deposits").perform(context))
    area_half = float(LaunchConfiguration("area_half").perform(context))
    seed = int(LaunchConfiguration("seed").perform(context))
    if seed < 0:
        seed = random.randrange(1 << 30)

    worlds_share = get_package_share_directory("swarm_worlds")
    world_path = os.path.join(worlds_share, "worlds", f"{world}.sdf")

    # Los mundos referencian modelos vendorizados vía model:// (p.ej. el
    # warehouse). Exportar antes de lanzar Gazebo; hereda a todos los hijos.
    models_dir = os.path.join(worlds_share, "models")
    prev = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if models_dir not in prev.split(":"):
        os.environ["GZ_SIM_RESOURCE_PATH"] = f"{models_dir}:{prev}" if prev else models_dir

    # Área de aparición: caja cuadrada centrada en (center_x, center_y) de
    # semilado area_half. Con el warehouse VACIADO (sin estanterías/carros/
    # pallets, solo edificio + paredes), todo el suelo interior está libre, así
    # que robots/basura/depósitos pueden colocarse al azar sin riesgo de
    # interpenetrar una colisión estática (lo que hundía el RTF a ~0.06). El
    # margen a las paredes (±15 x, ±25 y) lo da area_half < 15.
    box = (cx - area_half, cx + area_half, cy - area_half, cy + area_half)

    # Robots: posiciones Y yaw aleatorios (sembrados), separados entre sí. Se
    # calculan ANTES de basura/depósitos para que éstos los eviten.
    robot_specs = _robot_specs_random(n_robots, seed, box, min_sep=2.0)
    if len(robot_specs) < n_robots:
        print(f"[sim_summit] AVISO: solo se colocaron {len(robot_specs)}/{n_robots} "
              f"robots (zona saturada). Sube area_half o baja n_robots.")

    # Basura aleatoria (semilla reproducible) en la misma caja, evitando los
    # spawns de los robots. Se INYECTA en el <world> (no se spawnea suelta)
    # porque los DetachableJoint del gripper se enlazan a los modelos de basura
    # al CARGAR el robot: la basura debe existir antes de que el robot spawnee
    # (si no, el joint no se forma y no se puede agarrar).
    avoid = [(x, y) for (_, x, y, _) in robot_specs]
    trash = _trash_poses(n_trash, seed, avoid, box) if n_trash > 0 else []
    trash_names = [t[0] for t in trash]
    trash_list = " ".join(trash_names)
    if len(trash) < n_trash:
        print(f"[sim_summit] AVISO: solo se colocaron {len(trash)}/{n_trash} "
              f"basuras (zona saturada). Sube el área o baja n_trash/min_sep.")

    # Depositos aleatorios: discos planos atravesables (visual, sin colisión),
    # misma semilla y zona, evitando robots Y basuras (así el robot tiene que
    # transportar la basura hasta ellos). Stream de rng propio (reproducible).
    avoid_dep = avoid + [(x, y) for (_, x, y, _) in trash]
    deposits = _deposit_poses(n_deposits, seed, avoid_dep, box) if n_deposits > 0 else []
    if len(deposits) < n_deposits:
        print(f"[sim_summit] AVISO: solo se colocaron {len(deposits)}/{n_deposits} "
              f"depósitos (zona saturada). Sube el área o baja n_deposits.")
    if deposits:
        print("[sim_summit] depósitos: " +
              ", ".join(f"{n}=({x:.1f},{y:.1f})" for n, x, y in deposits))

    # Inyectar basura + depósitos en el <world> (no se spawnean sueltos):
    # los DetachableJoint del gripper se enlazan a los modelos de basura al
    # CARGAR el robot, así que la basura debe existir antes de spawnear el
    # robot (si no, el joint no se forma y no se puede agarrar). Los depósitos
    # van igual por consistencia.
    injection = "".join(
        [_trash_model_sdf(*t) for t in trash]
        + [_deposit_model_sdf(*d) for d in deposits]
    )
    if injection:
        with open(world_path) as f:
            world_xml = f.read()
        head, sep, tail = world_xml.rpartition("</world>")
        world_xml = head + injection + "\n  " + sep + tail
        os.makedirs(GEN_DIR, exist_ok=True)
        world_path = os.path.join(GEN_DIR, f"{world}_gen.sdf")
        with open(world_path, "w") as f:
            f.write(world_xml)

    # En portatiles Optimus (p.ej. RTX 4060 Laptop) con X en una GPU integrada,
    # glvnd intenta el vendor Mesa (dri2) para la PCI de la NVIDIA y falla
    # ("failed to create dri2 screen", driver null) -> el render del servidor
    # (gpu_lidar) y el viewport del GUI caen a software y el RTF se hunde a
    # ~0.05. Forzar el vendor NVIDIA para EGL y el offload NVIDIA para GLX lo
    # arregla. Solo si hay NVIDIA y el usuario no lo fijo ya (no rompe headless
    # ni maquinas sin NVIDIA).
    _force_nvidia_render()
    # En headless, --headless-rendering usa la GPU vía EGL (sin X). Sin ello
    # los gpu_lidar caen a render por software y el RTF se hunde (~0.05 en
    # el warehouse).
    gz_args = f"-r -v 4 {world_path}" + (" -s --headless-rendering" if headless else "")

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
            ),
            launch_arguments={"gz_args": gz_args}.items(),
        ),
    ]

    for ns, x, y, yaw in robot_specs:
        actions += _spawn_for_robot(ns, x, y, yaw, gripper, trash_list, trash_names)

    # Drones (aparcados por defecto, n_drones=0): spawnean en el suelo en poses
    # aleatorias de la misma caja, evitando robots/basura/depósitos. La
    # secuencia de despegue los deja en hover. Stream de rng propio.
    n_drones = int(LaunchConfiguration("n_drones").perform(context))
    if n_drones < 0:
        n_drones = n_robots
    if n_drones > 0:
        avoid_drone = avoid + [(x, y) for (_, x, y, _) in trash] \
            + [(x, y) for (_, x, y) in deposits]
        drone_pts = _random_poses(random.Random(seed + 30013), n_drones,
                                  avoid_drone, box=box, min_sep=1.5, avoid_clear=1.0)
        for i, (x, y) in enumerate(drone_pts):
            actions += _spawn_for_drone(f"drone{i}", x, y, 0.0)

    return actions


def generate_launch_description():
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    # Camara cenital del mundo (vista superior de las posiciones de los robots).
    overhead_cam_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="overhead_cam_bridge",
        output="screen",
        arguments=["/overhead/image@sensor_msgs/msg/Image[gz.msgs.Image"],
    )

    return LaunchDescription([
        DeclareLaunchArgument("n_robots", default_value="3", description="Number of Summit XLS to spawn"),
        DeclareLaunchArgument("n_drones", default_value="0", description="Number of X3 drones (0 = ninguno, -1 = same as n_robots)"),
        DeclareLaunchArgument("world", default_value="tugbot_warehouse", description="World file (sin .sdf) en swarm_worlds/worlds/"),
        DeclareLaunchArgument("center_x", default_value="0.0", description="Centro X del área de aparición aleatoria"),
        DeclareLaunchArgument("center_y", default_value="0.0", description="Centro Y del área de aparición (0 = centro del almacén vaciado, bajo la cámara cenital)"),
        DeclareLaunchArgument("headless", default_value="false", description="Run Gazebo server only (no GUI)"),
        DeclareLaunchArgument("gripper", default_value="true", description="Anadir pinza + camara + controladores gz_ros2_control (default: enjambre Summit+gripper, sin drones)"),
        DeclareLaunchArgument("n_trash", default_value="6", description="Nº de basuras (poses aleatorias sembradas, inyectadas en el mundo)"),
        DeclareLaunchArgument("n_deposits", default_value="2", description="Nº de depósitos (discos planos atravesables, poses aleatorias sembradas)"),
        DeclareLaunchArgument("area_half", default_value="8.0", description="Semilado (m) de la caja cuadrada de aparición centrada en (center_x, center_y). <15 para dejar margen a las paredes"),
        DeclareLaunchArgument("seed", default_value="42", description="Semilla de las poses aleatorias (reproducible; -1 = distinta cada vez)"),
        clock_bridge,
        overhead_cam_bridge,
        OpaqueFunction(function=_launch_setup),
    ])
