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
    ros2 launch swarm_worlds sim_summit.launch.py n_robots:=3            # 3+3
    ros2 launch swarm_worlds sim_summit.launch.py n_robots:=3 n_drones:=1
    ros2 launch swarm_worlds sim_summit.launch.py n_robots:=3 headless:=true
"""
import math
import os
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


def _generate_sdf(ns_str: str, xacro_path: str, gripper: bool = False):
    """xacro → URDF → SDF con fdir1 inyectado. Devuelve (urdf_str, sdf_path)."""
    os.makedirs(GEN_DIR, exist_ok=True)

    xacro_cmd = ["xacro", xacro_path, f"robot_ns:={ns_str}"]
    if gripper:
        xacro_cmd.append("gripper:=true")
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


def _spawn_for_robot(ns_str: str, x: float, y: float, yaw: float, gripper: bool = False):
    desc_share = get_package_share_directory("summit_xl_description")
    xacro_path = os.path.join(desc_share, "robots", "summit_xl_omni.urdf.xacro")
    urdf_str, sdf_path = _generate_sdf(ns_str, xacro_path, gripper)

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
    if gripper:
        nodes.append(Node(
            package="controller_manager",
            executable="spawner",
            name=f"spawn_gripper_{ns_str}",
            arguments=["gripper_controller",
                       "--controller-manager", f"/{ns_str}/controller_manager"],
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

    worlds_share = get_package_share_directory("swarm_worlds")
    world_path = os.path.join(worlds_share, "worlds", f"{world}.sdf")

    # Los mundos referencian modelos vendorizados vía model:// (p.ej. el
    # warehouse). Exportar antes de lanzar Gazebo; hereda a todos los hijos.
    models_dir = os.path.join(worlds_share, "models")
    prev = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if models_dir not in prev.split(":"):
        os.environ["GZ_SIM_RESOURCE_PATH"] = f"{models_dir}:{prev}" if prev else models_dir

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

    # Círculo alrededor de (center_x, center_y) con separación mínima de
    # ~1.5 m entre vecinos. El centro por defecto (0, 18) es la zona norte
    # abierta del tugbot_warehouse (≥5.5 m a cart, pallets, estanterías y
    # pared norte). OJO: las cajas de colisión de las shelf_big miden
    # 2.1x18x6 m (pasillos enteros, p.ej. shelf_big_3 cubre x∈[-1,1.1],
    # y∈[-22,-4]); spawnear un robot dentro de una de ellas interpenetra el
    # contacto y hunde el RTF de la sim entera a ~0.06.
    radius = 3.5
    if n_robots > 2:
        radius = max(3.5, 1.5 / (2.0 * math.sin(math.pi / n_robots)))
    for i in range(n_robots):
        ang = 2.0 * math.pi * i / max(n_robots, 1)
        x = cx + radius * math.cos(ang)
        y = cy + radius * math.sin(ang)
        yaw = ang + math.pi  # mirando al centro
        actions += _spawn_for_robot(f"summit{i}", x, y, yaw, gripper)

    # Drones: spawnean en el suelo, 1 m por fuera del anillo de summits
    # (despegar encima de uno acabaría aterrizándole en el techo), y la
    # secuencia de despegue los deja en hover sobrevolando el enjambre.
    n_drones = int(LaunchConfiguration("n_drones").perform(context))
    if n_drones < 0:
        n_drones = n_robots
    drone_radius = radius + 1.0
    for i in range(n_drones):
        ang = 2.0 * math.pi * i / max(n_drones, 1)
        x = cx + drone_radius * math.cos(ang)
        y = cy + drone_radius * math.sin(ang)
        yaw = ang + math.pi
        actions += _spawn_for_drone(f"drone{i}", x, y, yaw)

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
        DeclareLaunchArgument("n_drones", default_value="-1", description="Number of X3 drones (-1 = same as n_robots)"),
        DeclareLaunchArgument("world", default_value="tugbot_warehouse", description="World file (sin .sdf) en swarm_worlds/worlds/"),
        DeclareLaunchArgument("center_x", default_value="0.0", description="Spawn circle center X"),
        DeclareLaunchArgument("center_y", default_value="18.0", description="Spawn circle center Y"),
        DeclareLaunchArgument("headless", default_value="false", description="Run Gazebo server only (no GUI)"),
        DeclareLaunchArgument("gripper", default_value="false", description="Anadir pinza + camara + controladores gz_ros2_control (probar con n_robots:=1)"),
        clock_bridge,
        overhead_cam_bridge,
        OpaqueFunction(function=_launch_setup),
    ])
