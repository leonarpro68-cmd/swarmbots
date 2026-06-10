"""Spawn N Summit XLS (mecanum/omni) + N drones X3 into Gazebo Fortress.

Summits: el plugin MecanumDrive necesita fricción direccional en las ruedas
(<fdir1 ignition:expressed_in=...>), que no es expresable en URDF. Por eso
aquí se hace: xacro → URDF → `ign sdf -p` → inyección de fdir1 → spawn del
SDF resultante. El URDF (sin fdir1) se sigue usando para robot_state_publisher.

Drones: al model.sdf vendorizado (x3_uav) se le inyectan los plugins nativos
MulticopterMotorModel (x4) + MulticopterVelocityControl + OdometryPublisher
(config copiada del demo multicopter_velocity_control.sdf de ign-gazebo6).
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
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

GEN_DIR = "/tmp/swarm_summit"

# Direcciones de rodillo mecanum (patrón X, igual que el demo
# mecanum_drive.sdf de ign-gazebo6). FL/BR vs FR/BL alternados.
_FDIR = {
    "front_left":  "1 -1 0",
    "back_right":  "1 -1 0",
    "front_right": "1 1 0",
    "back_left":   "1 1 0",
}


def _generate_sdf(ns_str: str, xacro_path: str):
    """xacro → URDF → SDF con fdir1 inyectado. Devuelve (urdf_str, sdf_path)."""
    os.makedirs(GEN_DIR, exist_ok=True)

    urdf_str = subprocess.run(
        ["xacro", xacro_path, f"robot_ns:={ns_str}"],
        check=True, capture_output=True, text=True,
    ).stdout
    urdf_path = os.path.join(GEN_DIR, f"{ns_str}.urdf")
    with open(urdf_path, "w") as f:
        f.write(urdf_str)

    sdf_str = subprocess.run(
        ["ign", "sdf", "-p", urdf_path],
        check=True, capture_output=True, text=True,
    ).stdout

    # Namespace XML para el atributo ignition:expressed_in
    sdf_str = sdf_str.replace(
        "<model name='summit_xls'>",
        "<model name='summit_xls' xmlns:ignition='http://ignitionrobotics.org/schema'>",
        1,
    )

    # Inyectar fdir1 dentro del <ode> de la colisión de cada rueda
    for wheel, fdir in _FDIR.items():
        tag = f"<fdir1 ignition:expressed_in='base_footprint'>{fdir}</fdir1>"
        pattern = re.compile(
            rf"(<collision name='{wheel}_wheel_link_collision'>.*?<ode>)(.*?)(</ode>)",
            re.S,
        )
        sdf_str, n_subs = pattern.subn(rf"\1\2{tag}\3", sdf_str)
        if n_subs != 1:
            raise RuntimeError(
                f"fdir1 injection failed for '{wheel}' ({n_subs} matches). "
                "¿Cambió el formato de salida de 'ign sdf -p'?"
            )

    sdf_path = os.path.join(GEN_DIR, f"{ns_str}.sdf")
    with open(sdf_path, "w") as f:
        f.write(sdf_str)
    return urdf_str, sdf_path


_MOTOR_PLUGIN = """
      <plugin filename="ignition-gazebo-multicopter-motor-model-system"
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
      <plugin filename="ignition-gazebo-multicopter-control-system"
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
      <plugin filename="ignition-gazebo-odometry-publisher-system"
              name="ignition::gazebo::systems::OdometryPublisher">
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
        f"  ign topic -l 2>/dev/null | grep -q '{odom_topic}' && break; sleep 1; "
        f"done; sleep 1; "
        f"ign topic -t '{twist_topic}' -m ignition.msgs.Twist -p 'linear: {{z: 0.7}}'; "
        f"for i in $(seq 1 300); do "
        f"  z=$(ign topic -e -t '{odom_topic}' -n 1 2>/dev/null "
        f"      | awk '/position {{/{{f=1}} f&&/z:/{{print $2; exit}}'); "
        f"  case \"$z\" in ''|*[!0-9.eE+-]*) z=0;; esac; "
        f"  awk -v z=\"$z\" 'BEGIN{{exit !(z>=2.2)}}' && break; "
        f"  sleep 1; "
        f"done; "
        f"ign topic -t '{twist_topic}' -m ignition.msgs.Twist -p 'linear: {{z: 0.0}}'"
    )
    return [
        Node(
            package="ros_ign_gazebo",
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
            package="ros_ign_bridge",
            executable="parameter_bridge",
            namespace=ns_str,
            name=f"bridge_{ns_str}",
            output="screen",
            arguments=[
                f"{twist_topic}@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                f"/model/{ns_str}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            ],
            remappings=[
                (twist_topic,                  "cmd_vel"),
                (f"/model/{ns_str}/odometry",  "odom"),
            ],
        ),
    ]


def _spawn_for_robot(ns_str: str, x: float, y: float, yaw: float):
    desc_share = get_package_share_directory("summit_xl_description")
    xacro_path = os.path.join(desc_share, "robots", "summit_xl_omni.urdf.xacro")
    urdf_str, sdf_path = _generate_sdf(ns_str, xacro_path)

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
            package="ros_ign_gazebo",
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
            package="ros_ign_bridge",
            executable="parameter_bridge",
            namespace=ns_str,
            name=f"bridge_{ns_str}",
            output="screen",
            arguments=[
                f"/model/{ns_str}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                f"/model/{ns_str}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                f"/model/{ns_str}/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                f"/model/{ns_str}/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
                f"/model/{ns_str}/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            ],
            remappings=[
                (f"/model/{ns_str}/cmd_vel",      "cmd_vel"),
                (f"/model/{ns_str}/odometry",     "odom"),
                (f"/model/{ns_str}/scan",         "scan"),
                (f"/model/{ns_str}/joint_states", "joint_states"),
                (f"/model/{ns_str}/tf",           "tf"),
            ],
        ),
    ]
    return nodes


def _launch_setup(context, *args, **kwargs):
    n_robots = int(LaunchConfiguration("n_robots").perform(context))
    headless = LaunchConfiguration("headless").perform(context).lower() in ("true", "1")
    world = LaunchConfiguration("world").perform(context)
    cx = float(LaunchConfiguration("center_x").perform(context))
    cy = float(LaunchConfiguration("center_y").perform(context))

    worlds_share = get_package_share_directory("swarm_worlds")
    world_path = os.path.join(worlds_share, "worlds", f"{world}.sdf")

    # Los mundos referencian modelos vendorizados vía model:// (p.ej. el
    # warehouse). Exportar antes de lanzar Gazebo; hereda a todos los hijos.
    models_dir = os.path.join(worlds_share, "models")
    prev = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    if models_dir not in prev.split(":"):
        os.environ["IGN_GAZEBO_RESOURCE_PATH"] = f"{models_dir}:{prev}" if prev else models_dir
    # En headless, --headless-rendering usa la GPU vía EGL (sin X). Sin ello
    # los gpu_lidar caen a render por software y el RTF se hunde (~0.05 en
    # el warehouse).
    ign_args = f"-r -v 4 {world_path}" + (" -s --headless-rendering" if headless else "")

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("ros_ign_gazebo"), "launch", "ign_gazebo.launch.py")
            ),
            launch_arguments={"ign_args": ign_args}.items(),
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
        actions += _spawn_for_robot(f"summit{i}", x, y, yaw)

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
        package="ros_ign_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
    )

    return LaunchDescription([
        DeclareLaunchArgument("n_robots", default_value="3", description="Number of Summit XLS to spawn"),
        DeclareLaunchArgument("n_drones", default_value="-1", description="Number of X3 drones (-1 = same as n_robots)"),
        DeclareLaunchArgument("world", default_value="tugbot_warehouse", description="World file (sin .sdf) en swarm_worlds/worlds/"),
        DeclareLaunchArgument("center_x", default_value="0.0", description="Spawn circle center X"),
        DeclareLaunchArgument("center_y", default_value="18.0", description="Spawn circle center Y"),
        DeclareLaunchArgument("headless", default_value="false", description="Run Gazebo server only (no GUI)"),
        clock_bridge,
        OpaqueFunction(function=_launch_setup),
    ])
