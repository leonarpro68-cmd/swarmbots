"""Ciclo completo de recogida de basura para cada robot (Fase A+B).

Asume la sim ya corriendo (sim_summit.launch.py con gripper). Arranca:
  - central_planner (1): asignacion greedy + metricas (makespan, colisiones,
    piezas). Publica una meta por robot en /summitN/goal_pose y dispara
    grasp/release por proximidad.
  - go_to_goal (1 por robot, en su namespace): navega a /summitN/goal_pose
    (n_robots:=1 -> sin anillo, va al punto exacto), esquivando con el LiDAR.
  - grasp_manager (1 por robot): agarre/suelta real con DetachableJoint.

Cada robot hace: Ir -> agarrar (por proximidad) -> transportar -> depositar ->
siguiente basura, hasta vaciar el mundo.

  ros2 launch swarm_behavior swarm_collect.launch.py n_robots:=3
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    n = int(LaunchConfiguration("n_robots").perform(context))
    world = LaunchConfiguration("world").perform(context)
    max_lin = float(LaunchConfiguration("max_lin_vel").perform(context))
    # Perillas de velocidad (defaults = los del nodo => sin pasarlas, el
    # comportamiento es identico). Los nodos leen sus parametros UNA sola vez al
    # arrancar, asi que afinar en caliente con `ros2 param set` no surte efecto:
    # hay que relanzar pasando estos args.
    yaw_damp = float(LaunchConfiguration("yaw_damp").perform(context))
    k_rep = float(LaunchConfiguration("k_rep").perform(context))
    far_forward = float(LaunchConfiguration("far_forward").perform(context))
    slow_depth = float(LaunchConfiguration("slow_depth").perform(context))
    min_forward = float(LaunchConfiguration("min_forward").perform(context))
    align_timeout = float(LaunchConfiguration("align_timeout").perform(context))
    deposit_turns = LaunchConfiguration("deposit_turns").perform(context).lower() \
        in ("1", "true", "yes")
    queue_radius = float(LaunchConfiguration("queue_radius").perform(context))

    nodes = []
    for i in range(n):
        ns = f"summit{i}"
        # Navegacion: meta individual desde el planner. n_robots=1 -> el offset
        # de anillo es 0, va al punto EXACTO (no a un slot de formacion).
        nodes.append(Node(
            package="swarm_behavior", executable="go_to_goal",
            namespace=ns, name="go_to_goal", output="screen",
            parameters=[{
                "use_sim_time": True,
                "n_robots": 1,
                "robot_index": 0,
                "use_goal_topic": True,
                "goal_topic": f"/{ns}/goal_pose",
                "max_lin_vel": max_lin,
                "goal_tol": 0.3,
                "yaw_damp": yaw_damp,
                "k_rep": k_rep,
            }],
        ))
        # Agarre real (DetachableJoint) por robot.
        nodes.append(Node(
            package="swarm_behavior", executable="grasp_manager",
            name=f"grasp_manager_{ns}", output="screen",
            parameters=[{"use_sim_time": True, "robot": ns, "world": world}],
        ))
        # Aproximacion final RGB-D. Solo propone comandos; go_to_goal conserva
        # la autoria unica de cmd_vel y aplica la parada de seguridad LiDAR.
        nodes.append(Node(
            package="swarm_behavior", executable="visual_grasp",
            namespace=ns, name="visual_grasp", output="screen",
            parameters=[{
                "use_sim_time": True,
                "far_forward": far_forward,
                "slow_depth": slow_depth,
                "min_forward": min_forward,
                "timeout": align_timeout,
            }],
        ))

    # Cerebro central: asignacion greedy + metricas.
    nodes.append(Node(
        package="swarm_behavior", executable="central_planner",
        name="central_planner", output="screen",
        parameters=[{
            "use_sim_time": True, "n_robots": n, "world": world,
            "deposit_turns": deposit_turns,
            "queue_radius": queue_radius,
        }],
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("n_robots", default_value="3"),
        DeclareLaunchArgument("world", default_value="world_demo",
                              description="Nombre del <world> (world_demo para tugbot_warehouse)"),
        DeclareLaunchArgument("max_lin_vel", default_value="0.5"),
        # --- Esquive (go_to_goal) ---
        DeclareLaunchArgument("yaw_damp", default_value="1.0",
                              description="1.0 = no gira mientras esquiva (strafe puro); 0.0 = giro de siempre"),
        DeclareLaunchArgument("k_rep", default_value="0.30",
                              description="Ganancia de repulsion LiDAR; subirla desvia antes de los obstaculos"),
        # --- Aproximacion RGB-D al agarre (visual_grasp) ---
        DeclareLaunchArgument("far_forward", default_value="0.30",
                              description="m/s del tramo lejano (antes de slow_depth)"),
        DeclareLaunchArgument("slow_depth", default_value="0.18",
                              description="m: por debajo empieza el tramo lento de insercion"),
        DeclareLaunchArgument("min_forward", default_value="0.03",
                              description="m/s: suelo del tramo de insercion; 0.0 = comportamiento anterior"),
        DeclareLaunchArgument("align_timeout", default_value="15.0",
                              description="s para completar la alineacion RGB-D"),
        # --- Turno de descarga (central_planner) ---
        DeclareLaunchArgument("deposit_turns", default_value="true",
                              description="Un robot a la vez por deposito; false = comportamiento anterior"),
        DeclareLaunchArgument("queue_radius", default_value="2.5",
                              description="m del deposito donde espera el robot sin turno"),
        OpaqueFunction(function=_setup),
    ])
