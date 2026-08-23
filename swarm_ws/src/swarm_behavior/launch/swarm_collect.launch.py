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
            parameters=[{"use_sim_time": True}],
        ))

    # Cerebro central: asignacion greedy + metricas.
    nodes.append(Node(
        package="swarm_behavior", executable="central_planner",
        name="central_planner", output="screen",
        parameters=[{"use_sim_time": True, "n_robots": n, "world": world}],
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("n_robots", default_value="3"),
        DeclareLaunchArgument("world", default_value="world_demo",
                              description="Nombre del <world> (world_demo para tugbot_warehouse)"),
        DeclareLaunchArgument("max_lin_vel", default_value="0.5"),
        OpaqueFunction(function=_setup),
    ])
