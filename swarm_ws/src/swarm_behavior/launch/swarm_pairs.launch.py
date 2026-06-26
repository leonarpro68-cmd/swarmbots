"""N grupos, cada grupo con n pares {Summit + dron}, cada grupo a su meta aleatoria.

Dos parametros independientes:
  - n_groups        (N): nº de GRUPOS. Cada grupo va a su propia meta aleatoria.
  - pairs_per_group (n): nº de pares {Summit+dron} POR grupo. n=2 -> cada grupo
                         tiene 2 summits + 2 drones (4 robots).
Total de robots = n_groups * pairs_per_group  (summits y drones por separado).
La sim debe spawnear esa cantidad:  n_robots = n_groups * pairs_per_group.

Dentro de un grupo: los n summits forman un ANILLO alrededor de la meta del
grupo (no pelean por el punto) y los n drones la sobrevuelan en CAPAS de altura.
Entre grupos: se ceden el paso por PRIORIDAD (grupo de menor indice = preferente);
si las rutas se cruzan, el grupo que cede se aparta (ver yield en go_to_goal).
Solo ceden los summits (los drones van por capas, no colisionan).

Ejemplos:
    # 2 grupos de 1 par (4 robots): 2 summits + 2 drones
    ros2 launch swarm_worlds sim_summit.launch.py n_robots:=2
    ros2 launch swarm_behavior swarm_pairs.launch.py n_groups:=2 pairs_per_group:=1

    # 2 grupos de 2 pares (8 robots): 4 summits + 4 drones
    ros2 launch swarm_worlds sim_summit.launch.py n_robots:=4
    ros2 launch swarm_behavior swarm_pairs.launch.py n_groups:=2 pairs_per_group:=2
"""
import math
import random

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    N = int(LaunchConfiguration("n_groups").perform(context))
    n = int(LaunchConfiguration("pairs_per_group").perform(context))
    seed = int(LaunchConfiguration("seed").perform(context))
    if seed >= 0:
        random.seed(seed)
    xmin = float(LaunchConfiguration("area_xmin").perform(context))
    xmax = float(LaunchConfiguration("area_xmax").perform(context))
    ymin = float(LaunchConfiguration("area_ymin").perform(context))
    ymax = float(LaunchConfiguration("area_ymax").perform(context))
    min_sep = float(LaunchConfiguration("min_goal_sep").perform(context))
    spacing = float(LaunchConfiguration("formation_spacing").perform(context))
    cruise_z = float(LaunchConfiguration("cruise_z").perform(context))
    layer = float(LaunchConfiguration("drone_layer").perform(context))

    total = N * n
    print(f"[swarm_pairs] {N} grupos x {n} pares = {total} summits + {total} drones. "
          f"La sim debe tener n_robots:={total}.")

    # Una meta aleatoria distinta por GRUPO (separadas >= min_sep para que las
    # rutas de los grupos se crucen).
    goals = []
    for _ in range(N):
        for _try in range(200):
            gx = random.uniform(xmin, xmax)
            gy = random.uniform(ymin, ymax)
            if all(math.hypot(gx - ox, gy - oy) >= min_sep for ox, oy in goals):
                break
        goals.append((gx, gy))

    nodes = []
    for g, (gx, gy) in enumerate(goals):
        # Prioridad por grupo: el grupo g cede ante TODOS los summits de los
        # grupos 0..g-1 (indices globales 0 .. g*n-1).
        peers = ",".join(f"summit{k}" for k in range(g * n))
        for j in range(n):
            idx = g * n + j   # indice global del robot
            nodes.append(Node(
                package="swarm_behavior", executable="go_to_goal",
                namespace=f"summit{idx}", name="go_to_goal", output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "goal_x": gx, "goal_y": gy,
                    # Anillo dentro del grupo: n summits, este es el j-esimo.
                    "n_robots": n,
                    "robot_index": j,
                    "formation_spacing": spacing,
                    "use_goal_topic": False,
                    "yield_peers": peers,   # cede a los grupos prioritarios
                }],
            ))
            nodes.append(Node(
                package="swarm_behavior", executable="drone_go_to_goal",
                namespace=f"drone{idx}", name="drone_go_to_goal", output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "goal_x": gx, "goal_y": gy,
                    # Capa de altura por posicion dentro del grupo (acotada: no
                    # crece con N, asi no se sale del almacen).
                    "cruise_z": cruise_z + j * layer,
                    "use_goal_topic": False,
                }],
            ))
        print(f"[swarm_pairs] grupo {g}: meta ({gx:.2f}, {gy:.2f}), "
              f"summits {g*n}..{g*n+n-1}, cede ante {peers or '(nadie)'}")
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("n_groups", default_value="2",
                              description="N: nº de grupos (cada uno a su meta aleatoria)"),
        DeclareLaunchArgument("pairs_per_group", default_value="1",
                              description="n: pares {summit+dron} por grupo"),
        DeclareLaunchArgument("seed", default_value="-1",
                              description="Semilla del aleatorio (-1 = distinto cada vez)"),
        DeclareLaunchArgument("area_xmin", default_value="-6.0"),
        DeclareLaunchArgument("area_xmax", default_value="6.0"),
        DeclareLaunchArgument("area_ymin", default_value="11.0"),
        DeclareLaunchArgument("area_ymax", default_value="23.0"),
        DeclareLaunchArgument("min_goal_sep", default_value="6.0",
                              description="Separacion minima entre metas de grupos (m)"),
        DeclareLaunchArgument("formation_spacing", default_value="2.0",
                              description="Separacion entre summits del mismo grupo en su anillo (m)"),
        DeclareLaunchArgument("cruise_z", default_value="3.5"),
        DeclareLaunchArgument("drone_layer", default_value="1.0"),
        OpaqueFunction(function=_setup),
    ])
