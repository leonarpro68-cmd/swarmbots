"""Lanza los controladores de comportamiento del enjambre (summits + drones).

Asume que la sim ya esta corriendo (sim_summit.launch.py): este launch solo
arranca los nodos de comportamiento, no Gazebo. Un mismo goal (la herramienta
"2D Goal Pose" de RViz -> /goal_pose) mueve a TODOS: los summits van por el
suelo esquivando obstaculos con el LiDAR; los drones vuelan al mismo punto a su
altitud de crucero (capas de altura por dron para no colisionar).

Modos (solo afectan a los summits):
  mode:=goal    -> los N summits rodean el punto (goal_x, goal_y) repartidos en
                   un ANILLO (un slot por robot, radio segun formation_spacing),
                   en vez de pelear por el mismo punto y quedarse oscilando.
  mode:=follow  -> summit0 va al punto; summit1..N-1 siguen a summit0 (formacion).
Los drones siempre van al goal (sobrevuelan el punto, se superponen) en ambos modos.

Ejemplos:
  ros2 launch swarm_behavior swarm_behavior.launch.py n_robots:=3 goal_x:=0 goal_y:=8
  ros2 launch swarm_behavior swarm_behavior.launch.py n_robots:=3 mode:=follow
  ros2 launch swarm_behavior swarm_behavior.launch.py n_robots:=3 n_drones:=0   # sin drones
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    n = int(LaunchConfiguration("n_robots").perform(context))
    n_drones = int(LaunchConfiguration("n_drones").perform(context))
    if n_drones < 0:
        n_drones = n  # -1 = igual que n_robots (mismo criterio que sim_summit)
    mode = LaunchConfiguration("mode").perform(context).lower()
    gx = float(LaunchConfiguration("goal_x").perform(context))
    gy = float(LaunchConfiguration("goal_y").perform(context))
    standoff = float(LaunchConfiguration("standoff").perform(context))
    formation_spacing = float(LaunchConfiguration("formation_spacing").perform(context))
    cruise_z = float(LaunchConfiguration("cruise_z").perform(context))
    layer = float(LaunchConfiguration("drone_layer").perform(context))
    leader = "summit0"

    nodes = []
    # --- Summits: go_to_goal con LiDAR (modo goal/follow) ---
    for i in range(n):
        ns = f"summit{i}"
        params = {
            "use_sim_time": True,
            "goal_x": gx,
            "goal_y": gy,
            "standoff": standoff,
            "follow_robot": "",
            # Formacion en anillo: cada summit a su slot alrededor del goal.
            "robot_index": i,
            "n_robots": n,
            "formation_spacing": formation_spacing,
        }
        # En modo follow, todos menos el lider persiguen al lider.
        if mode == "follow" and ns != leader:
            params["follow_robot"] = leader

        nodes.append(Node(
            package="swarm_behavior",
            executable="go_to_goal",
            namespace=ns,
            name="go_to_goal",
            output="screen",
            parameters=[params],
        ))

    # --- Drones: go_to_goal 3D, cada uno a una capa de altura distinta ---
    for i in range(n_drones):
        ns = f"drone{i}"
        nodes.append(Node(
            package="swarm_behavior",
            executable="drone_go_to_goal",
            namespace=ns,
            name="drone_go_to_goal",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "goal_x": gx,
                "goal_y": gy,
                "cruise_z": cruise_z + i * layer,
            }],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("n_robots", default_value="3"),
        DeclareLaunchArgument("n_drones", default_value="-1",
                              description="Nº de drones a controlar (-1 = igual que n_robots; 0 = ninguno)"),
        DeclareLaunchArgument("mode", default_value="goal", description="goal | follow (solo summits)"),
        DeclareLaunchArgument("goal_x", default_value="0.0"),
        DeclareLaunchArgument("goal_y", default_value="8.0"),
        DeclareLaunchArgument("standoff", default_value="1.5",
                              description="Distancia a mantener del lider (modo follow)"),
        DeclareLaunchArgument("formation_spacing", default_value="2.0",
                              description="Distancia entre summits vecinos en el anillo del goal (m); >influence_radius para no repelerse en reposo"),
        DeclareLaunchArgument("cruise_z", default_value="3.5",
                              description="Altitud de crucero del primer dron (m)"),
        DeclareLaunchArgument("drone_layer", default_value="1.0",
                              description="Separacion de altura entre drones (m) para no colisionar"),
        OpaqueFunction(function=_setup),
    ])
