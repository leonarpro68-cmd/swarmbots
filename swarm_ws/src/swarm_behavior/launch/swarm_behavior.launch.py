"""Lanza un controlador go_to_goal por cada Summit del enjambre.

Asume que la sim ya esta corriendo (sim_summit.launch.py): este launch solo
arranca los nodos de comportamiento, no Gazebo.

Modos:
  mode:=goal    -> los N summits van todos al mismo punto (goal_x, goal_y).
  mode:=follow  -> summit0 va al punto; summit1..N-1 siguen a summit0 (formacion).

Ejemplos:
  ros2 launch swarm_behavior swarm_behavior.launch.py n_robots:=3 goal_x:=0 goal_y:=8
  ros2 launch swarm_behavior swarm_behavior.launch.py n_robots:=3 mode:=follow goal_x:=0 goal_y:=8
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    n = int(LaunchConfiguration("n_robots").perform(context))
    mode = LaunchConfiguration("mode").perform(context).lower()
    gx = float(LaunchConfiguration("goal_x").perform(context))
    gy = float(LaunchConfiguration("goal_y").perform(context))
    standoff = float(LaunchConfiguration("standoff").perform(context))
    leader = "summit0"

    nodes = []
    for i in range(n):
        ns = f"summit{i}"
        params = {
            "use_sim_time": True,
            "goal_x": gx,
            "goal_y": gy,
            "standoff": standoff,
            "follow_robot": "",
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
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("n_robots", default_value="3"),
        DeclareLaunchArgument("mode", default_value="goal", description="goal | follow"),
        DeclareLaunchArgument("goal_x", default_value="0.0"),
        DeclareLaunchArgument("goal_y", default_value="8.0"),
        DeclareLaunchArgument("standoff", default_value="1.5",
                              description="Distancia a mantener del lider (modo follow)"),
        OpaqueFunction(function=_setup),
    ])
