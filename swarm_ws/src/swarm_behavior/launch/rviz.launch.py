"""Abre RViz para fijar el punto destino del enjambre y ver los robots/laseres.

Usa la herramienta "2D Goal Pose" de RViz: al clicar publica en /goal_pose y
todos los nodos go_to_goal reubican su objetivo en vivo.

TF: la odom de cada Summit ya esta en frame MUNDO (el OdometryPublisher la
inicializa en la pose mundial del spawn), asi que todos los `summitN/odom`
coinciden con un unico frame `map` -> se publica un TF estatico identidad
map->summitN/odom por robot. Ademas el `/tf` de cada robot esta namespaced
(`/summitN/tf`), asi que se relA al `/tf` global para que RViz vea el arbol.

Asume que la sim ya corre (sim_summit.launch.py).

  ros2 launch swarm_behavior rviz.launch.py n_robots:=3
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_SCAN_COLORS = [
    (255, 80, 80), (80, 255, 80), (80, 160, 255),
    (255, 220, 0), (220, 0, 255), (0, 255, 220),
]


def _make_rviz_config(n: int) -> str:
    """Genera un .rviz con Grid, TF y un LaserScan por robot. Devuelve la ruta."""
    displays = [
        {"Class": "rviz_default_plugins/Grid", "Name": "Grid", "Enabled": True,
         "Cell Size": 1.0, "Plane Cell Count": 60},
        {"Class": "rviz_default_plugins/Map", "Name": "Map", "Enabled": True,
         "Topic": {"Value": "/map", "Durability Policy": "Transient Local",
                   "Reliability Policy": "Reliable"},
         "Color Scheme": "map", "Alpha": 0.7},
        {"Class": "rviz_default_plugins/TF", "Name": "TF", "Enabled": True,
         "Show Names": True, "Marker Scale": 1.5},
    ]
    for i in range(n):
        r, g, b = _SCAN_COLORS[i % len(_SCAN_COLORS)]
        displays.append({
            "Class": "rviz_default_plugins/LaserScan",
            "Name": f"scan summit{i}",
            "Enabled": True,
            "Topic": {"Value": f"/summit{i}/scan",
                      "Reliability Policy": "Best Effort",
                      "Durability Policy": "Volatile"},
            "Style": "Points",
            "Size (Pixels)": 3,
            "Color Transformer": "FlatColor",
            "Color": f"{r}; {g}; {b}",
        })
    # Marcador persistente del ultimo goal publicado.
    displays.append({
        "Class": "rviz_default_plugins/Pose",
        "Name": "goal",
        "Enabled": True,
        "Topic": {"Value": "/goal_pose"},
        "Shape": "Arrow",
        "Color": "255; 25; 0",
    })

    config = {
        "Panels": [
            {"Class": "rviz_common/Displays", "Name": "Displays"},
            {"Class": "rviz_common/Tool Properties", "Name": "Tool Properties"},
        ],
        "Visualization Manager": {
            "Global Options": {"Fixed Frame": "map", "Frame Rate": 30},
            "Displays": displays,
            "Tools": [
                {"Class": "rviz_default_plugins/MoveCamera"},
                {"Class": "rviz_default_plugins/SetGoal",
                 "Topic": {"Value": "/goal_pose"}},
                {"Class": "rviz_default_plugins/PublishPoint",
                 "Topic": {"Value": "/clicked_point"}},
            ],
            "Views": {
                "Current": {
                    "Class": "rviz_default_plugins/Orbit",
                    "Distance": 35.0,
                    "Pitch": 1.2,
                    "Focal Point": {"X": 0.0, "Y": 12.0, "Z": 0.0},
                },
            },
        },
    }
    path = os.path.join(tempfile.gettempdir(), "swarm_rviz.rviz")
    with open(path, "w") as f:
        yaml.safe_dump(config, f, default_flow_style=False)
    return path


def _setup(context, *args, **kwargs):
    n = int(LaunchConfiguration("n_robots").perform(context))
    map_yaml = LaunchConfiguration("map").perform(context)
    if not map_yaml:
        map_yaml = os.path.join(
            get_package_share_directory("swarm_worlds"), "maps", "warehouse.yaml")

    nodes = []
    # Mapa estatico pre-construido (occupancy grid) servido en /map. El
    # lifecycle manager lo arranca (configure->activate) automaticamente.
    nodes.append(Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        output="screen",
        parameters=[{"use_sim_time": True, "yaml_filename": map_yaml,
                     "frame_id": "map", "topic_name": "map"}],
    ))
    nodes.append(Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_map", output="screen",
        parameters=[{"use_sim_time": True, "autostart": True,
                     "node_names": ["map_server"]}],
    ))
    for i in range(n):
        ns = f"summit{i}"
        # map (mundo) == summitN/odom  ->  TF estatico identidad.
        nodes.append(Node(
            package="tf2_ros", executable="static_transform_publisher",
            name=f"map_to_{ns}_odom",
            arguments=["0", "0", "0", "0", "0", "0", "map", f"{ns}/odom"],
        ))
        # El frame_id del /scan es el nombre escopado del sensor en Fortress
        # (`summitN/base_footprint/lidar`), que no existe en el URDF. Se ata por
        # identidad a `summitN/lidar_link` (mismo punto fisico, ya colocado por
        # robot_state_publisher) para que RViz pueda transformar el laser.
        nodes.append(Node(
            package="tf2_ros", executable="static_transform_publisher",
            name=f"lidar_frame_{ns}",
            arguments=["0", "0", "0", "0", "0", "0",
                       f"{ns}/lidar_link", f"{ns}/base_footprint/lidar"],
        ))
        # /summitN/tf -> /tf  y  /summitN/tf_static -> /tf_static
        nodes.append(Node(
            package="topic_tools", executable="relay",
            name=f"tf_relay_{ns}", arguments=[f"/{ns}/tf", "/tf"],
        ))
        nodes.append(Node(
            package="topic_tools", executable="relay",
            name=f"tf_static_relay_{ns}", arguments=[f"/{ns}/tf_static", "/tf_static"],
        ))

    nodes.append(Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", _make_rviz_config(n)],
        parameters=[{"use_sim_time": True}],
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("n_robots", default_value="3"),
        DeclareLaunchArgument("map", default_value="",
                              description="Ruta a un .yaml de mapa (vacio = warehouse.yaml)"),
        OpaqueFunction(function=_setup),
    ])
