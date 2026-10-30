"""Prueba aislada: Gazebo + escenario CBS precargado + 3 Summit con pinzas.

NO inicia CBS, central_planner, ni elimina basuras. Reutiliza las funciones
originales de sim_summit.launch.py para generar SDF, puentes y controladores.

Preparacion previa (Docker):
  python3 scripts/escenario_aleatorio_swarm.py --seed 123 --output /tmp/swarm_cbs_grasp_123.json
  python3 scripts/swarm_cbs_grasp_prepare.py \\
    --scenario /tmp/swarm_cbs_grasp_123.json \\
    --world $(ros2 pkg prefix swarm_worlds)/share/swarm_worlds/worlds/tugbot_warehouse.sdf \\
    --output /tmp/tugbot_warehouse_grasp_123.sdf

Prueba:
  ros2 launch swarm_worlds swarm_cbs_grasp_base.launch.py seed:=123 \\
    scenario:=/tmp/swarm_cbs_grasp_123.json \\
    world_sdf:=/tmp/tugbot_warehouse_grasp_123.sdf
"""
import importlib.util
import json
import math
import os
import re
import random
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _original_sim_helpers():
    filename = Path(get_package_share_directory('swarm_worlds')) / 'launch' / 'sim_summit.launch.py'
    spec = importlib.util.spec_from_file_location('swarm_grasp_original_sim', str(filename))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _scenario_robot_specs(data, seed):
    """Spawn Summit robots at the same *validated* free nodes as offline CBS.

    This avoids the second, unrelated random generator that sometimes placed
    robots in locations whose nearest FREE grid node was > max_snap.
    """
    from swarm_behavior.swarm_planning import geometry as grid

    entries = data.get('robots', [])
    by_name = {item.get('name'): item for item in entries if isinstance(item, dict)}
    names = tuple(f'summit{i}' for i in range(3))
    if len(entries) != 3 or set(by_name) != set(names):
        raise ValueError('Se requieren exactamente tres inicios de robots diferentes en el JSON')

    valid, _, _, _ = grid.make_graph()
    specs = []
    # Independent yaw stream: deterministic, without changing CBS node positions.
    rng = random.Random(seed + 20011)
    for name in names:
        item = by_name[name]
        node_raw, xy = item.get('grid'), item.get('xy_m')
        if not (isinstance(node_raw, list) and len(node_raw) == 2 and
                all(type(v) is int for v in node_raw) and
                isinstance(xy, list) and len(xy) == 2 and
                all(type(v) in (int, float) and math.isfinite(v) for v in xy)):
            raise ValueError(f'{name}: inicio de escenario mal formado')
        node = tuple(node_raw)
        point = tuple(float(v) for v in xy)
        if node not in valid or math.dist(point, grid.node_position(node)) > 1e-5:
            raise ValueError(f'{name}: inicio fuera del grafo libre o inconsistente')
        if not grid.free_position(point):
            raise ValueError(f'{name}: no hay margen suficiente frente a paredes o columnas')
        if any(math.dist(point, (s[1], s[2])) < 2.0 for s in specs):
            raise ValueError(f'{name}: distancia inicial insuficiente entre robots')
        # No overlap with physical trash; deposits are visual and no-collision,
        # but the scenario generator keeps the same separation for all objects.
        for item_obj in data['elements']:
            if math.dist(point, item_obj['xy_m']) < 3.0:
                raise ValueError(f'{name}: inicio demasiado cerca de {item_obj["name"]}')
        specs.append((name, *point, rng.uniform(-math.pi, math.pi)))
    return specs


def _setup(context, *args, **kwargs):
    seed = int(LaunchConfiguration('seed').perform(context))
    scenario_path = Path(LaunchConfiguration('scenario').perform(context)).expanduser()
    world_path = Path(LaunchConfiguration('world_sdf').perform(context)).expanduser()
    headless = LaunchConfiguration('headless').perform(context).lower() in ('true', '1', 'yes')
    hybrid_enabled = LaunchConfiguration('hybrid_enabled').perform(context).lower() in ('true', '1', 'yes')
    hybrid_max_speed = float(LaunchConfiguration('hybrid_max_speed').perform(context))
    if not 0 < hybrid_max_speed <= 1.20:
        raise ValueError('hybrid_max_speed debe estar en (0, 1.20] m/s')
    if not scenario_path.is_file() or not world_path.is_file():
        raise FileNotFoundError('Falta JSON o mundo SDF. Ejecuta primero ambos preparadores.')
    data = json.loads(scenario_path.read_text(encoding='utf-8'))
    if data.get('seed') != seed:
        raise ValueError(f'Semilla inconsistente: launch={seed}, JSON={data.get("seed")}')
    objects = data.get('elements', [])
    trash_names = [e['name'] for e in objects if e.get('type') == 'trash']
    if len(trash_names) != 6 or sum(e.get('type') == 'deposit' for e in objects) != 3:
        raise ValueError('Se esperan seis basuras y tres depositos')

    with open(world_path, encoding="utf-8") as f:
        content = f.read()

    match = re.search(r'<world\b[^>]*\bname\s*=\s*["\']([^"\']+)["\']', content)

    if not match:
        raise RuntimeError(
            f"No se encontró el nombre del mundo en {world_path}"
        )


    tree = ET.parse(world_path)
    world = tree.getroot().find('world')
    world_name = world.get('name')
    if world is None:
        raise ValueError('Falta <world> en archivo SDF')
    model_names = {m.get('name') for m in world.findall('model')}
    absent = {e['name'] for e in objects} - model_names
    if absent:
        raise ValueError(f'Modelos del JSON ausentes del mundo: {sorted(absent)}')

    sim = _original_sim_helpers()
    models_dir = os.path.join(get_package_share_directory('swarm_worlds'), 'models')
    for env_name in ('GZ_SIM_RESOURCE_PATH', 'SDF_PATH'):
        entries = os.environ.get(env_name, '').split(':')
        if models_dir not in entries:
            os.environ[env_name] = models_dir + (':' + os.environ[env_name] if os.environ.get(env_name) else '')
    # D8.7: el mismo JSON determina las posiciones de spawn y el CBS offline.
    # El planificador en vivo sigue tomando la odometria REAL, sin inventar poses.
    specs = _scenario_robot_specs(data, seed)
    if len(specs) != 3:
        raise RuntimeError(f'No se pudieron generar 3 Summit: {len(specs)}')
    print('[GRASP BASE] mundo:', world_path, flush=True)
    print('[GRASP BASE] basuras:', ', '.join(trash_names), flush=True)
    print('[GRASP BASE] Summit spawn desde scenario.json (validado):', specs, flush=True)
    print(f'[GRASP BASE] hybrid_enabled={hybrid_enabled} max_speed={hybrid_max_speed if hybrid_enabled else 0.60:.2f} m/s, attach_timeout=5.0 s', flush=True)
    sim._force_nvidia_render()
    gz_args = f'-r -v 4 {world_path}' + (' -s --headless-rendering' if headless else '')
    gz_launch = Path(get_package_share_directory('ros_gz_sim')) / 'launch' / 'gz_sim.launch.py'
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gz_launch)),
        launch_arguments={'gz_args': gz_args}.items(),
    )]
    trash_list = ' '.join(trash_names)
    for ns, x, y, yaw in specs:
        actions.extend(sim._spawn_for_robot(ns, x, y, yaw, True, trash_list, trash_names))
        actions.append(Node(
            package='swarm_behavior', executable='swarm_path_follower_grasp',
            namespace=ns, name='path_follower', output='screen',
            parameters=[{'use_sim_time': True, 'max_speed': hybrid_max_speed if hybrid_enabled else 0.60,
                         'kp': 1.2, 'waypoint_tolerance': 0.08,
                         'hybrid_enabled': hybrid_enabled}],
        ))
        actions.append(Node(
            package='swarm_behavior', executable='visual_grasp',
            namespace=ns, name='visual_grasp', output='screen',
            parameters=[{'use_sim_time': True,
                         'near_field_handoff_enabled': True}],
        ))
        actions.append(Node(
            package='swarm_behavior', executable='grasp_manager',
            name=f'grasp_manager_{ns}', output='screen',
            parameters=[{'use_sim_time': True, 'robot': ns,
                         'world': world.get('name'),
                         'attach_timeout': 5.0}],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('seed', default_value='123'),
        DeclareLaunchArgument('scenario', default_value='/tmp/swarm_cbs_grasp_123.json'),
        DeclareLaunchArgument('world_sdf', default_value='/tmp/tugbot_warehouse_grasp_123.sdf'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('hybrid_enabled', default_value='false'),
        DeclareLaunchArgument('hybrid_max_speed', default_value='0.60'),
        OpaqueFunction(function=_setup),
    ])
