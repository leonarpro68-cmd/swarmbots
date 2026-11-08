#!/usr/bin/env python3
"""Nodos de soporte de SwarmBots para supermarket, SIN iniciar Gazebo ni una misión.

Iniciar DESPUES de sim_summit_supermarket.launch.py. Un único seguidor
por Summit es propietario de /summitN/cmd_vel. No crea metas ni manda trayectorias.

Ejemplo:
  ros2 launch swarm_worlds supermarket_support.launch.py \
    n_robots:=3 world_name:=world_demo

NO lanzar junto a relés, seguidores o gestores de agarre manuales ya activos.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    n_str = LaunchConfiguration('n_robots').perform(context)
    try:
        n = int(n_str)
    except ValueError as exc:
        raise ValueError('n_robots debe ser entero (1, 2 o 3)') from exc
    if n not in (1, 2, 3):
        raise ValueError('n_robots debe estar entre 1 y 3 para este lanzamiento')

    world_name = LaunchConfiguration('world_name').perform(context).strip()
    if not world_name or '/' in world_name or world_name != 'world_demo':
        raise ValueError(
            'Para el escenario supermarket comprobado, el mundo interno de '
            'Gazebo es world_demo; usa world_name:=world_demo')

    # Mantener los valores aprobados en Gazebo: navegación temporizada hasta
    # 2.0 m/s y seguimiento de waypoints/alineación local a 0.25 m/s.
    # Se deja finger_grip/fricción en los defaults actuales (no se modifica).
    actions = []
    for i in range(n):
        robot = f'summit{i}'

        # gz bridge: publica TF de odometría en /summitN/tf, separado del /tf
        # global. Relay le da acceso a TF2 sin tocar el modelo ni Gazebo.
        actions.append(Node(
            package='topic_tools', executable='relay',
            name=f'relay_tf_{robot}',
            arguments=[f'/{robot}/tf', '/tf'],
            output='screen',
        ))

        # ÚNICO propietario de /summitN/cmd_vel: el mismo nodo de D8 + V3.
        actions.append(Node(
            package='swarm_behavior', executable='swarm_path_follower_grasp',
            namespace=robot, name='path_follower', output='screen',
            parameters=[{
                'use_sim_time': True,
                'hybrid_enabled': True,
                'hybrid_timed_max_speed': 2.0,
                'waypoint_tolerance': 0.02,
                'max_speed': 0.25,
            }],
        ))

        # Solo publica /summitN/visual_grasp/cmd_vel; no un segundo cmd_vel.
        actions.append(Node(
            package='swarm_behavior', executable='visual_grasp',
            namespace=robot, name='visual_grasp', output='screen',
            parameters=[{'use_sim_time': True}],
        ))

        # grasp_manager utiliza temas ABSOLUTOS /<robot>/gripper/*.
        # Nombre único, namespace global, mundo real world_demo.
        actions.append(Node(
            package='swarm_behavior', executable='grasp_manager',
            name=f'grasp_manager_{robot}', output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot': robot,
                'world': world_name,
            }],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'n_robots', default_value='3',
            description='Summit a equipar (1, 2 o 3; escenario sincronizado: 3)'),
        DeclareLaunchArgument(
            'world_name', default_value='world_demo',
            description='Nombre interno del mundo Gazebo, NO nombre del escenario'),
        OpaqueFunction(function=_setup),
    ])
