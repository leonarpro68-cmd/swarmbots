"""Verificaciones independientes y conservadoras para automatización D7.

Observa poses actuales de Gazebo y odometrías ROS, no altera la simulación.
"""
from __future__ import annotations
import math
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from d6_grasp_existing import gazebo_trash_positions, snapshot_odom


def checked_positions(world='world_demo'):
    positions = gazebo_trash_positions(world)
    if not positions:
        raise RuntimeError('No se pueden leer poses trash_ de Gazebo')
    return positions


def verify_pickup(robot, trash, original, *, world='world_demo', max_carried_distance=1.05,
                  min_displacement=.30):
    positions = checked_positions(world)
    odom = snapshot_odom()
    if trash not in positions or trash not in original:
        raise RuntimeError(f'{trash}: objeto no observable; detener')
    displacement=math.dist(positions[trash], original[trash])
    attached_proximity=math.dist(positions[trash], odom[robot]['xy'])
    if displacement < min_displacement or attached_proximity > max_carried_distance:
        raise RuntimeError(f'{robot}/{trash}: agarre no corroborado por Gazebo: desplazamiento={displacement:.3f}m, proximidad={attached_proximity:.3f}m')
    return {'kind':'pickup','robot':robot,'trash':trash,
            'displacement_m':displacement,'robot_object_distance_m':attached_proximity,
            'note':'Consistencia geométrica; no equivale a verificación independiente del joint'}


def verify_delivery(robot, trash, depot, deposits, *, world='world_demo',
                    max_depot_distance=.57, min_robot_distance=.72):
    positions=checked_positions(world)
    odom=snapshot_odom()
    if trash not in positions or depot not in deposits:
        raise RuntimeError(f'{trash}/{depot}: no observable en mundo/escenario')
    dist_depot=math.dist(positions[trash], deposits[depot])
    dist_robot=math.dist(positions[trash], odom[robot]['xy'])
    if dist_depot > max_depot_distance or dist_robot < min_robot_distance:
        raise RuntimeError(f'{robot}/{trash}: entrega no corroborada: depósito={dist_depot:.3f}m, robot={dist_robot:.3f}m')
    return {'kind':'delivery','robot':robot,'trash':trash,'depot':depot,
            'object_depot_distance_m':dist_depot,'robot_object_distance_m':dist_robot,
            'note':'Consistencia geométrica tras detach confirmado por rutina original'}
