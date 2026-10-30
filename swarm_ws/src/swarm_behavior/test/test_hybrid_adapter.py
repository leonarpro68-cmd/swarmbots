"""Pruebas offline del adaptador; no requieren ROS, Gazebo ni SciPy."""
import json
import math

import pytest

from swarm_behavior.swarm_planning import cbs, geometry
from swarm_behavior.swarm_planning.dynamic_map import DynamicTrashGraph
from swarm_behavior.swarm_planning.hybrid_refinement import (
    CBSAdapterError, from_cbs_paths,
)


def test_cbs_real_three_summits_round_trip():
    graph = cbs.Graph()
    result = cbs.solve(((8, 18), (14, 18), (11, 14)),
                       ((14, 18), (8, 18), (11, 22)), graph=graph)
    schedule = from_cbs_paths(result['paths'], graph=graph)
    assert schedule.robot_names == ('summit0', 'summit1', 'summit2')
    assert schedule.ticks == max(len(p) for p in result['paths']) - 1
    assert schedule.max_step_m <= math.sqrt(2) * geometry.STEP_M + 1e-8
    assert schedule.tick_seconds >= schedule.max_step_m / .60
    assert schedule.duration_s == pytest.approx(schedule.ticks * schedule.tick_seconds)
    assert cbs.first_conflict(schedule.grid_paths) is None
    assert schedule.steps[0].t0 == 0
    assert schedule.steps[-1].t1 == pytest.approx(schedule.duration_s)
    assert schedule.metric_paths[0][0] == geometry.node_position((8, 18))
    assert schedule.metric_paths[1][-1] == geometry.node_position((8, 18))
    data = json.loads(json.dumps(schedule.to_dict()))
    assert data['frame_id'] == 'map'
    assert data['time_semantics'] == 'nominal_global_time_not_executable_trajectory'
    assert len(data['times_s']) == schedule.ticks + 1


def test_padding_and_internal_wait_are_preserved():
    graph = cbs.Graph()
    paths = (
        ((2, 10), (2, 10), (2, 11)),
        ((13, 10), (14, 10)),
        ((20, 10),),
    )
    schedule = from_cbs_paths(paths, graph=graph)
    assert schedule.was_padded == (0, 1, 2)
    assert schedule.grid_paths[0] == paths[0]
    assert schedule.grid_paths[1] == ((13, 10), (14, 10), (14, 10))
    assert schedule.grid_paths[2] == ((20, 10), (20, 10), (20, 10))
    assert schedule.steps[0].waits == (True, False, True)
    assert schedule.steps[1].waits == (False, True, True)


def test_rejects_too_short_tick_for_diagonal():
    graph = cbs.Graph()
    paths = (((2, 10), (3, 11)), ((15, 15), (15, 15)))
    with pytest.raises(CBSAdapterError, match='demasiado corto'):
        from_cbs_paths(paths, graph=graph, max_speed_m_s=.60, tick_seconds=1.)


def test_rejects_collision_during_synchronous_motion():
    # Distancia entre vértices de una arista horizontal: 1.25 m, por debajo
    # de la distancia de seguridad entre dos Summit (~1.52 m).
    graph = cbs.Graph()
    with pytest.raises(CBSAdapterError, match='Conflicto CBS'):
        from_cbs_paths((((3, 12),), ((4, 12),)), graph=graph)


def test_rejects_non_adjacent_edge():
    graph = cbs.Graph()
    with pytest.raises(CBSAdapterError, match='transición no válida'):
        from_cbs_paths((((2, 10), (5, 10)),), graph=graph)


def test_rejects_blocked_dynamic_trash_vertex():
    graph = DynamicTrashGraph()
    pending_trash = next(iter(graph.trash.values()))
    assert pending_trash.node not in graph.vertices
    with pytest.raises(CBSAdapterError, match='bloqueado'):
        from_cbs_paths(((pending_trash.node,),), graph=graph)


def test_rejects_invalid_names_and_speed():
    g = cbs.Graph()
    with pytest.raises(CBSAdapterError, match='Nombres'):
        from_cbs_paths((((2, 10),), ((15, 10),)), graph=g,
                       robot_names=('summit0', 'summit0'))
    with pytest.raises(CBSAdapterError, match='max_speed_m_s'):
        from_cbs_paths((((2, 10),),), graph=g, max_speed_m_s=-1)
