"""Validación offline de la geometría de corredores (sin ROS/SciPy/Shapely)."""
import json
import math

import pytest

from swarm_behavior.swarm_planning import cbs, geometry
from swarm_behavior.swarm_planning.bridge import PlanningBridge
from swarm_behavior.swarm_planning.hybrid_refinement.adapter import from_cbs_paths
from swarm_behavior.swarm_planning.hybrid_refinement.corridors import (
    CircleObstacle, CorridorError, HalfSpace, _closest_segments,
    build_hybrid_corridors, obstacles_for_graph,
)


def make_static_three():
    graph = cbs.Graph()
    res = cbs.solve(((8,18),(14,18),(11,14)), ((14,18),(8,18),(11,22)),graph=graph)
    return graph, from_cbs_paths(res['paths'],graph=graph)


def test_crossing_segments_are_subdivided_not_declared_safe():
    graph, schedule = make_static_three()
    with pytest.raises(CorridorError, match='No existe partición'):
        build_hybrid_corridors(schedule,graph=graph,max_subdivisions=1)
    plan = build_hybrid_corridors(schedule,graph=graph)
    assert plan.subdivisions_per_tick > 1
    assert plan.substeps == schedule.ticks * plan.subdivisions_per_tick
    assert plan.duration_s == pytest.approx(schedule.duration_s)
    assert len(plan.corridors[0]) == 3
    for k,row in enumerate(plan.corridors):
        for robot,corridor in enumerate(row):
            assert corridor.robot_name == schedule.robot_names[robot]
            assert corridor.t0_s < corridor.t1_s
            assert corridor.area_m2 > 0
            assert all(corridor.contains(point) for point in (*corridor.polygon,corridor.start,corridor.end))
            factor=plan.subdivisions_per_tick
            if k % factor == 0:
                assert corridor.start == pytest.approx(schedule.metric_paths[robot][k//factor])
            else:
                assert corridor.start == pytest.approx(plan.corridors[k-1][robot].end)
            if k % factor == factor-1:
                assert corridor.end == pytest.approx(schedule.metric_paths[robot][k//factor+1])
    # Cada par recibe dos semiplanos con distancia >= clearance.
    for row in plan.corridors:
        for i in range(3):
            for j in range(i+1,3):
                ai=next(h for h in row[i].halfspaces if h.kind=='robot_max_margin' and h.source==row[j].robot_name)
                bj=next(h for h in row[j].halfspaces if h.kind=='robot_max_margin' and h.source==row[i].robot_name)
                assert bj.normal == pytest.approx(tuple(-v for v in ai.normal))
                assert -bj.offset_m - ai.offset_m == pytest.approx(schedule.min_separation_m)
    assert sum(c.wait for row in plan.corridors for c in row) == sum(
        int(s.waits[i]) for s in schedule.steps for i in range(3)) * plan.subdivisions_per_tick


def test_nearest_segment_geometry_intersection_and_parallel():
    p,q,d = _closest_segments((0,0),(4,4),(0,4),(4,0))
    assert d == pytest.approx(0)
    p,q,d = _closest_segments((0,0),(2,0),(0,3),(2,3))
    assert d == pytest.approx(3)
    p,q,d = _closest_segments((0,0),(0,0),(3,0),(3,0))
    assert d == pytest.approx(3)


def test_dynamic_pickup_uses_six_trash_three_deposits_and_columns():
    bridge=PlanningBridge()
    plan=bridge.plan_pickup(((8,18),(14,18),(11,14)))
    schedule=from_cbs_paths(plan.paths,graph=bridge.graph)
    corridor_plan=build_hybrid_corridors(schedule,graph=bridge.graph)
    assert len(corridor_plan.obstacles)==len(geometry.COLUMNS_XY)+6+3
    assert len(corridor_plan.corridors)==schedule.ticks*corridor_plan.subdivisions_per_tick
    assert all(c.area_m2>0 for row in corridor_plan.corridors for c in row)
    report=json.loads(json.dumps(corridor_plan.to_dict()))
    assert report['status']=='geometry_only_not_executable'
    assert report['substeps']==corridor_plan.substeps
    assert len(report['corridors'])==corridor_plan.substeps
    assert len(report['corridors'][0][0]['halfspaces'])==4+2+19


def test_obstacles_reflect_dynamic_collect():
    b=PlanningBridge()
    obs_before=obstacles_for_graph(b.graph)
    name=sorted(b.graph.pending)[0]
    assert any(o.name==f'basura_{name}' for o in obs_before)
    b.confirm_collected(name)
    obs_after=obstacles_for_graph(b.graph)
    assert len(obs_before)==len(obs_after)+1
    assert not any(o.name==f'basura_{name}' for o in obs_after)


def test_obstacle_tangent_and_map_bounds_certificates():
    graph=cbs.Graph()
    schedule=from_cbs_paths(( ((2,10),(3,10)), ),graph=graph)
    plan=build_hybrid_corridors(schedule,graph=graph)
    c=plan.corridors[0][0]
    for p in c.polygon:
        assert geometry.X_MIN+geometry.CLEARANCE_WALL_M-1e-6<=p[0]<=geometry.X_MAX-geometry.CLEARANCE_WALL_M+1e-6
        assert geometry.Y_MIN+geometry.CLEARANCE_WALL_M-1e-6<=p[1]<=geometry.Y_MAX-geometry.CLEARANCE_WALL_M+1e-6
        for obs in plan.obstacles:
            assert math.dist(p,obs.center) >= obs.exclusion_radius_m-1e-6
    assert all(h.kind in ('wall','obstacle_tangent') for h in c.halfspaces)


def test_reject_unsupported_graph_type_and_invalid_subdivision():
    graph,schedule=make_static_three()
    with pytest.raises(CorridorError,match='potencia de dos'):
        build_hybrid_corridors(schedule,graph=graph,max_subdivisions=3)
    with pytest.raises(CorridorError,match='Grafo desconocido'):
        build_hybrid_corridors(schedule,graph=object())


def test_noop_schedule_has_no_motion_corridors():
    graph=cbs.Graph()
    sched=from_cbs_paths((((2,10),),((10,10),),((20,10),)),graph=graph)
    corridors=build_hybrid_corridors(sched,graph=graph)
    assert corridors.substeps==0 and corridors.duration_s==0
