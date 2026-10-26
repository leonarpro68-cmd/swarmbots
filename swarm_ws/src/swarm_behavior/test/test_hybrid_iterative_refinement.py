"""Pruebas offline Fase 4: muestras, hulls, seguridad continua y regresiones."""
from dataclasses import replace
import math

import pytest

from swarm_behavior.swarm_planning.bridge import PlanningBridge
from swarm_behavior.swarm_planning.hybrid_refinement.adapter import from_cbs_paths
from swarm_behavior.swarm_planning.hybrid_refinement.corridors import build_hybrid_corridors
from swarm_behavior.swarm_planning.hybrid_refinement.bezier_optimizer import (
    optimize_bezier, evaluate_bezier, scale_trajectory_time,
)
from swarm_behavior.swarm_planning.hybrid_refinement.iterative_refinement import (
    RefinementError, _hull, _closest_hulls, _validate_trajectory,
    rebuild_corridors_from_bezier, refine_iteratively,
)


@pytest.fixture(scope='module')
def initial():
    bridge=PlanningBridge()
    paths=bridge.plan_pickup(((8,18),(14,18),(11,14))).paths
    schedule=from_cbs_paths(paths,graph=bridge.graph)
    corridors=build_hybrid_corridors(schedule,graph=bridge.graph)
    trajectory=optimize_bezier(schedule,corridors,maxiter=150)
    return schedule,corridors,trajectory


def test_32_sampled_hulls_create_safe_corridors_for_three_robots(initial):
    schedule,old,trajectory=initial
    refined=rebuild_corridors_from_bezier(schedule,old,trajectory,samples_per_piece=32)
    assert len(refined.corridors)==old.substeps
    assert refined.robot_names==old.robot_names
    assert refined.obstacles==old.obstacles
    assert refined.corridors[0][0].halfspaces!=old.corridors[0][0].halfspaces
    for k,row in enumerate(refined.corridors):
        for i,c in enumerate(row):
            assert c.area_m2>1e-7
            assert c.t0_s==pytest.approx(old.corridors[k][i].t0_s)
            assert c.t1_s==pytest.approx(old.corridors[k][i].t1_s)
            assert c.wait==old.corridors[k][i].wait
            cp=trajectory.robots[i].control_points[k]
            assert c.contains(evaluate_bezier(cp,0),3e-6)
            assert c.contains(evaluate_bezier(cp,1),3e-6)
            for j in range(32):
                assert c.contains(evaluate_bezier(cp,j/31),3e-6)
        for i in range(3):
            for j in range(i+1,3):
                h=next(h for h in row[i].halfspaces
                       if h.kind=='robot_max_margin_refined' and h.source==row[j].robot_name)
                opposite=next(h for h in row[j].halfspaces
                              if h.kind=='robot_max_margin_refined' and h.source==row[i].robot_name)
                assert all(h.slack(p)>=-3e-6 for p in row[i].polygon)
                assert all(opposite.slack(p)>=-3e-6 for p in row[j].polygon)
                assert h.offset_m+opposite.offset_m==pytest.approx(-schedule.min_separation_m,abs=1e-8)


def test_optimized_initial_converges_without_increasing_objective(initial):
    s,c,q=initial
    r=refine_iteratively(s,c,initial_trajectory=q,iterations=3,maxiter=150)
    assert r.reason in ('converged','iteration_limit')
    assert r.iterations_completed<=3
    assert sum(x.objective_acceleration_sq for x in r.final_trajectory.robots) <= sum(
        x.objective_acceleration_sq for x in q.robots)+1e-6
    assert r.to_dict()['status']=='offline_validated_not_ros_executable'
    _validate_trajectory(s,r.final_corridors,r.final_trajectory)


def test_refinement_can_improve_early_stopped_slsqp(initial):
    s,c,_=initial
    seed=optimize_bezier(s,c,maxiter=1)
    assert not all(x.solver_success for x in seed.robots)
    r=refine_iteratively(s,c,initial_trajectory=seed,iterations=3,maxiter=150)
    assert r.iterations_completed>=1,r.reason
    assert r.history[1].objective < r.history[0].objective-1e-3
    assert all(r.history[1].solver_success)
    assert all(a.objective>=b.objective-1e-6 for a,b in zip(r.history,r.history[1:]))
    for i,robot in enumerate(r.final_trajectory.robots):
        for k,cp in enumerate(robot.control_points):
            for point in cp:
                assert r.final_corridors.corridors[k][i].contains(point,1e-6)
            if r.final_corridors.corridors[k][i].wait:
                expected=s.metric_paths[i][r.final_corridors.corridors[k][i].original_tick]
                assert all(math.dist(point,expected)<1e-6 for point in cp)
    scaled=scale_trajectory_time(r.final_trajectory)
    assert max(scaled.speed_bound_m_s)<=scaled.speed_limit_m_s+1e-8


def test_zero_iterations_keeps_original_plan(initial):
    s,c,q=initial
    r=refine_iteratively(s,c,initial_trajectory=q,iterations=0)
    assert r.reason=='iteration_limit'
    assert r.final_trajectory is q
    assert r.final_corridors is c
    assert len(r.history)==1


def test_hull_handles_waits_lines_and_polygons():
    assert _hull([(1,2)]*32)==((1.,2.),)
    assert len(_hull([(0,0),(1,0),(.5,0)]))==2
    poly=_hull([(0,0),(1,0),(1,1),(0,1),(.5,.5)])
    assert len(poly)==4
    _,_,d=_closest_hulls(poly,_hull([(2,0),(3,0),(2,1),(3,1)]))
    assert d==pytest.approx(1.)


def test_rejects_bad_inputs_without_silent_relaxation(initial):
    s,c,q=initial
    with pytest.raises(RefinementError,match='samples_per_piece'):
        rebuild_corridors_from_bezier(s,c,q,samples_per_piece=2)
    with pytest.raises(RefinementError,match='iterations'):
        refine_iteratively(s,c,initial_trajectory=q,iterations=-1)
    with pytest.raises(RefinementError,match='improvement_tolerance'):
        refine_iteratively(s,c,initial_trajectory=q,improvement_tolerance=-1)
    tampered=replace(q,robot_names=('nombres','distintos','xyz'))
    with pytest.raises(RefinementError,match='incompatibles'):
        rebuild_corridors_from_bezier(s,c,tampered)


def test_continuous_checker_catches_control_outside_corridor(initial):
    s,c,q=initial
    robot=q.robots[0]
    pieces=list(robot.control_points)
    piece=list(pieces[0])
    piece[2]=(100.,100.)
    pieces[0]=tuple(piece)
    bad=replace(q,robots=(replace(robot,control_points=tuple(pieces)),*q.robots[1:]))
    with pytest.raises(RefinementError,match='Control inseguro'):
        _validate_trajectory(s,c,bad)


def test_continuous_checker_catches_wait_motion(initial):
    s,c,q=initial
    robot_index,part=next((i,k) for k,row in enumerate(c.corridors) for i,segment in enumerate(row)
                          if segment.wait)
    robot=q.robots[robot_index]
    pieces=list(robot.control_points)
    piece=list(pieces[part])
    original=piece[2]
    piece[2]=(original[0]+.01,original[1])
    pieces[part]=tuple(piece)
    robots=list(q.robots)
    robots[robot_index]=replace(robot,control_points=tuple(pieces))
    corrupted=replace(q,robots=tuple(robots))
    with pytest.raises(RefinementError,match='WAIT'):
        _validate_trajectory(s,c,corrupted)
