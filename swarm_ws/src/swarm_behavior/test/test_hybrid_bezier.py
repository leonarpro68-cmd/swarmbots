"""Tests OFFLINE Fase 3: contenido, tiempos, C², WAIT, factibilidad."""
from dataclasses import replace
import math

import numpy as np
import pytest

from swarm_behavior.swarm_planning import cbs
from swarm_behavior.swarm_planning.bridge import PlanningBridge
from swarm_behavior.swarm_planning.hybrid_refinement.adapter import from_cbs_paths
from swarm_behavior.swarm_planning.hybrid_refinement.corridors import build_hybrid_corridors
from swarm_behavior.swarm_planning.hybrid_refinement.bezier_optimizer import (
    BezierOptimizationError, optimize_bezier, evaluate_bezier,
    scale_trajectory_time, _single_axis_gram,
)


@pytest.fixture(scope='module')
def pickup_case():
    bridge=PlanningBridge()
    cbs_plan=bridge.plan_pickup(((8,18),(14,18),(11,14)))
    schedule=from_cbs_paths(cbs_plan.paths,graph=bridge.graph)
    corridors=build_hybrid_corridors(schedule,graph=bridge.graph)
    curves=optimize_bezier(schedule,corridors,maxiter=150)
    return schedule,corridors,curves


def test_pickup_c2_accel_cost_decreases(pickup_case):
    schedule,corridors,curves=pickup_case
    assert len(curves.robots)==3
    assert curves.initial_duration_s == pytest.approx(schedule.duration_s)
    assert curves.dt_s == pytest.approx(schedule.tick_seconds / corridors.subdivisions_per_tick)
    for index,robot in enumerate(curves.robots):
        assert robot.solver_success, robot.solver_message
        assert robot.objective_acceleration_sq <= robot.seed_objective_acceleration_sq+1e-6
        assert robot.objective_acceleration_sq >= -1e-8
        assert len(robot.control_points)==corridors.substeps
        for k,ctrl in enumerate(robot.control_points):
            assert len(ctrl)==6
            assert all(corridors.corridors[k][index].contains(p,tolerance_m=1e-6) for p in ctrl)
        assert np.allclose(robot.control_points[0][0],schedule.metric_paths[index][0])
        assert np.allclose(robot.control_points[-1][-1],schedule.metric_paths[index][-1])
        # Toda unión satisface continuidad física, no solo posición.
        for k in range(corridors.substeps-1):
            a,b=robot.control_points[k:k+2]
            for d in (0,1,2):
                assert evaluate_bezier(a,1,derivative=d,dt_s=curves.dt_s)==pytest.approx(
                    evaluate_bezier(b,0,derivative=d,dt_s=curves.dt_s),abs=1e-6)
        for d in (1,2):
            assert np.linalg.norm(evaluate_bezier(robot.control_points[0],0,d,curves.dt_s))<1e-6
            assert np.linalg.norm(evaluate_bezier(robot.control_points[-1],1,d,curves.dt_s))<1e-6


def test_every_original_tick_is_anchored(pickup_case):
    schedule,corridors,curves=pickup_case
    f=corridors.subdivisions_per_tick
    for i,r in enumerate(curves.robots):
        for tick in range(schedule.ticks+1):
            if tick==0: pt=r.control_points[0][0]
            else: pt=r.control_points[tick*f-1][-1]
            assert pt==pytest.approx(schedule.metric_paths[i][tick],abs=1e-6)


def test_cbs_wait_is_stationary_whole_interval(pickup_case):
    schedule,corridors,curves=pickup_case
    waits=0
    for k,row in enumerate(corridors.corridors):
        for i,c in enumerate(row):
            if c.wait:
                waits+=1
                waypoint=schedule.metric_paths[i][c.original_tick]
                for control in curves.robots[i].control_points[k]:
                    assert control==pytest.approx(waypoint,abs=1e-7)
                for u in (0,0.1,.33,.65,1):
                    assert evaluate_bezier(curves.robots[i].control_points[k],u)==pytest.approx(waypoint,abs=1e-7)
    assert waits>0


def test_speed_and_acceleration_bounds_are_global(pickup_case):
    schedule,corridors,curves=pickup_case
    scaled=scale_trajectory_time(curves,max_speed_m_s=.60,max_acceleration_m_s2=.35)
    assert scaled.factor>=1.001
    assert scaled.duration_s == pytest.approx(schedule.duration_s*scaled.factor)
    assert max(scaled.speed_bound_m_s)<=.60+1e-10
    assert max(scaled.acceleration_bound_m_s2)<=.35+1e-10
    for i,robot in enumerate(curves.robots):
        for ctrl in robot.control_points:
            for u in np.linspace(0,1,13):
                speed=np.linalg.norm(evaluate_bezier(ctrl,float(u),1,scaled.dt_s))
                accel=np.linalg.norm(evaluate_bezier(ctrl,float(u),2,scaled.dt_s))
                assert speed<=scaled.speed_bound_m_s[i]+1e-7
                assert accel<=scaled.acceleration_bound_m_s2[i]+1e-7
    report=scaled.to_dict()
    assert report['status']=='offline_bezier_not_ros_executable'
    assert report['temporal_scaling']['duration_s']==pytest.approx(scaled.duration_s)


def test_static_cbs_with_temporal_subdivision():
    g=cbs.Graph()
    paths=cbs.solve(((8,18),(14,18),(11,14)),((14,18),(8,18),(11,22)),graph=g)['paths']
    schedule=from_cbs_paths(paths,graph=g)
    corridors=build_hybrid_corridors(schedule,graph=g)
    assert corridors.subdivisions_per_tick==4
    curves=optimize_bezier(schedule,corridors,maxiter=8)
    assert len(curves.robots)==3
    assert len(curves.robots[0].control_points)==schedule.ticks*4
    for i,robot in enumerate(curves.robots):
        assert robot.objective_acceleration_sq<=robot.seed_objective_acceleration_sq+1e-6
        for k,piece in enumerate(robot.control_points):
            for cp in piece:
                assert corridors.corridors[k][i].contains(cp,tolerance_m=1e-6)


def test_input_mismatch_rejected(pickup_case):
    schedule,corridors,curves=pickup_case
    bad=replace(corridors,robot_names=('otro',)+corridors.robot_names[1:])
    with pytest.raises(BezierOptimizationError,match='incompatibles'):
        optimize_bezier(schedule,bad)
    with pytest.raises(BezierOptimizationError,match='Parámetros'):
        optimize_bezier(schedule,corridors,maxiter=0)
    with pytest.raises(BezierOptimizationError,match='positivo'):
        scale_trajectory_time(curves,max_speed_m_s=0)
    with pytest.raises(BezierOptimizationError,match='>= 1'):
        scale_trajectory_time(curves,safety_factor=.99)


def test_quintic_gram_and_derivatives():
    q=_single_axis_gram(2.)
    assert q.shape==(6,6)
    assert np.allclose(q,q.T,atol=1e-10)
    assert np.linalg.eigvalsh(q).min()>=-1e-10
    # recta de velocidad constante: aceleración integrada exactamente cero
    x=np.arange(6,dtype=float)
    assert x@q@x==pytest.approx(0,abs=1e-9)
    ctrl=[(t,t*t) for t in np.linspace(0,1,6)]
    assert evaluate_bezier(ctrl,0)==pytest.approx(ctrl[0])
    assert evaluate_bezier(ctrl,1)==pytest.approx(ctrl[-1])
    with pytest.raises(ValueError):evaluate_bezier(ctrl,1.2)


def test_sampled_pair_distances_independent_check(pickup_case):
    """Chequeo diagnóstico adicional; la certificación es geométrica por polígonos."""
    schedule,corridors,curves=pickup_case
    minimum=float('inf')
    for k in range(corridors.substeps):
        for t in np.linspace(0,1,11):
            xy=[evaluate_bezier(robot.control_points[k],float(t)) for robot in curves.robots]
            for i in range(3):
                for j in range(i+1,3):
                    minimum=min(minimum,math.dist(xy[i],xy[j]))
    assert minimum >= schedule.min_separation_m-1e-5
