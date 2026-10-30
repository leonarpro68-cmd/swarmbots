import numpy as np
import pytest
from swarm_behavior.swarm_planning.hybrid_v2.safety import certify, SafetySettings


def stationary(x,y,k=2):
    return np.broadcast_to(np.array([x,y],float),(k,6,2)).copy()


def test_stationary_safe_certified_and_timed():
    c=np.array([stationary(-5.,0.),stationary(5.,0.)])
    p=np.array([[[-5.,0.]]*3,[[5.,0.]]*3])
    paths=[[(0,0)]*3,[(10,0)]*3]
    result=certify(c,p,paths,[],settings=SafetySettings())
    assert result['certified'] and result['duration_s']>0
    assert result['min_pair_extra_clearance_m']>0


def test_wait_violation_is_rejected():
    c=np.array([stationary(-5.,0.)])
    c[0,0,2,0]+=.15
    p=np.array([[[-5.,0.]]*3])
    with pytest.raises(ValueError,match='WAIT'):
        certify(c,p,[[(0,0)]*3],[],preserve_intermediate_waits=True)


def test_additional_obstacle_margin_is_enforced():
    c=np.array([stationary(-5.,0.)]);p=np.array([[[-5.,0.]]*3]);paths=[[(0,0)]*3]
    with pytest.raises(ValueError,match='obstáculo'):
        certify(c,p,paths,[(np.array([-4.5,0.]),.48)])


def test_pair_margin_is_enforced():
    from swarm_behavior.swarm_planning.hybrid_v2 import geometry_reference as g
    x=2*g.CLEARANCE_WALL_M+.02
    c=np.array([stationary(-x/2,0.),stationary(x/2,0.)])
    p=np.array([[[-x/2,0.]]*3,[[x/2,0.]]*3])
    with pytest.raises(ValueError,match='interrobot'):
        certify(c,p,[[(0,0)]*3,[(1,0)]*3],[])


def test_time_scaling_derivative_bound():
    # Straight-line Bézier with no intermediate WAIT: two robots far apart.
    curve=np.array([[0.,0.],[.2,0.],[.4,0.],[.6,0.],[.8,0.],[1.,0.]])
    c=np.array([[curve + [-4,0]], [curve + [4,0]]])
    p=np.array([[[-4,0],[-3,0]],[[4,0],[5,0]]],float)
    result=certify(c,p,[[(0,0),(1,0)],[(0,1),(1,1)]],[],settings=SafetySettings(max_speed_m_s=.5,max_acceleration_m_s2=1.))
    assert result['speed_upper_bound_m_s']<=.5+1e-9
    assert result['acceleration_upper_bound_m_s2']<=1.+1e-9
