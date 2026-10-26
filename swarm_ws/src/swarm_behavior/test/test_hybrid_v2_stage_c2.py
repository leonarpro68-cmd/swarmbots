import numpy as np
import pytest
from swarm_behavior.swarm_planning.hybrid_v2.geometry_corridors import corridors
from swarm_behavior.swarm_planning.hybrid_v2.safety import certify, SafetySettings
from swarm_behavior.swarm_planning.hybrid_v2.pipeline_c2 import run


def test_margin_shifts_existing_obstacle_halfplane_once():
    pts=np.array([[[-5.,0.],[-4.,0.]]])
    circle=[(np.array([-1.,0.]),0.4)]
    no=corridors(pts,circle)[0][0][-1]
    yes=corridors(pts,circle,operational_margin_m=.05)[0][0][-1]
    assert np.allclose(no[0],yes[0])
    assert yes[1]==pytest.approx(no[1]-.05)


def test_margin_shifts_robot_pair_total_gap():
    pts=np.array([[[-5.,0.],[-4.,0.]],[[4.,0.],[5.,0.]]])
    a=corridors(pts,[],operational_margin_m=0)
    b=corridors(pts,[],operational_margin_m=.1)
    assert b[0][0][-1][1]==pytest.approx(a[0][0][-1][1]-.05)
    assert b[1][0][-1][1]==pytest.approx(a[1][0][-1][1]-.05)


def test_relaxed_wait_is_not_forced_stationary():
    curve=np.array([[-4,0],[-3.8,0],[-3.6,0],[-3.4,0],[-3.2,0],[-3,0]],float)
    c=np.array([[curve]])
    p=np.array([[[-4.,0.],[-3.,0.]]])
    # Same symbolic CBS node is a WAIT, but optimization is allowed to move.
    result=certify(c,p,[[(0,0),(0,0)]],[],settings=SafetySettings(),
                   preserve_intermediate_waits=False)
    assert result['certified']
    with pytest.raises(ValueError,match='WAIT'):
        certify(c,p,[[(0,0),(0,0)]],[],settings=SafetySettings(),
                preserve_intermediate_waits=True)


def test_pipeline_rejection_is_explicit():
    pts=np.array([[[-5.,0.],[-4.,0.]]])
    result,ctrl=run(pts,[[(0,0),(1,0)]],[(np.array([-4.5,0.]),.7)],margin=.05,iterations=0)
    assert result['status']=='REJECTED' and ctrl is None
