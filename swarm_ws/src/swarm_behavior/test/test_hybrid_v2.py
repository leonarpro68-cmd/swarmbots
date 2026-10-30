import numpy as np
from swarm_behavior.swarm_planning.hybrid_v2.scenario import reference_inputs, inputs_from_cbs
from swarm_behavior.swarm_planning.hybrid_v2.refinement import cost, sampled_corridors
from swarm_behavior.swarm_planning.hybrid_v2 import geometry_reference as g

def test_reference_inputs():
    paths,pts,obs=reference_inputs()
    assert pts.shape==(3,20,2)
    assert len(obs)==16
    assert np.allclose(pts[0,0],g.node_position(paths[0][0]))
    assert np.allclose(pts[1,-1],pts[1,len(paths[1])-1])

def test_arbitrary_cbs_input():
    nodes=[[(0,0),(0,1)],[(10,10),(10,11),(10,12)]]
    _,pts,obs=inputs_from_cbs(nodes)
    assert pts.shape==(2,3,2)
    assert np.allclose(pts[0,2],pts[0,1])

def test_reference_cost():
    from pathlib import Path
    import os
    f=os.environ.get('HYBRID_V2_REF_NPZ')
    if not f:return
    with np.load(f) as data: controls=[data[f'R{i}'] for i in range(3)]
    assert abs(cost(controls)-5.55188457)<1e-4
