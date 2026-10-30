import numpy as np
import pytest
from swarm_behavior.swarm_planning.hybrid_v2.live_start import nearest_free_starts,exact_start_inputs
from swarm_behavior.swarm_planning.bridge import PlanningBridge
from swarm_behavior.swarm_planning.scenario import generate
from swarm_behavior.swarm_planning import geometry as geom

def test_nearest_live_starts():
    b=PlanningBridge(generate(123)); verts=sorted(b.graph.vertices)
    nodes=(verts[0],verts[len(verts)//2],verts[-1])
    poses=[(geom.node_position(v)[0]+.13,geom.node_position(v)[1]+.08) for v in nodes]
    recovered,offsets=nearest_free_starts(b.graph,poses)
    assert recovered==nodes and max(offsets)<.16
    plan=b.plan_pickup(nodes)
    data,points,_=exact_start_inputs(plan,b.graph,poses)
    assert np.allclose(points[:,0],poses)
    assert all(len(x)>=2 for x in data.paths)

def test_reject_unsafe_snap():
    b=PlanningBridge(generate(123)); v=sorted(b.graph.vertices)[0]
    with pytest.raises(ValueError):
        nearest_free_starts(b.graph,[geom.node_position(v)]*3)
