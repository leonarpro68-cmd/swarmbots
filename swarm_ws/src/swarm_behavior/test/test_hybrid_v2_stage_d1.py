from types import SimpleNamespace
import numpy as np
import pytest
from swarm_behavior.swarm_planning import geometry
from swarm_behavior.swarm_planning.dynamic_map import DynamicTrashGraph, TRASH_EXCLUSION_M, DEPOSIT_RADIUS_M
from swarm_behavior.swarm_planning.hybrid_v2.cbs_adapter import adapt_plan


def test_adapter_has_live_obstacle_radii_and_geometry():
    graph=DynamicTrashGraph(elements=())
    v=sorted(graph.vertices)
    # Isolated standing agents are legitimate stage inputs.
    p=[v[0],v[len(v)//2],v[-1]]
    obj=SimpleNamespace(paths=tuple((node,) for node in p),starts=tuple(p))
    data=adapt_plan(obj,graph,stage='pickup1')
    assert data.points.shape==(3,2,2)
    assert all(len(x)==2 and x[0]==x[1] for x in data.paths)
    assert np.allclose(data.points[0,0], geometry.node_position(p[0]))
    assert len(data.obstacles)==len(geometry.COLUMNS_XY)


def test_dynamic_stage_obstacles_from_pending_and_deposits():
    from swarm_behavior.swarm_planning.scenario import generate
    scenario=generate(123)
    graph=DynamicTrashGraph(elements=scenario.elements)
    vertices=sorted(graph.vertices)
    # choose three far apart valid start nodes
    p=(vertices[0],vertices[len(vertices)//2],vertices[-1])
    obj=SimpleNamespace(paths=tuple((n,) for n in p),starts=p)
    data=adapt_plan(obj,graph)
    assert len(data.obstacles)==len(geometry.COLUMNS_XY)+len(graph.pending)+len(graph.deposits)
    assert sum(abs(r-TRASH_EXCLUSION_M)<1e-12 for _,r in data.obstacles)>=len(graph.pending)
    name=next(iter(graph.pending))
    graph.collect(name)
    next_data=adapt_plan(obj,graph)
    assert name not in next_data.pending_trash
    assert len(next_data.obstacles)==len(data.obstacles)-1


def test_bad_edge_is_rejected():
    g=DynamicTrashGraph(elements=())
    v=sorted(g.vertices)
    p=((v[0],v[-1]),(v[1],),(v[-2],))
    obj=SimpleNamespace(paths=p,starts=tuple(x[0] for x in p))
    with pytest.raises(ValueError):
        adapt_plan(obj,g)
