"""Inputs matching the supplied delivery-round-2 offline experiment."""
import numpy as np
from . import geometry_reference as geometry
from . import geometry_corridors as corridor_math

REL = [
[(-7,-7),(-7,-6),(-7,-5),(-7,-4),(-7,-3),(-7,-2),(-7,-1),(-7,0),(-7,1),(-7,2),(-7,3),(-6,4),(-5,5),(-5,6),(-4,7),(-3,8),(-2,9),(-1,10),(0,11),(1,12)],
[(-2,12),(-2,11),(-2,10),(-2,9),(-2,8),(-2,7),(-2,6),(-2,5),(-2,4),(-2,3),(-2,2),(-1,1),(0,0),(1,-1),(2,-2)],
[(8,-14),(8,-13),(8,-12),(8,-11),(8,-10),(7,-9),(6,-8),(5,-7),(5,-6),(5,-5),(4,-4)],
]
TRASH = [(-1.25,5),(-10,10),(-11.25,3.75)]
DEPOSITS = [(2.5,16.25),(11.25,-6.25),(3.75,-3.75)]

def reference_inputs():
    nodes=[[(x+11,y+19) for x,y in path] for path in REL]
    return inputs_from_cbs(nodes, [(p,0.4) for p in TRASH],[(p,0.5) for p in DEPOSITS])

def inputs_from_cbs(node_paths, trash=(), deposits=()):
    """Adapt full CBS node paths, without requiring ROS runtime.

    Trash/deposit entries are ``(metric_xy, exclusion_radius)``; radii already
    describe center exclusion and must not be inflated a second time.
    """
    if not node_paths or any(len(p)<2 for p in node_paths):
        raise ValueError('All agents need at least one discrete transition')
    k=max(len(p) for p in node_paths)-1
    pts=np.asarray([[geometry.node_position(v) for v in p]
        +[geometry.node_position(p[-1])]*(k+1-len(p)) for p in node_paths],float)
    obs=[(np.asarray(p,float),geometry.CLEARANCE_COLUMN_M) for p in geometry.COLUMNS_XY]
    for point,radius in [*trash,*deposits]:
        if radius<=0:raise ValueError('exclusion radius must be positive')
        obs.append((np.asarray(point,float),float(radius)))
    return node_paths,pts,obs
