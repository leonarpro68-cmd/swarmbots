"""Safe V2 pickup inputs with exact metric start poses from Gazebo.

CBS still operates on valid grid nodes. Bezier endpoint constraints use the
actual odometry starts. No controller or ROS dependency.
"""
import math
import numpy as np
from .. import geometry as grid
from .cbs_adapter import adapt_plan


def nearest_free_starts(graph, xy, *, max_snap_m=0.75):
    if len(xy) != 3: raise ValueError('Expected three robot positions')
    valid=tuple(graph.vertices)
    if not valid: raise ValueError('No free grid vertices')
    nodes=[]; offsets=[]
    for i, p in enumerate(xy):
        if len(p)!=2 or not all(math.isfinite(float(v)) for v in p):
            raise ValueError(f'Robot {i}: invalid pose')
        node=min(valid,key=lambda v:math.dist(grid.node_position(v),p))
        error=math.dist(grid.node_position(node),p)
        if error > max_snap_m:
            raise ValueError(f'Robot {i}: distance to nearest free grid node {error:.3f} m > {max_snap_m:.3f} m')
        nodes.append(node); offsets.append(error)
    if len(set(nodes)) != 3: raise ValueError('Two robots map onto same CBS node')
    return tuple(nodes),tuple(offsets)


def exact_start_inputs(plan, graph, xy, *, max_snap_m=0.75, stage='pickup1'):
    starts, offsets=nearest_free_starts(graph,xy,max_snap_m=max_snap_m)
    if tuple(tuple(v) for v in plan.starts) != starts:
        raise ValueError('CBS plan starts do not correspond to current odometry')
    data=adapt_plan(plan,graph,stage=stage)
    points=np.asarray(data.points,dtype=float).copy()
    # Only the first point changes. This is a new SLSQP solve, NEVER a
    # translation of previously certified curves.
    for i,p in enumerate(xy):
        points[i,0]=np.asarray(p,dtype=float)
    return data,points,offsets
