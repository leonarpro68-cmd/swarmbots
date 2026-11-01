"""D.1: offline adapter from live PlanningBridge CBS plans to Hybrid V2.

No ROS, no hardcoded experimental routes and no hidden graph mutation.
Obstacle exclusions are taken from the exact DynamicTrashGraph stage snapshot.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from .. import geometry as grid_geometry
from .. import cbs
from ..dynamic_map import TRASH_EXCLUSION_M, DEPOSIT_RADIUS_M
from .scenario import inputs_from_cbs
from .pipeline_c2 import run

@dataclass(frozen=True)
class CBSV2Inputs:
    paths: tuple
    points: np.ndarray
    obstacles: tuple
    stage: str
    pending_trash: tuple


def adapt_plan(plan, graph, *, stage='unknown') -> CBSV2Inputs:
    """Convert an existing PickupPlan/DeliveryPlan without changing CBS.

    Important: call BEFORE changing graph.pending for the next mission phase.
    A one-vertex path is padded with one WAIT to support an already-at-goal robot.
    """
    raw = getattr(plan, 'paths', None)
    if not raw or len(raw) != 3:
        raise ValueError('Expected three CBS robot paths')
    paths = tuple(tuple(tuple(v) for v in path) for path in raw)
    if any(not p for p in paths):
        raise ValueError('Empty CBS path')
    if cbs.first_conflict(paths) is not None:
        raise ValueError('CBS plan contains a conflict')
    for i, path in enumerate(paths):
        if hasattr(plan,'starts') and tuple(plan.starts[i]) != path[0]:
            raise ValueError(f'Robot {i}: start mismatch')
        for v in path:
            if v not in graph.vertices:
                raise ValueError(f'Robot {i}: vertex blocked or outside current map: {v}')
        for a,b in zip(path,path[1:]):
            if a != b and b not in graph.neighbors[a]:
                raise ValueError(f'Robot {i}: invalid CBS edge {a}->{b}')
    # Never invent motion for a robot already at its goal.
    paths = tuple(p if len(p)>=2 else p+(p[-1],) for p in paths)
    trash = [(grid_geometry.node_position(graph.trash[name].node),TRASH_EXCLUSION_M)
             for name in sorted(graph.pending)]
    deposits = [(grid_geometry.node_position(d.node),DEPOSIT_RADIUS_M)
                for _,d in sorted(graph.deposits.items())]
    node_paths,points,obstacles=inputs_from_cbs(paths,trash,deposits)
    # The v2 reference geometry must agree exactly with the live CBS grid.
    for i,path in enumerate(paths):
        for k,v in enumerate(path):
            if not np.allclose(points[i,k],grid_geometry.node_position(v),atol=1e-10,rtol=0):
                raise ValueError('V2 and CBS grid geometry disagree')
    return CBSV2Inputs(paths,points,tuple(obstacles),str(stage),tuple(sorted(graph.pending)))


def plan_v2(plan, graph, *, stage='unknown', **optimizer_kwargs):
    """Run existing C.2 optimizer; failure returns REJECTED, never an unsafe plan."""
    data=adapt_plan(plan,graph,stage=stage)
    report,controls=run(data.points,data.paths,data.obstacles,**optimizer_kwargs)
    report['stage']='D.1'
    report['mission_stage']=data.stage
    report['pending_trash']=list(data.pending_trash)
    report['cbs_lengths']=[len(p) for p in data.paths]
    return report,controls
