"""CBS multigoal -> matriz física de Hybrid V2 y obstáculos por etapa.

No modifica CBS, SLSQP ni SwarmBots original. Las basuras permanecen
bloqueadas para todos los robots HASTA collect(), incluso la asignada.
Las metas elegidas son vecinos libres, nunca el centro del objeto.
"""
from __future__ import annotations
import numpy as np
from clearance_config import TRASH_CENTER_EXCLUSION_M, VISUAL_DEPOSIT_NAV_EXCLUSION_M


def build_stage(scenario, stage, mapdata):
    paths=stage['paths_grid']; selected=stage['selected_goals']
    if len(paths)!=3 or any(list(path[-1])!=list(g) for path,g in zip(paths,selected)):
        raise ValueError('Meta CBS multigoal no coincide con último nodo')
    if any(tuple(g) not in {tuple(p) for p in opts} for g,opts in zip(selected,stage['goal_options'])):
        raise ValueError('Meta CBS no pertenece a las opciones de aproximación')
    K=max(map(len,paths))-1
    if K<=0:raise ValueError('Etapa sin intervalos activos')
    origin=np.asarray(mapdata['graph']['origin'],float)
    step=float(mapdata['graph']['step_m'])
    points=[]
    for path in paths:
        pp=np.asarray(path,float)*step+origin
        if len(pp)<K+1:
            pp=np.concatenate([pp,np.repeat(pp[-1:,:],K+1-len(pp),axis=0)])
        points.append(pp)
    points=np.asarray(points)
    pending=set(stage['pending_before'])
    known={t['name'] for t in scenario['trash']}
    if not pending <= known:raise ValueError('Basura pendiente desconocida')
    circles=[]
    for i in range(3):
        obstacles=[]
        for t in scenario['trash']:
            if t['name'] in pending:
                obstacles.append((tuple(t['xy_m']), TRASH_CENTER_EXCLUSION_M))
        # Depósitos son marcadores visuales. Se conserva el bloqueo virtual
        # de navigation según el grafo multigoal utilizado por CBS.
        for dep in scenario['deposits']:
            obstacles.append((tuple(dep['xy_m']), VISUAL_DEPOSIT_NAV_EXCLUSION_M))
        circles.append(obstacles)
    return points,circles,set()
