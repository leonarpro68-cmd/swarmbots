"""DynamicTrashGraph aislado para supermercado: basura continua y depósitos sobre nodos.

Mantiene el grafo estático inmutable. El bloqueo se calcula por distancia
segmento-punto y se reconstruye por etapas o tras collect().
"""
from __future__ import annotations
import math
from . import geometry, cbs
try:
    from ..clearance_config import VISUAL_DEPOSIT_NAV_EXCLUSION_M
except ImportError:
    from clearance_config import VISUAL_DEPOSIT_NAV_EXCLUSION_M

TRASH_RADIUS_M=math.hypot(.09,.09)/2
TRASH_EXCLUSION_M=.936+TRASH_RADIUS_M
# Misma exclusión del centro Summit que el corredor Hybrid/Bézier.
DEPOSIT_RADIUS_M=VISUAL_DEPOSIT_NAV_EXCLUSION_M
EPS=1e-9

def segment_point_distance(a,b,p):
    vx,vy=b[0]-a[0],b[1]-a[1]
    d=vx*vx+vy*vy
    t=max(0.,min(1.,((p[0]-a[0])*vx+(p[1]-a[1])*vy)/d)) if d else 0.
    return math.hypot(a[0]+t*vx-p[0],a[1]+t*vy-p[1])

class DynamicTrashGraph(cbs.Graph):
    def __init__(self,scenario, *, pending=None, ignored_trash=(), accessible_deposits=()):
        super().__init__()
        self.scenario=scenario
        self.trash={t['name']:t for t in scenario['trash']}
        self.deposits={d['name']:d for d in scenario['deposits']}
        self.pending=set(self.trash) if pending is None else set(pending)
        self.ignored_trash=set(ignored_trash)
        self.accessible_deposits=set(accessible_deposits)
        if not self.pending<=self.trash.keys():raise ValueError('Basura desconocida')
        self.static_vertices=frozenset(self.vertices)
        self.static_edges=frozenset(self.edges)
        self.rebuild()

    def obstacle_circles(self):
        for name in sorted(self.pending-self.ignored_trash):
            yield name,tuple(self.trash[name]['xy_m']),TRASH_EXCLUSION_M
        for name,d in self.deposits.items():
            if name not in self.accessible_deposits:
                yield name,tuple(d['xy_m']),DEPOSIT_RADIUS_M

    def clear_segment(self,a,b):
        return all(segment_point_distance(a,b,p)>=radius-EPS
                   for _,p,radius in self.obstacle_circles())

    def rebuild(self):
        circles=tuple(self.obstacle_circles())
        def clear(a,b):
            return all(segment_point_distance(a,b,p)>=r-EPS for _,p,r in circles)
        self.vertices={n for n in self.static_vertices if clear(geometry.node_position(n),geometry.node_position(n))}
        self.edges={(a,b) for a,b in self.static_edges if a in self.vertices and b in self.vertices
                    and clear(geometry.node_position(a),geometry.node_position(b))}
        self.neighbors={n:[] for n in self.vertices}
        for a,b in self.edges:
            self.neighbors[a].append(b);self.neighbors[b].append(a)
        for n in self.neighbors:self.neighbors[n].sort()

    def collect(self,name):
        if name not in self.trash:raise ValueError(f'Basura desconocida: {name}')
        if name not in self.pending:raise ValueError(f'Basura ya recogida: {name}')
        self.pending.remove(name)
        self.rebuild()

    def stage(self,*,ignored_trash=(),accessible_deposits=()):
        return DynamicTrashGraph(self.scenario,pending=self.pending,
            ignored_trash=ignored_trash,accessible_deposits=accessible_deposits)
