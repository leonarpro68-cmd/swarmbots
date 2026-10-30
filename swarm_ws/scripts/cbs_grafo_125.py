#!/usr/bin/env python3
"""CBS discreto para el grafo geometrico SwarmBots de 1.25 m.

ETAPA OFFLINE: no utiliza ROS ni mueve robots. Cada transicion (horizontal,
vertical, diagonal o WAIT) toma UN tick abstracto. Su temporizacion aun NO es
realizable directamente en Gazebo: se requiere un ejecutor sincronizado.

Separacion fisica: compara distancia entre dos centros durante TODA una
transicion lineal sincronizada, no solo vertices o intercambios de aristas.
Para evitar errores por tracking se usa distancia minima 2*(radio+margin).

Uso: python3 cbs_grafo_125.py [--png cbs_grafo_125.png]
Requiere tener grafo_cbs_125.py en el mismo directorio.
"""
from __future__ import annotations
import argparse
from dataclasses import dataclass
import heapq
import itertools
import math
from pathlib import Path

import grafo_cbs_125 as geom

Node = tuple[int,int]
PathT = tuple[Node, ...]
MIN_SEPARATION = 2 * (geom.ROBOT_RADIUS_M + geom.EXTRA_MARGIN_M)

@dataclass(frozen=True)
class Constraint:
    agent: int
    time: int
    vertex: Node | None = None
    edge: tuple[Node,Node] | None = None

@dataclass(frozen=True)
class Conflict:
    a: int
    b: int
    time: int
    kind: str
    distance: float
    a_edge: tuple[Node,Node] | None
    b_edge: tuple[Node,Node] | None
    a_vertex: Node | None = None
    b_vertex: Node | None = None

@dataclass
class CTNode:
    paths: tuple[PathT, ...]
    constraints: frozenset[Constraint]
    cost: int
    depth: int

class Graph:
    def __init__(self):
        self.vertices, self.edges, _, _ = geom.make_graph()
        self.neighbors = {v: [] for v in self.vertices}
        for a,b in self.edges:
            self.neighbors[a].append(b)
            self.neighbors[b].append(a)
        for v in self.vertices:
            self.neighbors[v].sort()

    @staticmethod
    def pos(v):
        return geom.node_position(v)


def pos_at(path: PathT, t:int) -> Node:
    return path[min(t,len(path)-1)]


def simultaneous_distance(a0:Node,a1:Node,b0:Node,b1:Node) -> float:
    """Min distance during synchronous straight line motion (linear interpolation)."""
    ax,ay = geom.node_position(a0); aex,aey = geom.node_position(a1)
    bx,by = geom.node_position(b0); bex,bey = geom.node_position(b1)
    rx,ry = ax-bx, ay-by
    vx,vy = (aex-ax)-(bex-bx), (aey-ay)-(bey-by)
    den = vx*vx+vy*vy
    t = max(0.,min(1.,-(rx*vx+ry*vy)/den)) if den>1e-12 else 0.
    return math.hypot(rx+t*vx,ry+t*vy)


def first_conflict(paths:tuple[PathT,...], clearance:float=MIN_SEPARATION) -> Conflict | None:
    horizon=max(map(len,paths))
    for t in range(horizon):
        for a,b in itertools.combinations(range(len(paths)),2):
            pa=pos_at(paths[a],t); pb=pos_at(paths[b],t)
            d=math.dist(geom.node_position(pa),geom.node_position(pb))
            if d < clearance-1e-8:
                return Conflict(a,b,t,'proximidad en vertice',d,None,None,pa,pb)
            if t+1<horizon:
                qa=pos_at(paths[a],t+1); qb=pos_at(paths[b],t+1)
                d=simultaneous_distance(pa,qa,pb,qb)
                if d < clearance-1e-8:
                    return Conflict(a,b,t,'proximidad durante movimiento',d,(pa,qa),(pb,qb))
    return None


def low_level(g:Graph, agent:int, start:Node, goal:Node,
              constraints:frozenset[Constraint], max_time:int=65) -> PathT | None:
    """Shortest-time A* with space-time constraints, unit tick each action."""
    if start not in g.vertices or goal not in g.vertices:
        return None
    cv={(c.vertex,c.time) for c in constraints if c.agent==agent and c.vertex is not None}
    ce={(c.edge,c.time) for c in constraints if c.agent==agent and c.edge is not None}
    if (start,0) in cv:
        return None
    latest=max((c.time for c in constraints if c.agent==agent),default=-1)
    serial=itertools.count()
    def h(v): return max(abs(v[0]-goal[0]),abs(v[1]-goal[1]))
    open_=[(h(start),0,next(serial),start,0)]
    best={(start,0):0}; parent={}
    while open_:
        _,_,_,v,t=heapq.heappop(open_)
        if t>max_time: continue
        if v==goal and t>latest:
            out=[v]; key=(v,t)
            while key in parent:
                key=parent[key];out.append(key[0])
            return tuple(reversed(out))
        if t==max_time: continue
        for w in (*g.neighbors[v],v):
            if (w,t+1) in cv or ((v,w),t) in ce:continue
            nxt=(w,t+1)
            if nxt in best:continue
            best[nxt]=t+1;parent[nxt]=(v,t)
            heapq.heappush(open_,(t+1+h(w),h(w),next(serial),w,t+1))
    return None


def solve(starts:tuple[Node,...],goals:tuple[Node,...], *,
          max_ct:int=20000,max_time:int=65, verbose:bool=False, graph=None):
    if len(starts)!=len(goals) or not starts:raise ValueError('Inicios y metas incompatibles')
    g=graph if graph is not None else Graph(); logs=[]
    if first_conflict(tuple((v,) for v in starts)):
        raise ValueError('Posiciones iniciales demasiado proximas para el clearance')
    root_paths=[]
    for i,(s,tgt) in enumerate(zip(starts,goals)):
        p=low_level(g,i,s,tgt,frozenset(),max_time)
        if p is None:raise ValueError(f'Sin ruta individual para agente {i}')
        root_paths.append(p)
    root_paths=tuple(root_paths)
    root=CTNode(root_paths,frozenset(),sum(len(p)-1 for p in root_paths),0)
    logs.append(f'Raiz CT: costo={root.cost}, primer conflicto={first_conflict(root.paths)}')
    heap=[]; counter=itertools.count()
    heapq.heappush(heap,(root.cost,root.depth,next(counter),root))
    seen={root.constraints};expanded=0;generated=1
    while heap:
        _,_,_,cur=heapq.heappop(heap)
        expanded+=1
        if expanded>max_ct: raise RuntimeError(f'Limite CT de {max_ct} expansiones excedido')
        conflict=first_conflict(cur.paths)
        if conflict is None:
            logs.append(f'SOLUCION CT costo={cur.cost}, expandidos={expanded}, generados={generated}')
            return {'paths':cur.paths,'cost':cur.cost,'expanded':expanded,
                    'generated':generated,'log':logs,'graph':g}
        logs.append(f'CT depth={cur.depth} cost={cur.cost}: {conflict.kind} '
                    f'R{conflict.a}/R{conflict.b}, t={conflict.time}, d={conflict.distance:.3f}m')
        # Vertex conflict -> forbid particular agent at its OWN vertex at time t.
        # Motion conflict -> forbid EXACT transition for each agent (incl WAIT).
        for agent in (conflict.a, conflict.b):
            if conflict.a_vertex is not None:
                v=conflict.a_vertex if agent==conflict.a else conflict.b_vertex
                c=Constraint(agent,conflict.time,vertex=v)
            else:
                edge=conflict.a_edge if agent==conflict.a else conflict.b_edge
                c=Constraint(agent,conflict.time,edge=edge)
            newc=cur.constraints|{c}
            if newc in seen: continue
            seen.add(newc)
            paths=list(cur.paths)
            p=low_level(g,agent,starts[agent],goals[agent],newc,max_time)
            if p is None:continue
            paths[agent]=p
            cost=sum(len(p)-1 for p in paths)
            child=CTNode(tuple(paths),newc,cost,cur.depth+1)
            heapq.heappush(heap,(cost,child.depth,next(counter),child))
            generated+=1
            logs.append(f'  -> restringir R{agent}: {c}, costo hijo={cost}')
    raise RuntimeError('CBS no encontro una solucion dentro del horizonte')


def plot(result, png):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle,Rectangle
    fig,ax=plt.subplots(figsize=(7,11))
    ax.add_patch(Rectangle((-14,-24),28,48,fill=False,ec='black'))
    for xy in geom.COLUMNS_XY:
        ax.add_patch(Circle(xy,geom.CLEARANCE_COLUMN_M,ec='orange',ls='--',fc='orange',alpha=.13))
        ax.add_patch(Circle(xy,geom.COLUMN_RADIUS_M,color='saddlebrown'))
    colors=['tab:blue','tab:green','tab:red']
    for i,path in enumerate(result['paths']):
        pts=[geom.node_position(v) for v in path]
        ax.plot([p[0] for p in pts],[p[1] for p in pts],'-o',label=f'R{i}: {len(path)-1} ticks',c=colors[i%3],ms=3)
        ax.scatter(*pts[0],marker='s',s=100,c=colors[i%3]);ax.scatter(*pts[-1],marker='*',s=140,c=colors[i%3])
    ax.set(xlim=(-14,14),ylim=(-24,24),xlabel='X (m)',ylabel='Y (m)',title='CBS discreto sobre grafo de 1.25 m')
    ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend();fig.tight_layout()
    fig.savefig(png,dpi=160);plt.close(fig)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--png',default='cbs_grafo_125.png')
    ap.add_argument('--log',default='cbs_grafo_125_registro.txt')
    ap.add_argument('--max-ct',type=int,default=20000)
    args=ap.parse_args()
    # Escenario de prueba cruzando el centro. Metas separadas entre si.
    starts=((8,18),(14,18),(11,14))
    goals=((14,18),(8,18),(11,22))
    res=solve(starts,goals,max_ct=args.max_ct)
    for i,p in enumerate(res['paths']):
        print(f'Robot {i}: {p} ({len(p)-1} ticks)')
    print(f"Costo={res['cost']}  CT expandidos={res['expanded']}  generados={res['generated']}")
    Path(args.log).write_text('\n'.join(res['log'])+'\n',encoding='utf-8')
    plot(res,args.png)
    print('Imagen:',args.png,'Registro:',args.log)

if __name__=='__main__':main()
