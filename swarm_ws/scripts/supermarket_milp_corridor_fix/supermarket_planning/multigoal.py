"""Adaptación mínima de assignment.low_level_multi/solve_multi originales.

Se conservan la heurística Chebyshev, las metas como conjuntos,
las restricciones CBS y los desempates. Diferencia: un grafo dinámico por
agente, necesario si más adelante se reintroducen restricciones privadas.
"""
from __future__ import annotations
import heapq,itertools
from . import cbs


def low_level_multi(graph,agent,start,goals,constraints,max_time=110):
    goals=frozenset(goals)
    if start not in graph.vertices or not goals or not goals <= graph.vertices:
        return None
    cv={(c.vertex,c.time) for c in constraints if c.agent==agent and c.vertex is not None}
    ce={(c.edge,c.time) for c in constraints if c.agent==agent and c.edge is not None}
    if (start,0) in cv:
        return None
    latest=max((c.time for c in constraints if c.agent==agent),default=-1)
    serial=itertools.count()
    def h(v):
        return min(max(abs(v[0]-w[0]),abs(v[1]-w[1])) for w in goals)
    queue=[(h(start),h(start),next(serial),start,0)]
    seen={(start,0)};parents={}
    while queue:
        _,_,_,v,t=heapq.heappop(queue)
        if v in goals and t>latest:
            path=[v];key=(v,t)
            while key in parents:
                key=parents[key];path.append(key[0])
            return tuple(reversed(path))
        if t>=max_time:continue
        for w in (*graph.neighbors[v],v):
            if (w,t+1) in cv or ((v,w),t) in ce:continue
            nxt=(w,t+1)
            if nxt in seen:continue
            seen.add(nxt);parents[nxt]=(v,t)
            heapq.heappush(queue,(t+1+h(w),h(w),next(serial),w,t+1))
    return None


def solve_multi(graphs,starts,goal_sets,max_ct=6000,max_time=110):
    graphs=tuple(graphs); starts=tuple(starts); goal_sets=tuple(tuple(x) for x in goal_sets)
    if len(starts)!=len(goal_sets) or len(graphs)!=len(starts) or not starts:
        raise ValueError('Inicio/metas/grafos incompatibles')
    if cbs.first_conflict(tuple((s,) for s in starts)):
        raise ValueError('Inicios demasiado próximos')
    paths=[]
    for i,(g,s,goals) in enumerate(zip(graphs,starts,goal_sets)):
        p=low_level_multi(g,i,s,goals,frozenset(),max_time)
        if p is None:
            raise ValueError(f'Sin ruta a ningún vecino válido para robot {i}')
        paths.append(p)
    root=cbs.CTNode(tuple(paths),frozenset(),sum(len(p)-1 for p in paths),0)
    logs=[f'Raíz: costo={root.cost}; conflicto={cbs.first_conflict(root.paths)}']
    counter=itertools.count();pq=[(root.cost,root.depth,next(counter),root)]
    seen={root.constraints};expanded=0;generated=1
    while pq:
        _,_,_,cur=heapq.heappop(pq);expanded+=1
        if expanded>max_ct:raise RuntimeError('Límite de nodos CT alcanzado')
        conflict=cbs.first_conflict(cur.paths)
        if conflict is None:
            logs.append(f'SOLUCIÓN: costo={cur.cost}, expandidos={expanded}, generados={generated}')
            return {'paths':cur.paths,'cost':cur.cost,'expanded':expanded,'generated':generated,'log':logs}
        for agent in (conflict.a,conflict.b):
            if conflict.a_vertex is not None:
                v=conflict.a_vertex if agent==conflict.a else conflict.b_vertex
                constraint=cbs.Constraint(agent,conflict.time,vertex=v)
            else:
                edge=conflict.a_edge if agent==conflict.a else conflict.b_edge
                constraint=cbs.Constraint(agent,conflict.time,edge=edge)
            new_constraints=cur.constraints|{constraint}
            if new_constraints in seen:continue
            seen.add(new_constraints)
            paths=list(cur.paths)
            replanned=low_level_multi(graphs[agent],agent,starts[agent],goal_sets[agent],new_constraints,max_time)
            if replanned is None:continue
            paths[agent]=replanned
            child=cbs.CTNode(tuple(paths),new_constraints,sum(len(p)-1 for p in paths),cur.depth+1)
            heapq.heappush(pq,(child.cost,child.depth,next(counter),child))
            generated+=1
    raise RuntimeError('CBS no encontró solución')
