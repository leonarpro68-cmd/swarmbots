#!/usr/bin/env python3
"""Primera ronda offline: asignación de 3 basuras y CBS hacia vecinos libres.

Requiere grafo_cbs_125.py, colocar_basuras_depositos.py,
grafo_basuras_dinamico.py y cbs_grafo_125.py en la misma carpeta.
No conecta con ROS ni modifica Gazebo.
"""
from __future__ import annotations
import argparse
import heapq
import itertools
import math
from pathlib import Path
import grafo_cbs_125 as geom
import cbs_grafo_125 as core
from grafo_basuras_dinamico import DynamicTrashGraph

# Inicios de la primera prueba real de los tres Summit (seed=42).
DEFAULT_STARTS = ((12,16),(15,25),(8,18))


def approach_vertices(graph, trash_name):
    """Vecinos (hasta ocho) del nodo ocupado por basura, libres y alcanzables por arista local.

    No requieren conexión al vértice bloqueado: el objetivo es aproximarse,
    no atravesar la basura. La validez del grafo incluye los otros objetos.
    """
    i,j = graph.trash[trash_name].node
    return tuple(sorted((i+di,j+dj) for di in (-1,0,1) for dj in (-1,0,1)
                        if (di or dj) and (i+di,j+dj) in graph.vertices))


def assign_nearest(starts, graph):
    """Asignación iterativa del par robot-basura más cercano entre los disponibles.

    Distancias euclidianas inicio-centro de basura, ignorando obstáculos del camino.
    Desempate reproducible por índice del robot y nombre del objeto.
    """
    if len(starts)>len(graph.pending):
        raise ValueError('No hay suficientes basuras distintas para cada robot')
    unused=set(range(len(starts)))
    remaining=set(graph.pending)
    result={}
    while unused:
        candidates=[]
        for robot in sorted(unused):
            for name in sorted(remaining):
                # Excluir basuras sin posiciones de recogida accesibles.
                if not approach_vertices(graph,name):
                    continue
                distance=math.dist(geom.node_position(starts[robot]),
                                   geom.node_position(graph.trash[name].node))
                candidates.append((distance,robot,name))
        if not candidates:
            raise ValueError('No hay suficientes basuras con vecinos transitables')
        distance,robot,name=min(candidates)
        result[robot]=(name,distance)
        unused.remove(robot);remaining.remove(name)
    return tuple(result[i] for i in range(len(starts)))


def low_level_multi(graph,agent,start,goals,constraints,max_time=65):
    """A* espacio-temporal con CONJUNTO de metas; costo unitario CBS."""
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
            if (w,t+1) in cv or ((v,w),t) in ce: continue
            nxt=(w,t+1)
            if nxt in seen: continue
            seen.add(nxt);parents[nxt]=(v,t)
            heapq.heappush(queue,(t+1+h(w),h(w),next(serial),w,t+1))
    return None


def solve_multi(graph,starts,goal_sets,max_ct=20000,max_time=65):
    if len(starts)!=len(goal_sets) or not starts:
        raise ValueError('Inicio/metas incompatibles')
    if core.first_conflict(tuple((s,) for s in starts)):
        raise ValueError('Inicios demasiado próximos')
    root_paths=[]
    for i,(start,goals) in enumerate(zip(starts,goal_sets)):
        path=low_level_multi(graph,i,start,goals,frozenset(),max_time)
        if path is None:
            raise ValueError(f'Sin ruta a ningún vecino válido para robot {i}')
        root_paths.append(path)
    root_paths=tuple(root_paths)
    root=core.CTNode(root_paths,frozenset(),sum(len(p)-1 for p in root_paths),0)
    logs=[f'Raíz: costo={root.cost}; conflicto={core.first_conflict(root.paths)}']
    counter=itertools.count();pq=[(root.cost,root.depth,next(counter),root)]
    seen={root.constraints};expanded=0;generated=1
    while pq:
        _,_,_,cur=heapq.heappop(pq);expanded+=1
        if expanded>max_ct:
            raise RuntimeError('Límite de nodos CT alcanzado')
        conflict=core.first_conflict(cur.paths)
        if conflict is None:
            logs.append(f'SOLUCIÓN: costo={cur.cost}, expandidos={expanded}, generados={generated}')
            return {'paths':cur.paths,'cost':cur.cost,'expanded':expanded,'generated':generated,'log':logs}
        logs.append(f'CT depth={cur.depth}, costo={cur.cost}: {conflict}')
        for agent in (conflict.a,conflict.b):
            if conflict.a_vertex is not None:
                v=conflict.a_vertex if agent==conflict.a else conflict.b_vertex
                constraint=core.Constraint(agent,conflict.time,vertex=v)
            else:
                edge=conflict.a_edge if agent==conflict.a else conflict.b_edge
                constraint=core.Constraint(agent,conflict.time,edge=edge)
            new_constraints=cur.constraints|{constraint}
            if new_constraints in seen:continue
            seen.add(new_constraints)
            paths=list(cur.paths)
            replanned=low_level_multi(graph,agent,starts[agent],goal_sets[agent],new_constraints,max_time)
            if replanned is None:continue
            paths[agent]=replanned
            child=core.CTNode(tuple(paths),new_constraints,sum(len(p)-1 for p in paths),cur.depth+1)
            heapq.heappush(pq,(child.cost,child.depth,next(counter),child))
            generated+=1
            logs.append(f'  -> Restringir robot {agent}: {constraint}; costo hijo={child.cost}')
    raise RuntimeError('CBS no encontró solución')


def plan_first_round(starts=DEFAULT_STARTS,max_ct=20000):
    graph=DynamicTrashGraph()
    if not all(s in graph.vertices for s in starts):
        raise ValueError('Inicio fuera del grafo libre; primero alinear robots')
    assignment=assign_nearest(starts,graph)
    goal_sets=tuple(approach_vertices(graph,name) for name,_ in assignment)
    result=solve_multi(graph,starts,goal_sets,max_ct=max_ct)
    for robot,path in enumerate(result['paths']):
        assert path[0]==starts[robot] and path[-1] in goal_sets[robot]
        assert all(v in graph.vertices for v in path)
        assert all(a==b or b in graph.neighbors[a] for a,b in zip(path,path[1:]))
    assert core.first_conflict(result['paths']) is None
    return graph,assignment,goal_sets,result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--starts',nargs=6,type=int,metavar=('I0','J0','I1','J1','I2','J2'))
    parser.add_argument('--log',default='cbs_asignacion_basuras_registro.txt')
    parser.add_argument('--max-ct',type=int,default=20000)
    args=parser.parse_args()
    starts=tuple(zip(args.starts[0::2],args.starts[1::2])) if args.starts else DEFAULT_STARTS
    graph,assignment,goals,result=plan_first_round(starts,args.max_ct)
    lines=['PRIMERA RONDA OFFLINE: ASIGNACIÓN + CBS',f'Inicios: {starts}',
           f'Basuras bloqueadas: {sorted(graph.pending)}']
    for i,((name,d),options,path) in enumerate(zip(assignment,goals,result['paths'])):
        info=f'R{i}: basura={name}, distancia asignación={d:.2f}m, vecinos={options}, meta elegida={path[-1]}, ruta={path}'
        print(info);lines.append(info)
    info=f'CBS costo={result["cost"]}, CT expandidos={result["expanded"]}, generados={result["generated"]}'
    print(info);lines.append(info);lines.extend(result['log'])
    Path(args.log).write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print('Registro:',args.log)

if __name__=='__main__':main()
