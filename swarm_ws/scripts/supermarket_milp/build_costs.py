#!/usr/bin/env python3
"""FASE 1 OFFLINE: costos A* para MILP, sin ROS ni órdenes de movimiento.

Usa el grafo de supermarket (588 nodos / 1786 aristas) y los nodos de
aproximación del escenario. Para depósitos, usa un punto de parada
verificado para seed 100; un escenario diferente debe aportar el mismo
depósito y un punto de parada libre, o editarse mediante --stands.
"""
from __future__ import annotations
import argparse
import hashlib
import heapq
import itertools
import json
import math
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from swarm_maps import map_files

HERE = Path(__file__).resolve().parent
DEFAULT_STANDS = {
    'azul': (5.0, -10.0),
    'verde': (-2.5, -1.25),
    'naranja': (-5.0, 15.0),  # final delivery1 real; distancia euclidiana al depósito ~1.77m
}
STAGES = ('pickup1', 'delivery1', 'pickup2', 'delivery2')


def _coord(xy):
    if len(xy) != 2: raise ValueError('coordenada distinta de XY')
    out = tuple(round(float(x), 5) for x in xy)
    if not all(math.isfinite(v) for v in out): raise ValueError('coordenadas no finitas')
    return out


def _read(p):
    p = Path(p)
    return json.loads(p.read_text(encoding='utf-8'))


def _write(path, data):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, sort_keys=False)+'\n',encoding='utf-8')


class Graph:
    def __init__(self, data):
        self.meta = data.get('metadata',{})
        self.nodes = {_coord((n['x'], n['y'])) for n in data['nodes']}
        self.adj = {p: [] for p in self.nodes}
        if len(self.nodes) != len(data['nodes']): raise ValueError('nodos duplicados')
        for a,b in data['edges']:
            a,b = _coord(a), _coord(b)
            if a not in self.nodes or b not in self.nodes: raise ValueError('arista fuera del grafo')
            w = math.dist(a,b)
            if w < 1e-10: raise ValueError('arista degenerada')
            self.adj[a].append((b,w)); self.adj[b].append((a,w))
        self.edges_count = len(data['edges'])
        self._cache = {}

    def astar(self,start,goal):
        start,goal = _coord(start), _coord(goal)
        if start not in self.nodes: raise ValueError(f'Punto inicial fuera del grafo: {start}')
        if goal not in self.nodes: raise ValueError(f'Meta fuera del grafo: {goal}')
        if (start,goal) in self._cache: return self._cache[(start,goal)]
        if start==goal: return (0.0,[list(start)])
        best={start:0.}; prev={}; heap=[(math.dist(start,goal),0.,start)]
        while heap:
            _,g,u=heapq.heappop(heap)
            if g > best[u]+1e-9:continue
            if u==goal:
                rev=[u]
                while u in prev:
                    u=prev[u];rev.append(u)
                path=[list(p) for p in rev[::-1]]
                self._cache[(start,goal)]=(g,path)
                self._cache[(goal,start)]=(g,path[::-1])
                return g,path
            for v,w in self.adj[u]:
                alt=g+w
                if alt<best.get(v,float('inf'))-1e-9:
                    best[v]=alt;prev[v]=u
                    heapq.heappush(heap,(alt+math.dist(v,goal),alt,v))
        raise ValueError(f'No hay ruta A* entre {start} y {goal}')


def build(scenario, graph, speed=2.0, stands=None):
    if speed<=0 or not math.isfinite(speed):raise ValueError('Velocidad inválida')
    gg = Graph(graph)
    robots=sorted(scenario['robots'],key=lambda r:r['name'])
    trash=sorted(scenario['trash'],key=lambda t:t['name'])
    if len(robots)!=3 or len(trash)!=6: raise ValueError('Fase actual requiere exactamente 3 robots y 6 basuritas')
    if len({r['name'] for r in robots})!=3 or len({t['name'] for t in trash})!=6:
        raise ValueError('Nombre de robot/basura duplicado')
    dep={d['color']:d for d in scenario['deposits']}
    if len(dep)!=3:raise ValueError('Se requieren tres depósitos de colores distintos')
    if {t['color'] for t in trash}!=set(dep):raise ValueError('Falta algún depósito según color')
    stands=stands or DEFAULT_STANDS
    positions={}
    for r in robots:positions[r['name']]=_coord(r['xy_m'])
    for t in trash:positions[t['name']]=_coord(t['approach_node_xy_m'])
    for c,d in dep.items():
        if c not in stands:raise ValueError(f'Falta parada de depósito para {c}')
        stand=_coord(stands[c]); center=_coord(d['xy_m'])
        if math.dist(stand,center)>2.1:raise ValueError(f'Parada de {c} demasiado alejada del depósito')
        positions[f'deposit_stand_{c}']=stand
    for name,p in positions.items():
        if p not in gg.nodes:raise ValueError(f'{name} no pertenece a grafo A*: {p}')

    routes={}
    def leg(start_name,end_name):
        key=f'{start_name} -> {end_name}'
        if key not in routes:
            m, pts=gg.astar(positions[start_name],positions[end_name])
            routes[key]={'distance_m':round(m,6),'estimated_s':round(m/speed,6),
                         'start_xy_m':list(positions[start_name]),'end_xy_m':list(positions[end_name]),
                         'node_count':len(pts),'path_xy_m':pts}
        return routes[key]['distance_m']

    # Distancia desde origen de cada robot -> basura de primera ronda.
    for r in robots:
        for a in trash:leg(r['name'],a['name'])
    for a in trash:
        dest=f"deposit_stand_{a['color']}"
        leg(a['name'],dest)
        for b in trash:
            if b['name']!=a['name']:leg(dest,b['name'])
    for b in trash: leg(b['name'],f"deposit_stand_{b['color']}")

    pairs=[]
    for robot,a,b in itertools.product(robots,trash,trash):
        if a['name']==b['name']:continue
        r, an, bn=robot['name'],a['name'],b['name']
        ds=[leg(r,an),leg(an,f"deposit_stand_{a['color']}"),
            leg(f"deposit_stand_{a['color']}",bn), leg(bn,f"deposit_stand_{b['color']}")]
        pairs.append({'robot':r,'pickup1':an,'pickup2':bn,
                      'color1':a['color'],'color2':b['color'],
                      'distance_by_stage_m':dict(zip(STAGES,ds)),
                      'time_by_stage_s':dict(zip(STAGES,[round(d/speed,6) for d in ds])),
                      'total_distance_m':round(sum(ds),6)})
    return {
        'schema':'supermarket.milp.phase1.astar_costs.v1',
        'status':'OFFLINE_COSTS_NOT_CBS_NOT_EXECUTABLE',
        'world':scenario.get('world','supermarket'), 'seed':scenario.get('seed'),
        'model':{'speed_for_proxy_m_s':speed,'length_metric':'A* edge Euclidean lengths',
                 'objective_estimate':'sum of synchronized per-stage max travel times, no handling times',
                 'method':'fixed approach nodes and fixed per-color delivery stand-off',
                 'limitations':['A* solo: no CBS ni espera por conflictos',
                                'Nodos de aproximacion a basura del escenario: no necesariamente los objetivos CBS definitivos',
                                'Paradas de deposito fijas, aproximadas para segundas entregas',
                                'No incluye tiempos de manipulacion, friccion ni transporte fisico',
                                'No representa la duracion real de Bezier / V3 ni prueba seguridad dinamica']},
        'graph':{'nodes':len(gg.nodes),'edges':gg.edges_count,'metadata':gg.meta},
        'robots':{r['name']:list(positions[r['name']]) for r in robots},
        'trash':{t['name']:{'color':t['color'],'xy_m':t['xy_m'],
                            'approach_xy_m':list(positions[t['name']])} for t in trash},
        'deposit_stands':{c:{'deposit_center_xy_m':dep[c]['xy_m'],
                             'navigation_stand_xy_m':list(positions[f'deposit_stand_{c}'])} for c in sorted(dep)},
        'stages':list(STAGES),'pairs':pairs,'routes':routes,
    }


def main():
    p=argparse.ArgumentParser(description='Fase 1: matriz de costos A* sin movimiento')
    p.add_argument('--scenario',type=Path,default=HERE/'data'/'seed_100.json')
    p.add_argument('--graph',type=Path,default=map_files(HERE.parents[1], 'supermarket')[0])
    p.add_argument('--stands',type=Path,help='JSON: {"azul":[x,y],"verde":[x,y],"naranja":[x,y]}')
    p.add_argument('--speed',type=float,default=2.0)
    p.add_argument('--output',type=Path,default=HERE/'results'/'seed_100'/'astar_costs.json')
    args=p.parse_args()
    stands=_read(args.stands) if args.stands else None
    result=build(_read(args.scenario),_read(args.graph),args.speed,stands)
    result['input_sha256']={'scenario':hashlib.sha256(args.scenario.read_bytes()).hexdigest(),
                            'graph':hashlib.sha256(args.graph.read_bytes()).hexdigest()}
    _write(args.output,result)
    print(json.dumps({'status':result['status'],'nodes':result['graph']['nodes'],
                      'edges':result['graph']['edges'],'pairs':len(result['pairs']),
                      'Astar_legs':len(result['routes']),'output':str(args.output)},indent=2))

if __name__=='__main__':main()
