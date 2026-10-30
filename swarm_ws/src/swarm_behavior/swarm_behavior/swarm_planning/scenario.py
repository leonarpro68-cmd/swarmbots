"""Configuración explícita de escenarios SwarmBots, sin ROS/Gazebo ni estado global."""
from __future__ import annotations
from dataclasses import dataclass
import json
import math
import random
from pathlib import Path
from . import geometry as geom
from .objects import ELEMENTS, Element

@dataclass(frozen=True)
class Scenario:
    seed: int
    elements: tuple[Element, ...]
    robots: tuple[tuple[int, int], ...]
    attempt: int = 1

    def to_dict(self):
        return {
            'version': 1, 'seed': self.seed, 'attempt': self.attempt,
            'grid_step_m': geom.STEP_M,
            'elements': [{'name': e.name, 'type': e.kind, 'color': e.color,
                          'grid': list(e.node), 'xy_m': list(geom.node_position(e.node))}
                         for e in self.elements],
            'robots': [{'name': f'summit{i}', 'grid': list(v),
                        'xy_m': list(geom.node_position(v))}
                       for i,v in enumerate(self.robots)],
        }

def validate(scenario: Scenario) -> Scenario:
    valid, edges, _, _ = geom.make_graph()
    catalog = {e.name: e for e in ELEMENTS}
    if len(scenario.elements) != len(catalog) or {e.name for e in scenario.elements} != set(catalog):
        raise ValueError('Se requieren los nueve nombres de objetos del catálogo')
    if len(scenario.robots) != 3 or len(set(scenario.robots)) != 3:
        raise ValueError('Se requieren tres robots en vértices distintos')
    adj = {v: set() for v in valid}
    for a,b in edges:
        adj[a].add(b);adj[b].add(a)
    for e in scenario.elements:
        expected = catalog[e.name]
        if e.color != expected.color or e.kind != expected.kind or e.node not in valid:
            raise ValueError(f'Objeto incompatible o vértice inválido: {e.name}')
        if len(adj[e.node]) < 3:
            raise ValueError(f'Objeto sin accesos suficientes: {e.name}')
    for i,a in enumerate(scenario.elements):
        for b in scenario.elements[i+1:]:
            if math.dist(geom.node_position(a.node), geom.node_position(b.node)) < 2.0:
                raise ValueError(f'Objetos demasiado cerca: {a.name}, {b.name}')
    for v in scenario.robots:
        if v not in valid:
            raise ValueError('Robot fuera del grafo')
        if any(math.dist(geom.node_position(v), geom.node_position(e.node)) < 3.0 for e in scenario.elements):
            raise ValueError('Robot demasiado cerca de un objeto')
    return scenario

def from_dict(data) -> Scenario:
    if data.get('version') != 1 or data.get('grid_step_m') != geom.STEP_M:
        raise ValueError('Versión o resolución de grafo incompatible')
    if len(data.get('elements', ())) != 9 or len(data.get('robots', ())) != 3:
        raise ValueError('Cantidad incorrecta de objetos o robots')
    catalog = {e.name: e for e in ELEMENTS}
    elements=[]
    for item in data['elements']:
        name=item['name']
        if name not in catalog: raise ValueError(f'Nombre desconocido: {name}')
        node=tuple(item['grid'])
        if len(node)!=2 or math.dist(item['xy_m'], geom.node_position(node)) > 1e-6:
            raise ValueError('Coordenadas y vértice de objeto incoherentes')
        elements.append(Element(name,item['color'],node,item['type']))
    robot_by_name={}
    for r in data['robots']:
        if r['name'] in robot_by_name: raise ValueError('Robot duplicado')
        node=tuple(r['grid'])
        if len(node)!=2 or math.dist(r['xy_m'],geom.node_position(node))>1e-6:
            raise ValueError('Coordenadas y vértice de robot incoherentes')
        robot_by_name[r['name']]=node
    if set(robot_by_name)!={'summit0','summit1','summit2'}:
        raise ValueError('Identidades de robot incorrectas')
    return validate(Scenario(int(data['seed']),tuple(elements),
                             tuple(robot_by_name[f'summit{i}'] for i in range(3)),
                             int(data.get('attempt',1))))

def load(path):
    return from_dict(json.loads(Path(path).read_text(encoding='utf-8')))

def save(scenario,path):
    Path(path).write_text(json.dumps(validate(scenario).to_dict(),indent=2,ensure_ascii=False)+'\n',encoding='utf-8')

def generate(seed: int, attempts=2000, verifier=None):
    """Genera candidatos como el generador original. verifier(scenario) opcional
    permite exigir solución CBS de dos rondas antes de aceptar el escenario.
    Sin verifier, solo garantiza validez geométrica, NO planificabilidad.
    """
    vertices, edges, _, _ = geom.make_graph()
    adjacency={v:set() for v in vertices}
    for a,b in edges:
        adjacency[a].add(b);adjacency[b].add(a)
    ordered=sorted(vertices)
    pool_base=[v for v in ordered if len(adjacency[v])>=6 and 2<=v[0]<=20 and 2<=v[1]<=36]
    rng=random.Random(seed)
    for attempt in range(1,attempts+1):
        pool=list(pool_base);rng.shuffle(pool)
        chosen=[]
        for v in pool:
            if all(math.dist(geom.node_position(v),geom.node_position(w))>=5.0 for w in chosen):
                chosen.append(v)
                if len(chosen)==12: break
        if len(chosen)<12: continue
        scenario=Scenario(seed,tuple(Element(e.name,e.color,v,e.kind)
                                    for e,v in zip(ELEMENTS,chosen[:9])),tuple(chosen[9:]),attempt)
        try: validate(scenario)
        except ValueError: continue
        if verifier is not None and not verifier(scenario): continue
        return scenario
    raise RuntimeError('No se generó un escenario válido en los intentos permitidos')
