"""Rutas y validación de los dos archivos de cada mapa SwarmBots.

El directorio config/maps/<nombre>/ solo contiene grafo.json y obstaculos.json.
Los escenarios y resultados de cada semilla se almacenan FUERA de config/maps.
Para supermarket se conservan las rutas de resultados heredadas.
"""
from __future__ import annotations
import json
import math
import re
from pathlib import Path

MAPS = ('supermarket', 'tugbot_warehouse')  # mapas incluidos en esta entrega, no lista cerrada
STEP_M = 1.25


def check_map_name(name):
    if not isinstance(name,str) or not re.fullmatch(r'[a-z][a-z0-9_]*',name):
        raise ValueError(f'Nombre de mapa inválido: {name!r}')
    return name


def map_files(ws: Path, name: str):
    check_map_name(name)
    folder = Path(ws) / 'config' / 'maps' / name
    paths = (folder / 'grafo.json', folder / 'obstaculos.json')
    for path in paths:
        if not path.is_file():
            raise FileNotFoundError(f'Falta el archivo del mapa {name}: {path}')
    return paths


def scenario_file(ws: Path, name: str, seed: int):
    check_map_name(name)
    if name == 'supermarket':
        return Path(ws)/'config'/'supermarket'/f'seed_{seed}.json'
    return Path(ws)/'config'/'scenarios'/name/f'seed_{seed}.json'


def generation_dir(ws: Path, name: str, seed: int):
    root=Path(ws)/'scripts'/'supermarket_escenarios'/'resultados'
    return root/f'seed_{seed}' if name=='supermarket' else root/name/f'seed_{seed}'


def milp_results_dir(ws: Path, name: str, seed: int):
    root=Path(ws)/'scripts'/'supermarket_milp'/'results'
    return root/f'seed_{seed}' if name=='supermarket' else root/name/f'seed_{seed}'


def plans_dir(ws: Path, name: str, seed: int):
    root=Path(ws)/'scripts'/'supermarket_seed_pipeline'/'plans'
    return root/f'seed_{seed}' if name=='supermarket' else root/name/f'seed_{seed}'


def validate_map(graph_file: Path, obstacles_file: Path):
    """Comprobar contrato mapa/grafo sin regenerar ni eliminar excepciones aprobadas."""
    graph=json.loads(Path(graph_file).read_text(encoding='utf-8'))
    geom=json.loads(Path(obstacles_file).read_text(encoding='utf-8'))
    res=float(graph['metadata']['resolution_m'])
    step=float(geom['graph']['step_m'])
    if not (math.isclose(res, STEP_M) and math.isclose(step, STEP_M)):
        raise ValueError('La retícula de navegación debe tener resolución de 1.25 m')
    go=graph.get('metadata',{}).get('origin',geom['graph']['origin'])
    origin=geom['graph']['origin']
    if any(abs(float(a)-float(b))>1e-6 for a,b in zip(go,origin)):
        raise ValueError('Orígenes del grafo y los obstáculos no coinciden')
    nodes={(int(n['i']),int(n['j'])) for n in graph['nodes']}
    if len(nodes)!=len(graph['nodes']): raise ValueError('Nodos duplicados en el mapa')
    xy={(round(float(n['x']),5),round(float(n['y']),5)) for n in graph['nodes']}
    if len(xy)!=len(nodes): raise ValueError('XY duplicados en el mapa')
    for n in graph['nodes']:
        px=float(origin[0])+STEP_M*int(n['i'])
        py=float(origin[1])+STEP_M*int(n['j'])
        if math.dist((px,py),(float(n['x']),float(n['y'])))>1e-5:
            raise ValueError(f'Nodo con coordenadas incoherentes: {n}')
    for a,b in graph['edges']:
        if tuple(map(lambda x:round(float(x),5),a)) not in xy or tuple(map(lambda x:round(float(x),5),b)) not in xy:
            raise ValueError(f'Arista fuera del grafo: {a} {b}')
    bounds=geom['bounds']
    if not (bounds['xmin']<bounds['xmax'] and bounds['ymin']<bounds['ymax']):
        raise ValueError('Límites de mapa inválidos')
    return graph,geom
