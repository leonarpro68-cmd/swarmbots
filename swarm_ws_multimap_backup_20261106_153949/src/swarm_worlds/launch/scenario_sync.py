"""Validación estricta del escenario offline de supermarket (solo stdlib).

Devuelve poses en el formato EXACTO que usa sim_summit.launch.py original:
robots [(ns,x,y,yaw)], trash/deposits [(name,x,y,type_idx)].
No genera posiciones aleatorias y no requiere ROS/Gazebo.
"""
import json
import math
import re
from pathlib import Path

COLORS = {'verde': 0, 'azul': 1, 'naranja': 2}
ROBOT_NAMES = ('summit0', 'summit1', 'summit2')
BOUNDS = (-15.0, 15.0, -25.0, 25.0)
STEP = 1.25


def _xy(obj, label):
    a = obj.get('xy_m')
    if not isinstance(a, (list, tuple)) or len(a) != 2:
        raise ValueError(f'{label}: falta xy_m de 2 elementos')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in a):
        raise ValueError(f'{label}: coordenadas no finitas')
    x, y = map(float, a)
    if not (BOUNDS[0] < x < BOUNDS[1] and BOUNDS[2] < y < BOUNDS[3]):
        raise ValueError(f'{label}: fuera de los límites del mundo')
    return x, y


def _validate_grid(obj, xy, label):
    grid = obj.get('grid')
    if not isinstance(grid, list) or len(grid) != 2 or any(type(v) is not int for v in grid):
        raise ValueError(f'{label}: grid inválido')
    ix, iy = grid
    if not (0 <= ix <= 24 and 0 <= iy <= 40):
        raise ValueError(f'{label}: grid fuera de rango')
    gx, gy = -15.0 + STEP*ix, -25.0 + STEP*iy
    if math.hypot(gx-xy[0], gy-xy[1]) > 1e-5:
        raise ValueError(f'{label}: xy_m no corresponde a grid ({gx}, {gy})')


def _objects(objects, kind, expected):
    if not isinstance(objects, list) or len(objects) != expected:
        raise ValueError(f'Se esperaban {expected} elementos de tipo {kind}')
    converted = []
    for obj in objects:
        name = obj.get('name')
        color = obj.get('color')
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', name) or not name.startswith(f'{kind}_') or color not in COLORS or not name.endswith('_' + color):
            raise ValueError(f'Nombre/color no compatible con plugins de Gazebo: {name!r}')
        if obj.get('type') != kind:
            raise ValueError(f'{name}: tipo incorrecto')
        x,y = _xy(obj, name)
        if kind == 'deposit':
            _validate_grid(obj, (x,y), name)
        converted.append((name, x, y, COLORS[color]))
    return converted


def load_scenario(path, expected_seed=100):
    path = Path(path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f'Escenario no encontrado: {path}')
    data = json.loads(path.read_text(encoding='utf8'))
    if data.get('world') != 'supermarket':
        raise ValueError('El archivo no es del mundo supermarket')
    if data.get('seed') != expected_seed:
        raise ValueError(f'La semilla del escenario ({data.get("seed")}) no coincide con {expected_seed}')
    if data.get('units') != 'm':
        raise ValueError('Las unidades del JSON deben ser metros')
    robots_data=data.get('robots')
    if not isinstance(robots_data, list) or len(robots_data) != 3:
        raise ValueError('Se requieren exactamente tres robots')
    robots=[]
    for idx, obj in enumerate(robots_data):
        name=obj.get('name')
        if name != ROBOT_NAMES[idx]:
            raise ValueError(f'Orden/nombre de Summit inválido: {name!r}, se esperaba {ROBOT_NAMES[idx]}')
        x,y=_xy(obj,name)
        _validate_grid(obj, (x,y), name)
        yaw=obj.get('yaw_rad', 0.0) # Offline no define yaw: neutro y determinista
        if isinstance(yaw,bool) or not isinstance(yaw,(int,float)) or not math.isfinite(yaw):
            raise ValueError(f'{name}: yaw_rad inválido')
        robots.append((name,x,y,float(yaw)))
    trash=_objects(data.get('trash'), 'trash', 6)
    deposits=_objects(data.get('deposits'), 'deposit', 3)
    names=[x[0] for x in robots+trash+deposits]
    if len(names)!=len(set(names)):
        raise ValueError('Nombres repetidos en el escenario')
    if {s[3] for s in deposits} != set(COLORS.values()):
        raise ValueError('Se requiere un depósito por cada color')
    if sorted(t[3] for t in trash) != [0,0,1,1,2,2]:
        raise ValueError('Se requieren dos basuras por color')
    return {'scenario_path': str(path), 'seed': expected_seed, 'robots': robots,
            'trash': trash, 'deposits': deposits, 'source': data}
