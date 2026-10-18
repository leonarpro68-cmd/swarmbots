#!/usr/bin/env python3
"""Prepara, SIN iniciar Gazebo, un mundo SDF con objetos del escenario CBS.

Uso (tras generar el JSON con escenario_aleatorio_swarm.py):
  python3 swarm_cbs_grasp_prepare.py --scenario /tmp/swarm_cbs_escenario.json \
     --world /ruta/tugbot_warehouse.sdf --output /tmp/tugbot_warehouse_grasp.sdf

Los modelos de basura son DINÁMICOS y tienen el link 'link', como requiere
DetachableJoint. El archivo original del mundo nunca se modifica.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from xml.sax.saxutils import escape

COLORS = {
    'verde': ('0.2 0.6 0.3', '1.36e-3', '1.36e-3', '7.2e-4'),
    'azul': ('0.3 0.4 0.7', '1.36e-3', '1.36e-3', '7.2e-4'),
    'naranja': ('0.7 0.4 0.2', '1.36e-3', '1.36e-3', '7.2e-4'),
}


def validate_scenario(path: Path):
    data = json.loads(path.read_text(encoding='utf-8'))
    elements = data.get('elements', [])
    if data.get('version') != 1 or len(elements) != 9:
        raise ValueError('Se requiere escenario version=1 con nueve objetos')
    if sum(x.get('type') == 'trash' for x in elements) != 6:
        raise ValueError('Se requieren seis basuras')
    if sum(x.get('type') == 'deposit' for x in elements) != 3:
        raise ValueError('Se requieren tres depositos')
    names = [x.get('name') for x in elements]
    if any(not isinstance(n, str) or not n for n in names) or len(set(names)) != len(names):
        raise ValueError('Nombres repetidos o invalidos')
    for obj in elements:
        if obj.get('color') not in COLORS:
            raise ValueError(f"Color desconocido: {obj.get('color')}")
        if not isinstance(obj.get('xy_m'), list) or len(obj['xy_m']) != 2:
            raise ValueError(f"Coordenadas incorrectas para {obj['name']}")
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in obj['xy_m']):
            raise ValueError(f"Coordenadas no finitas para {obj['name']}")
    return data


def _trash_model(obj):
    name = escape(obj['name'], {'"': '&quot;'})
    x, y = obj['xy_m']
    color, ixx, iyy, izz = COLORS[obj['color']]
    return f'''\n    <model name="{name}">
      <pose>{x:.6f} {y:.6f} 0.100 0 0 0</pose>
      <link name="link">
        <inertial><mass>0.3</mass><inertia>
          <ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia></inertial>
        <collision name="c"><geometry><box><size>0.12 0.12 0.20</size></box></geometry>
          <surface><friction><ode><mu>0.4</mu><mu2>0.4</mu2></ode></friction></surface>
        </collision>
        <visual name="v"><geometry><box><size>0.12 0.12 0.20</size></box></geometry>
          <material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse></material>
        </visual>
      </link>
    </model>'''


def _deposit_model(obj):
    name = escape(obj['name'], {'"': '&quot;'})
    x, y = obj['xy_m']
    color = COLORS[obj['color']][0]
    return f'''\n    <model name="{name}">
      <static>true</static>
      <pose>{x:.6f} {y:.6f} 0.010 0 0 0</pose>
      <link name="link">
        <visual name="v"><geometry><cylinder><radius>0.5</radius><length>0.02</length></cylinder></geometry>
          <material><ambient>{color} 0.45</ambient><diffuse>{color} 0.45</diffuse></material>
        </visual>
      </link>
    </model>'''


def make_world(scenario_path: Path, world_path: Path, output_path: Path):
    data = validate_scenario(scenario_path)
    world = world_path.read_text(encoding='utf-8')
    if world.count('</world>') != 1:
        raise ValueError('El mundo SDF debe contener un unico cierre </world>')
    root = ET.fromstring(world)
    worlds = root.findall('world')
    if len(worlds) != 1:
        raise ValueError('El SDF debe contener exactamente un <world>')
    existing = {m.get('name') for m in worlds[0].findall('model')}
    overlaps = existing.intersection(obj['name'] for obj in data['elements'])
    if overlaps:
        raise ValueError(f'Modelos duplicados en mundo: {sorted(overlaps)}')
    injection = ''.join(
        _trash_model(e) if e['type'] == 'trash' else _deposit_model(e)
        for e in data['elements']
    )
    head, separator, tail = world.rpartition('</world>')
    result = head + injection + '\n  ' + separator + tail
    ET.fromstring(result)  # comprobacion de XML bien formado
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if output_path.resolve() == world_path.resolve():
        raise ValueError('La salida no puede sobrescribir el mundo original')
    output_path.write_text(result, encoding='utf-8')
    trash_names = [e['name'] for e in data['elements'] if e['type'] == 'trash']
    print('Mundo experimental:', output_path)
    print('Mundo original intacto:', world_path)
    print('Basuras dinamicas:', len(trash_names), '| depositos:', 3)
    print('trash_list para DetachableJoint:')
    print(' '.join(trash_names))
    print('IMPORTANTE: todavia NO se lanza Gazebo ni se activan los joints.')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scenario', type=Path, required=True)
    ap.add_argument('--world', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    make_world(args.scenario, args.world, args.output)


if __name__ == '__main__':
    main()
