#!/usr/bin/env python3
"""Inserta 6 basuras y 3 depósitos fijos en Gazebo Harmonic.

Ejecutar dentro de swarm_dev con Gazebo abierto:
    python3 ~/swarm_ws/scripts/colocar_basuras_depositos.py --dry-run
    python3 ~/swarm_ws/scripts/colocar_basuras_depositos.py

No altera sim_summit.launch.py ni A*/CBS. Genera SDF por entidad y un
manifiesto JSON para el planificador de recolección de la próxima etapa.
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path

from grafo_cbs_125 import COLUMNS_XY, CLEARANCE_COLUMN_M, make_graph, node_position

COLORS = {
    'verde': (0.18, 0.72, 0.28),
    'naranja': (0.98, 0.46, 0.11),
    'azul': (0.12, 0.35, 0.94),
}

# Coordenadas de vértices (i,j) de nuestro grafo compartido de 1.25 m.
# Los primeros seis son zonas de basura bloqueables; los últimos tres,
# depósitos asociados al color. Se eligen alejados de columnas.
@dataclass(frozen=True)
class Element:
    name: str
    color: str
    node: tuple[int, int]
    kind: str  # 'trash' o 'deposit'

ELEMENTS = (
    Element('trash_V1_verde',   'verde',   (8, 12), 'trash'),
    Element('trash_V2_verde',   'verde',   (14, 12), 'trash'),
    Element('trash_N1_naranja', 'naranja', (8, 27), 'trash'),
    Element('trash_N2_naranja', 'naranja', (14, 27), 'trash'),
    Element('trash_A1_azul',    'azul',    (8, 32), 'trash'),
    Element('trash_A2_azul',    'azul',    (14, 7), 'trash'),
    Element('deposit_verde',   'verde',   (3, 10), 'deposit'),
    Element('deposit_naranja', 'naranja', (19, 30), 'deposit'),
    Element('deposit_azul',    'azul',    (11, 9), 'deposit'),
)


def model_sdf(element: Element) -> str:
    rgb = COLORS[element.color]
    rgb_text = ' '.join(f'{c:.3f}' for c in rgb)
    if element.kind == 'trash':
        # Caja visual pequeña, obstáculo físico local en Gazebo.
        # El bloqueo abstracto de vértices/aristas llegará en la etapa siguiente.
        geometry = '<box><size>0.12 0.12 0.20</size></box>'
        height = 0.10
        collision = f'<collision name="collision"><geometry>{geometry}</geometry></collision>'
        visual_geometry = geometry
        alpha = 1.0
    else:
        # Zona de entrega: sin colisión, solo marca visual en el suelo.
        visual_geometry = '<cylinder><radius>0.50</radius><length>0.02</length></cylinder>'
        collision = ''
        height = 0.01
        alpha = 0.65
    return f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{element.name}">
    <static>true</static>
    <link name="link">
      <pose>0 0 {height:.3f} 0 0 0</pose>
      {collision}
      <visual name="visual">
        <geometry>{visual_geometry}</geometry>
        <material>
          <ambient>{rgb_text} {alpha:.2f}</ambient>
          <diffuse>{rgb_text} {alpha:.2f}</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
'''


def validate(elements=ELEMENTS):
    valid, edges, _, _ = make_graph()
    assert len(elements) == 9 and len(set(e.name for e in elements)) == 9
    assert sum(e.kind == 'trash' for e in elements) == 6
    assert sum(e.kind == 'deposit' for e in elements) == 3
    assert {e.color for e in elements if e.kind == 'deposit'} == set(COLORS)
    adjacency = {v: set() for v in valid}
    for a, b in edges:
        adjacency[a].add(b)
        adjacency[b].add(a)
    for e in elements:
        assert e.node in valid, f'Vértice inválido: {e}'
        assert len(adjacency[e.node]) >= 3, f'Insuficientes vecinos libres: {e}'
    for i, a in enumerate(elements):
        for b in elements[i+1:]:
            assert math.dist(node_position(a.node), node_position(b.node)) >= 2.0, (a, b)
    return True


def build_manifest(elements=ELEMENTS):
    return {
        'version': 1,
        'grid_step_m': 1.25,
        'description': 'Posiciones fijas; los trash son obstáculos eliminables para la fase futura',
        'elements': [
            {'name': e.name, 'type': e.kind, 'color': e.color,
             'grid': list(e.node), 'xy_m': list(node_position(e.node)),
             'blocking': e.kind == 'trash'}
            for e in elements
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true', help='Genera y valida archivos sin insertar modelos')
    parser.add_argument('--output', default='/tmp/swarm_fixed_objects', help='Carpeta de SDF y manifiesto')
    args = parser.parse_args()
    validate()
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    manifest_path = out / 'objetos_fijos.json'
    manifest_path.write_text(json.dumps(build_manifest(), indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(f'Manifiesto: {manifest_path}', flush=True)
    for e in ELEMENTS:
        x, y = node_position(e.node)
        path = out / f'{e.name}.sdf'
        path.write_text(model_sdf(e), encoding='utf-8')
        cmd = ['ros2', 'run', 'ros_gz_sim', 'create', '-name', e.name,
               '-file', str(path), '-x', str(x), '-y', str(y), '-z', '0.0']
        print(f'{e.kind:7} {e.name:23} vértice={e.node} xy=({x:.2f}, {y:.2f})', flush=True)
        if args.dry_run:
            print('   ' + ' '.join(cmd), flush=True)
        else:
            completed = subprocess.run(cmd, capture_output=True, text=True, timeout=45)
            if completed.returncode != 0:
                raise SystemExit(f'Fallo al insertar {e.name}:\n{completed.stdout}\n{completed.stderr}')
            print('   ' + (completed.stdout.strip() or 'Solicitada inserción; comprobar en Gazebo'), flush=True)
    print('Finalizado. No vuelvas a ejecutar sin reiniciar mundo: los nombres se duplicarían.', flush=True)


if __name__ == '__main__':
    main()
