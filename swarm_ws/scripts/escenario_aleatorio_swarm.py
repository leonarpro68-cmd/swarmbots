#!/usr/bin/env python3
"""Genera escenarios reproducibles compatibles con objetos, grafo y CBS existentes."""
from __future__ import annotations
import argparse
import json
import math
import random
from pathlib import Path

import grafo_cbs_125 as geom
from colocar_basuras_depositos import ELEMENTS, Element


def build(seed: int, attempts: int = 2000):
    vertices, edges, _, _ = geom.make_graph()
    ordered = sorted(vertices)
    neighbors = {v: set() for v in vertices}
    for a, b in edges:
        neighbors[a].add(b)
        neighbors[b].add(a)
    rng = random.Random(seed)
    for attempt in range(attempts):
        # Dejar margen junto a paredes/columnas y múltiples accesos para entregas.
        pool = [v for v in ordered if len(neighbors[v]) >= 6 and 2 <= v[0] <= 20 and 2 <= v[1] <= 36]
        rng.shuffle(pool)
        chosen = []
        for v in pool:
            p = geom.node_position(v)
            if all(math.dist(p, geom.node_position(w)) >= 5.0 for w in chosen):
                chosen.append(v)
                if len(chosen) == 12:
                    break
        if len(chosen) < 12:
            continue
        elements = tuple(Element(e.name, e.color, v, e.kind) for e, v in zip(ELEMENTS, chosen[:9]))
        starts = tuple(chosen[9:])
        # Verificar las dos rondas usando las funciones reales del planificador.
        try:
            from grafo_basuras_dinamico import DynamicTrashGraph
            from cbs_primera_recogida_ros import plan
            from cbs_entrega_depositos_ros import plan_delivery
            import grafo_basuras_dinamico as dyn
            previous = dyn.ELEMENTS
            try:
                dyn.ELEMENTS = elements
                graph = DynamicTrashGraph()
                current = starts
                for _ in range(2):
                    assignments, goals, result = plan(current, graph, max_ct=1500, max_time=90)
                    for name, _distance in assignments:
                        graph.collect(name)
                    _options, delivery = plan_delivery(
                        graph, tuple(path[-1] for path in result['paths']),
                        [name for name, _ in assignments], max_ct=1500, max_time=90)
                    current = tuple(path[-1] for path in delivery['paths'])
                if graph.pending:
                    raise ValueError('Quedaron basuras')
            finally:
                dyn.ELEMENTS = previous
        except (ValueError, RuntimeError, KeyError, IndexError):
            continue
        return {
            'version': 1, 'seed': seed, 'attempt': attempt + 1,
            'grid_step_m': geom.STEP_M,
            'elements': [{'name': e.name, 'type': e.kind, 'color': e.color,
                          'grid': list(e.node), 'xy_m': list(geom.node_position(e.node))}
                         for e in elements],
            'robots': [{'name': f'summit{i}', 'grid': list(v), 'xy_m': list(geom.node_position(v))}
                       for i, v in enumerate(starts)],
        }
    raise RuntimeError(f'No se logró generar un escenario viable en {attempts} intentos')


def load_scenario(path):
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    if data.get('version') != 1:
        raise ValueError('Versión de escenario incorrecta')
    catalog = {e.name: e for e in ELEMENTS}
    entries = data['elements']
    if len(entries) != 9 or {x['name'] for x in entries} != set(catalog):
        raise ValueError('El escenario debe contener exactamente los nueve objetos esperados')
    elements = []
    valid, _, _, _ = geom.make_graph()
    for item in entries:
        original = catalog[item['name']]
        if item['color'] != original.color or item['type'] != original.kind:
            raise ValueError('Color o tipo incompatible')
        node = tuple(item['grid'])
        if node not in valid:
            raise ValueError(f'Objeto fuera del grafo: {item["name"]}')
        if math.dist(item['xy_m'], geom.node_position(node)) > 1e-6:
            raise ValueError('Desacuerdo entre coordenadas y vértice')
        elements.append(Element(original.name, original.color, node, original.kind))
    from colocar_basuras_depositos import validate
    validate(tuple(elements))
    robots = data['robots']
    if len(robots) != 3 or {r['name'] for r in robots} != {'summit0', 'summit1', 'summit2'}:
        raise ValueError('Se requieren tres robots')
    for r in robots:
        v = tuple(r['grid'])
        if v not in valid or math.dist(r['xy_m'], geom.node_position(v)) > 1e-6:
            raise ValueError('Inicio de robot inválido')
        if any(math.dist(r['xy_m'], geom.node_position(e.node)) < 3.0 for e in elements):
            raise ValueError('Robot demasiado cerca de objeto')
    return data, tuple(elements)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--output', default='/tmp/swarm_escenario.json')
    args = ap.parse_args()
    data = build(args.seed)
    dest = Path(args.output)
    dest.write_text(json.dumps(data, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print('Escenario:', dest, '| semilla:', args.seed, '| intentos:', data['attempt'])
    for r in data['robots']:
        print(r['name'], 'inicio:', r['xy_m'], 'vértice:', r['grid'])
    for e in data['elements']:
        print(e['name'], e['xy_m'], 'vértice:', e['grid'])
    print('Planificación offline de dos rondas: OK')


if __name__ == '__main__':
    main()
