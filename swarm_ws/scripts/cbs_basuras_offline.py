#!/usr/bin/env python3
"""Demo CBS sobre grafo dinámico: basuras bloqueadas y liberación lógica.

No mueve robots ni modifica Gazebo. Requiere los archivos originales
cbs_grafo_125.py, grafo_basuras_dinamico.py, colocar_basuras_depositos.py
 y grafo_cbs_125.py en el mismo directorio.
"""
from __future__ import annotations
import argparse
from pathlib import Path
import cbs_grafo_125 as cbs
from grafo_basuras_dinamico import DynamicTrashGraph

STARTS = ((8, 10), (14, 10), (11, 18))
GOALS = ((8, 14), (14, 14), (11, 22))
TARGET = 'trash_V1_verde'


def verify(result, graph, starts, goals):
    paths = result['paths']
    assert len(paths) == len(starts)
    assert cbs.first_conflict(paths) is None, 'CBS devolvió conflictos'
    for i, path in enumerate(paths):
        assert path[0] == starts[i] and path[-1] == goals[i]
        for v in path:
            assert v in graph.vertices, f'Robot {i} ocupa vértice bloqueado {v}'
        for a, b in zip(path, path[1:]):
            assert a == b or b in graph.neighbors[a], f'Arista bloqueada: {a}->{b}'
    return True


def run_demo(starts=STARTS, goals=GOALS, target=TARGET, *, max_ct=10000):
    graph = DynamicTrashGraph()
    assert target in graph.pending
    assert all(v in graph.vertices for v in starts + goals), 'Inicio/meta bloqueados'
    before = cbs.solve(starts, goals, graph=graph, max_ct=max_ct)
    verify(before, graph, starts, goals)
    before_pending = set(graph.pending)
    graph.collect(target)
    after = cbs.solve(starts, goals, graph=graph, max_ct=max_ct)
    verify(after, graph, starts, goals)
    return graph, before, after, before_pending


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', default='cbs_basuras_offline_registro.txt')
    args = parser.parse_args()
    graph, before, after, pending_before = run_demo()
    lines = ['CBS + grafo dinámico (prueba offline)',
             f'Basuras pendientes antes: {sorted(pending_before)}',
             f'Bloqueo liberado: {TARGET}',
             f'Basuras pendientes después: {sorted(graph.pending)}']
    for label, result in [('ANTES', before), ('DESPUES', after)]:
        print(f'{label}: costo={result["cost"]}, CT expandidos={result["expanded"]}, generados={result["generated"]}')
        lines.append(f'\n--- {label} ---')
        lines.extend(result['log'])
        for i, path in enumerate(result['paths']):
            print(f'  Robot {i}: {path}')
            lines.append(f'Robot {i}: {path}')
    Path(args.log).write_text('\n'.join(lines)+'\n', encoding='utf-8')
    released_vertex = graph.trash[TARGET].node
    new_goal_path = cbs.low_level(graph, 0, STARTS[0], released_vertex, frozenset())
    assert new_goal_path is not None
    print(f'Ruta ahora posible hasta vértice liberado {released_vertex}: {new_goal_path}')
    lines.append(f'Ruta posible al vértice liberado: {new_goal_path}')
    Path(args.log).write_text('\n'.join(lines)+'\n', encoding='utf-8')
    print(f'Registro: {args.log}')

if __name__ == '__main__':
    main()
