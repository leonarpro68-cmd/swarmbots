#!/usr/bin/env python3
"""Generar escenario + paradas A* + costos + MILP, sin ejecutar Gazebo.

Instalación: scripts/supermarket_milp/run_offline_seed.py
Uso: python3 scripts/supermarket_milp/run_offline_seed.py --seed 102
"""
from __future__ import annotations
import argparse
import json
import math
import subprocess
import sys
from pathlib import Path


def run(*argv):
    print('\n$ ' + ' '.join(map(str, argv)), flush=True)
    subprocess.run([str(a) for a in argv], check=True)


def load(path):
    return json.loads(path.read_text(encoding='utf-8'))


def dump(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def choose_stands(scenario, graph_data):
    # Elegir exclusivamente vecinos conectados por una arista real del grafo.
    # Se requiere 1.0 <= distancia al centro <= 2.1 metros, como stand-off
    # preliminar; no certifica separación del robot/deposito ni maniobras D8.
    nodes = {(round(float(n['x']), 5), round(float(n['y']), 5)) for n in graph_data['nodes']}
    adj = {p: set() for p in nodes}
    for u,v in graph_data['edges']:
        u = tuple(map(float,u)); v = tuple(map(float,v))
        adj[u].add(v); adj[v].add(u)
    used = set()
    trash = scenario['trash']
    stands = {}
    for deposit in sorted(scenario['deposits'], key=lambda d: d['color']):
        center = tuple(map(float, deposit['xy_m']))
        if center not in adj:
            raise ValueError(f"Deposito {deposit['color']} fuera del grafo")
        related = [t for t in trash if t['color'] == deposit['color']]
        if not related:
            raise ValueError(f"No hay basura del color {deposit['color']}")
        cx = sum(float(t['approach_node_xy_m'][0]) for t in related) / len(related)
        cy = sum(float(t['approach_node_xy_m'][1]) for t in related) / len(related)
        forbidden = {tuple(map(float,r['xy_m'])) for r in scenario['robots']}
        forbidden |= {tuple(map(float,t['approach_node_xy_m'])) for t in trash}
        forbidden |= {tuple(map(float,d['xy_m'])) for d in scenario['deposits']}
        candidates = [p for p in adj[center] if 1.0 <= math.dist(p,center) <= 2.1
                      and p not in forbidden and p not in used]
        if not candidates:
            raise ValueError(f"Sin parada A* candidata para deposito {deposit['color']}")
        # Orden estable: hacia las basuritas del mismo color, sin inventar nodos.
        best = min(candidates, key=lambda p: (math.dist(p, (cx,cy)),p[0],p[1]))
        stands[deposit['color']] = list(best)
        used.add(best)
    return stands


def main():
    ap = argparse.ArgumentParser(description='Preparación offline: escenario, stands, A* y MILP')
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[2])
    ap.add_argument('--overwrite', action='store_true', help='Recalcular y sobrescribir archivos de esta semilla (usar con cuidado)')
    args = ap.parse_args()
    ws = args.workspace.expanduser().resolve()
    gen = ws/'scripts/supermarket_escenarios/generar_escenario.py'
    milp = ws/'scripts/supermarket_milp'
    graph = milp/'data/supermarket_grafo.json'
    out = milp/'results'/f'seed_{args.seed}'
    scenario_dir = ws/'scripts/supermarket_escenarios/resultados'/f'seed_{args.seed}'
    scenario_config = ws/'config/supermarket'/f'seed_{args.seed}.json'
    costs = out/'astar_costs.json'
    stands_path = out/'deposit_stands.json'
    assignment = out/'assignment_milp_one_color_per_round.json'
    for path in (gen, milp/'build_costs.py',milp/'solve_assignment.py',graph):
        if not path.is_file():
            ap.error(f'Falta archivo requerido: {path}')
    # La ejecución por defecto NO sobrescribe una asignación existente.
    if assignment.exists() and not args.overwrite:
        ap.error(f'Ya existe {assignment}. No se sobrescribe. Para recalcular explícitamente usa --overwrite.')
    scenario_generated = scenario_dir/'scenario.json'
    if not scenario_generated.exists() or args.overwrite:
        run(sys.executable, gen, '--seed', args.seed, '--output-dir', scenario_dir)
    if not scenario_generated.is_file():
        raise RuntimeError(f'Generador no produjo {scenario_generated}')
    generated = load(scenario_generated)
    if generated.get('seed') != args.seed:
        raise ValueError(f'Semilla discordante en escenario: {generated.get("seed")}')
    if scenario_config.exists() and load(scenario_config) != generated and not args.overwrite:
        raise ValueError(f'{scenario_config} difiere del escenario generado; no se sobrescribe. Revisa archivos.')
    dump(scenario_config,generated)
    out.mkdir(parents=True,exist_ok=True)
    stands = choose_stands(generated, load(graph))
    if stands_path.exists() and load(stands_path) != stands and not args.overwrite:
        raise ValueError('Las paradas calculadas difieren de las existentes; no se sobrescriben.')
    dump(stands_path,stands)
    print('STANDS_ASTAR_PRELIMINARES: '+json.dumps(stands,ensure_ascii=False),flush=True)
    run(sys.executable,milp/'build_costs.py','--scenario',scenario_config,
        '--graph',graph,'--stands',stands_path,'--output',costs)
    run(sys.executable,milp/'solve_assignment.py','--costs',costs,
        '--one-color-per-round','--output',assignment)
    print('\nOFFLINE_COMPLETADO',flush=True)
    print('ESCENARIO:',scenario_config,flush=True)
    print('RESULTADOS:',out,flush=True)
    print('NOTA: no se han generado ni certificado CBS/Bezier/V3/D8.',flush=True)

if __name__ == '__main__':
    main()
