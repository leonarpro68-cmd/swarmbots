#!/usr/bin/env python3
"""Genera offline las cuatro etapas CBS + refinamiento híbrido en un PNG.

Misma semilla y organización de paneles que plot_mission_routes.py.
No requiere ROS 2, Gazebo, odometría ni comunicación DDS.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

import plot_mission_routes as original
from swarm_behavior.swarm_planning.bridge import PlanningBridge
from swarm_behavior.swarm_planning.scenario import load as load_scenario
from swarm_behavior.swarm_planning.hybrid_refinement.adapter import from_cbs_paths
from swarm_behavior.swarm_planning.hybrid_refinement.corridors import build_hybrid_corridors
from swarm_behavior.swarm_planning.hybrid_refinement.bezier_optimizer import (
    evaluate_bezier, optimize_bezier, scale_trajectory_time,
)
from swarm_behavior.swarm_planning.hybrid_refinement.iterative_refinement import refine_iteratively

STAGES = ('pickup1', 'delivery1', 'pickup2', 'delivery2')
STAGE_LABELS = ('Recogida · ronda 1', 'Entrega · ronda 1',
                'Recogida · ronda 2', 'Entrega · ronda 2')


def make_stages(scenario, *, max_ct, max_time):
    """Reconstruye exactamente el mismo ciclo de la figura CBS original.

    Para cada etapa conserva el grafo dinámico antes de retirar el siguiente
    lote de basura. Así las restricciones son las propias de esa etapa.
    """
    bridge = PlanningBridge(scenario)
    positions = original.gazebo_robot_positions(scenario.seed)
    starts = original.positions_to_grid(positions, bridge.graph)
    print('Semilla:', scenario.seed, '| inicios CBS:', starts, flush=True)
    pending_r1 = set(bridge.pending)
    entries = {}

    def record(key, plan, pending, picked, removed):
        entries[key] = {
            'plan': plan,
            'graph': bridge.graph,
            'pending': set(pending),
            'picked': tuple(picked),
            'removed': tuple(removed),
        }

    p1 = bridge.plan_pickup(starts, max_ct=max_ct, max_time=max_time)
    picked1 = tuple(name for name, _ in p1.assigned)
    record('pickup1', p1, pending_r1, picked1, ())
    for name in picked1:
        bridge.confirm_collected(name)
    pending_after1 = set(bridge.pending)
    d1 = bridge.plan_delivery(tuple(p[-1] for p in p1.paths), picked1,
                              max_ct=max_ct, max_time=max_time)
    record('delivery1', d1, pending_after1, (), picked1)
    p2 = bridge.plan_pickup(tuple(p[-1] for p in d1.paths),
                             max_ct=max_ct, max_time=max_time)
    picked2 = tuple(name for name, _ in p2.assigned)
    record('pickup2', p2, pending_after1, picked2, picked1)
    for name in picked2:
        bridge.confirm_collected(name)
    pending_after2 = set(bridge.pending)
    d2 = bridge.plan_delivery(tuple(p[-1] for p in p2.paths), picked2,
                              max_ct=max_ct, max_time=max_time)
    record('delivery2', d2, pending_after2, (), picked1 + picked2)
    if bridge.pending:
        raise RuntimeError(f'Basuras pendientes: {sorted(bridge.pending)}')
    return entries


def optimize_stage(entry, args):
    schedule = from_cbs_paths(entry['plan'].paths, graph=entry['graph'],
                              max_speed_m_s=args.max_speed)
    corridors = build_hybrid_corridors(schedule, graph=entry['graph'])
    initial = optimize_bezier(schedule, corridors, maxiter=args.maxiter)
    if not all(robot.solver_success for robot in initial.robots):
        raise RuntimeError('SLSQP no convergió en optimización inicial')
    result = refine_iteratively(schedule, corridors,
                                initial_trajectory=initial, iterations=args.iterations,
                                samples_per_piece=args.samples,
                                maxiter=args.maxiter,
                                max_speed_m_s=args.max_speed,
                                max_acceleration_m_s2=args.max_accel)
    trajectory = result.final_trajectory
    if not all(robot.solver_success for robot in trajectory.robots):
        raise RuntimeError('SLSQP no convergió tras el refinamiento')
    scaled = scale_trajectory_time(trajectory, max_speed_m_s=args.max_speed,
                                   max_acceleration_m_s2=args.max_accel,
                                   safety_factor=args.time_safety_factor)
    return schedule, result, scaled


def sample_robot(robot, n=20):
    xy = []
    for piece in robot.control_points:
        xy.extend(evaluate_bezier(piece, u / n) for u in range(n))
    xy.append(evaluate_bezier(robot.control_points[-1], 1))
    return np.asarray(xy)


def draw_stage(ax, scenario, key, entry, schedule, result, scaled, show_graph):
    original.draw_background(ax, show_graph=show_graph)
    original.draw_objects(ax, scenario, pending=entry['pending'],
                          just_picked=entry['picked'], already_removed=entry['removed'],
                          delivered=entry['removed'])
    for idx, robot in enumerate(result.final_trajectory.robots):
        color = original.ROBOT_COLORS[idx % len(original.ROBOT_COLORS)]
        discrete = np.asarray(schedule.metric_paths[idx])
        smooth = sample_robot(robot)
        ax.plot(discrete[:, 0], discrete[:, 1], ':o', color=color,
                alpha=0.42, linewidth=1.15, markersize=2, zorder=9)
        ax.plot(smooth[:, 0], smooth[:, 1], '-', color=color,
                linewidth=2.4, zorder=10)
        ax.scatter(*discrete[0], marker='s', s=90, color=color, zorder=11)
        ax.scatter(*discrete[-1], marker='*', s=140, color=color, zorder=12)
        ax.text(discrete[0,0]+.12, discrete[0,1]+.15, f'R{idx}',
                color=color, fontweight='bold', fontsize=9)
    cost = sum(r.objective_acceleration_sq for r in result.final_trajectory.robots)
    ax.set_title(f'{STAGE_LABELS[STAGES.index(key)]}\n'
                 f'CBS={entry["plan"].cost} · Bézier J={cost:.3f} · T={scaled.duration_s:.1f} s',
                 fontsize=11, fontweight='bold')


def generate_png(scenario, entries, computed, path, *, show_graph=False):
    selected = tuple(key for key in STAGES if key in computed)
    if len(selected) == 1:
        fig, ax = plt.subplots(figsize=(11, 10))
        axes = (ax,)
    else:
        fig, grid = plt.subplots(2, 2, figsize=(18, 23))
        axes = tuple(grid.flat)
    for ax, key in zip(axes, selected):
        schedule, result, scaled = computed[key]
        draw_stage(ax, scenario, key, entries[key], schedule, result, scaled, show_graph)
    for ax in axes[len(selected):]:
        ax.axis('off')
    fig.legend(handles=[
        Line2D([], [], color='black', ls=':', marker='o', label='CBS discreto'),
        Line2D([], [], color='black', lw=2.5, label='Bézier refinado'),
        Line2D([], [], marker='s', color='black', ls='', label='Inicio'),
        Line2D([], [], marker='*', color='black', ls='', label='Meta'),
    ], loc='upper center', ncol=4, bbox_to_anchor=(.5,.981))
    fig.suptitle(f'SwarmBots · Misión CBS + postprocesamiento híbrido · semilla {scenario.seed}',
                 fontsize=17, fontweight='bold', y=.997)
    fig.tight_layout(rect=(0, .015, 1, .955))
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=165)
    plt.close(fig)


def execute(scenario, args):
    entries = make_stages(scenario, max_ct=args.max_ct, max_time=args.max_time)
    computed = {}
    report = {'seed': scenario.seed, 'note': 'Planificación offline; no demuestra seguimiento en Gazebo',
              'parameters': {'iterations': args.iterations, 'max_speed_m_s': args.max_speed,
                             'max_accel_m_s2': args.max_accel}, 'stages': {}}
    selected = STAGES if args.stage == 'all' else (args.stage,)
    for key in selected:
        print(f'Optimización híbrida: {key}...', flush=True)
        schedule, result, scaled = optimize_stage(entries[key], args)
        computed[key] = (schedule, result, scaled)
        report['stages'][key] = {
            'cbs_cost': entries[key]['plan'].cost,
            'schedule': schedule.to_dict(),
            'refinement': result.to_dict(),
            'scaled': scaled.to_dict(),
        }
        print(f'  piezas={len(result.final_trajectory.robots[0].control_points)}, '
              f'iteraciones={result.iterations_completed}, T={scaled.duration_s:.2f}s', flush=True)
    # No escribir outputs parciales si alguna etapa falla.
    generate_png(scenario, entries, computed, args.output, show_graph=args.show_graph)
    if args.json_output:
        out = Path(args.json_output)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    print(f'PNG generado: {args.output}')
    if args.json_output:
        print(f'JSON generado: {args.json_output}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--seed', type=int)
    source.add_argument('--scenario')
    parser.add_argument('--output', default='/tmp/mision_hibrida.png')
    parser.add_argument('--json-output')
    parser.add_argument('--stage', choices=('all',)+STAGES, default='all',
                        help='Una sola etapa para prueba rápida, o las cuatro (por defecto)')
    parser.add_argument('--iterations', type=int, default=2)
    parser.add_argument('--samples', type=int, default=32)
    parser.add_argument('--maxiter', type=int, default=180)
    parser.add_argument('--max-speed', type=float, default=.60)
    parser.add_argument('--max-accel', type=float, default=.50)
    parser.add_argument('--time-safety-factor', type=float, default=1.08)
    parser.add_argument('--max-ct', type=int, default=20000)
    parser.add_argument('--max-time', type=int, default=65)
    parser.add_argument('--show-graph', action='store_true')
    args = parser.parse_args()
    if args.seed is not None and args.seed < 0:
        parser.error('--seed no puede ser negativo')
    if not (0 <= args.iterations <= 5 and args.samples >= 4 and args.maxiter >= 1
            and 0 < args.max_speed <= 1.20 and 0 < args.max_accel <= 1.00
            and args.time_safety_factor >= 1.0):
        parser.error('Parámetros de optimización fuera del rango de esta demo')
    if args.scenario:
        scenario = load_scenario(args.scenario)
        execute(scenario, args)
    else:
        generator = Path(__file__).resolve().with_name('escenario_aleatorio_swarm.py')
        if not generator.is_file():
            parser.error(f'Generador de escenarios no encontrado: {generator}')
        with tempfile.TemporaryDirectory(prefix='hybrid_plot_') as temp:
            scenario_file = Path(temp) / 'scenario.json'
            subprocess.run([sys.executable, str(generator), '--seed', str(args.seed),
                            '--output', str(scenario_file)], check=True)
            scenario = load_scenario(scenario_file)
            if scenario.seed != args.seed:
                raise RuntimeError('La semilla generada no coincide con la solicitada')
            execute(scenario, args)


if __name__ == '__main__':
    main()
