#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import random
import math
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Rectangle

from swarm_behavior.swarm_planning import geometry as geom
from swarm_behavior.swarm_planning.bridge import PlanningBridge
from swarm_behavior.swarm_planning.scenario import load as load_scenario

ROBOT_COLORS = ['tab:blue', 'tab:green', 'tab:red']
OBJECT_COLORS = {
    'verde': '#22c55e',
    'naranja': '#f97316',
    'azul': '#2563eb',
}
def gazebo_robot_positions(seed, n=3, center_x=0.0,
                           center_y=0.0, area_half=8.0):
    """Reproduce las posiciones iniciales de sim_summit.launch.py."""
    rng = random.Random(seed + 20011)

    xmin = center_x - area_half
    xmax = center_x + area_half
    ymin = center_y - area_half
    ymax = center_y + area_half

    positions = []
    attempts = 0

    while len(positions) < n and attempts < n * 300 + 100:
        attempts += 1
        x = rng.uniform(xmin, xmax)
        y = rng.uniform(ymin, ymax)

        if any(math.hypot(x - px, y - py) < 2.0
               for px, py in positions):
            continue

        positions.append((x, y))

    if len(positions) != n:
        raise RuntimeError("No se pudieron generar tres posiciones.")

    return positions

def positions_to_grid(positions, graph):
    """Proyección al vértice transitable más cercano."""
    available = sorted(graph.vertices)
    selected = []

    for x, y in positions:
        candidates = [
            v for v in available if v not in selected
        ]

        if not candidates:
            raise RuntimeError("No quedan vértices disponibles.")

        nearest = min(
            candidates,
            key=lambda v: math.dist(
                (x, y), geom.node_position(v)
            )
        )

        selected.append(nearest)

    return tuple(selected)

TRASH_MARKER_SIZE = 140
DEPOSIT_MARKER_SIZE = 220
START_MARKER_SIZE = 90
END_MARKER_SIZE = 120


def nodes_to_xy(path):
    pts = [geom.node_position(v) for v in path]
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return xs, ys


def draw_background(ax, show_graph=False):
    ax.add_patch(
        Rectangle(
            (geom.X_MIN, geom.Y_MIN),
            geom.X_MAX - geom.X_MIN,
            geom.Y_MAX - geom.Y_MIN,
            fill=False,
            edgecolor='black',
            linewidth=1.8,
            zorder=1,
        )
    )

    if show_graph:
        valid, edges, _, _ = geom.make_graph()
        for a, b in edges:
            pa = geom.node_position(a)
            pb = geom.node_position(b)
            ax.plot(
                [pa[0], pb[0]],
                [pa[1], pb[1]],
                color='#cbd5e1',
                linewidth=0.5,
                alpha=0.35,
                zorder=0,
            )

    for cx, cy in geom.COLUMNS_XY:
        ax.add_patch(
            Circle(
                (cx, cy),
                geom.CLEARANCE_COLUMN_M,
                fill=False,
                linestyle='--',
                linewidth=1.0,
                edgecolor='#f59e0b',
                alpha=0.8,
                zorder=2,
            )
        )
        ax.add_patch(
            Circle(
                (cx, cy),
                geom.COLUMN_RADIUS_M,
                color='#8b5a2b',
                zorder=3,
            )
        )

    ax.set_xlim(geom.X_MIN - 0.8, geom.X_MAX + 0.8)
    ax.set_ylim(geom.Y_MIN - 0.8, geom.Y_MAX + 0.8)
    ax.set_aspect('equal')
    ax.grid(alpha=0.18)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')


def draw_objects(ax, scenario, pending, just_picked=(), already_removed=(), delivered=()):
    """
    pending: basuras que todavía están en el mapa en esta etapa
    just_picked: basuras recogidas en la ronda actual (se marcan distinto)
    already_removed: basuras recogidas en rondas anteriores
    delivered: opcional, solo para anotación visual
    """
    for e in scenario.elements:
        x, y = geom.node_position(e.node)
        color = OBJECT_COLORS.get(e.color, 'gray')

        if e.kind == 'deposit':
            ax.scatter(
                [x], [y],
                s=DEPOSIT_MARKER_SIZE,
                marker='s',
                color=color,
                edgecolors='black',
                linewidths=1.1,
                zorder=7,
            )
            ax.text(
                x + 0.25, y + 0.25, e.name,
                fontsize=8, color=color, weight='bold', zorder=8
            )
        else:
            if e.name in pending:
                ax.scatter(
                    [x], [y],
                    s=TRASH_MARKER_SIZE,
                    marker='o',
                    color=color,
                    edgecolors='black',
                    linewidths=1.0,
                    zorder=7,
                )
            elif e.name in just_picked:
                ax.scatter(
                    [x], [y],
                    s=TRASH_MARKER_SIZE,
                    marker='o',
                    facecolors='none',
                    edgecolors=color,
                    linewidths=2.0,
                    zorder=7,
                )
            elif e.name in already_removed or e.name in delivered:
                ax.scatter(
                    [x], [y],
                    s=TRASH_MARKER_SIZE * 0.85,
                    marker='x',
                    color=color,
                    linewidths=2.0,
                    zorder=7,
                )
            else:
                ax.scatter(
                    [x], [y],
                    s=TRASH_MARKER_SIZE,
                    marker='o',
                    facecolors='none',
                    edgecolors='#94a3b8',
                    linewidths=1.2,
                    zorder=6,
                )

            ax.text(
                x + 0.25, y + 0.25, e.name,
                fontsize=8, color=color, zorder=8
            )


def draw_robot_paths(ax, paths, title_prefix='R'):
    for i, path in enumerate(paths):
        color = ROBOT_COLORS[i % len(ROBOT_COLORS)]
        xs, ys = nodes_to_xy(path)

        ax.plot(xs, ys, '-o', color=color, linewidth=2.2, markersize=3.5, zorder=10)
        ax.scatter([xs[0]], [ys[0]], marker='s', s=START_MARKER_SIZE, color=color, zorder=11)
        ax.scatter([xs[-1]], [ys[-1]], marker='*', s=END_MARKER_SIZE, color=color, zorder=12)

        ax.text(
            xs[0] + 0.15, ys[0] + 0.15,
            f'{title_prefix}{i}',
            fontsize=9,
            weight='bold',
            color=color,
            zorder=13
        )


def summary_lines(plan, prefix):
    rows = []
    if hasattr(plan, 'assigned'):
        for i, (name, dist) in enumerate(plan.assigned):
            rows.append(f'{prefix}R{i} → {name} (d≈{dist:.2f})')
    elif hasattr(plan, 'trash_names'):
        for i, name in enumerate(plan.trash_names):
            rows.append(f'{prefix}R{i} entrega {name}')
    return rows


def build_mission_plans(scenario, max_ct=20000, max_time=65):
    bridge = PlanningBridge(scenario)

    gazebo_positions = gazebo_robot_positions(scenario.seed)

    robot_starts = positions_to_grid(
        gazebo_positions,
        bridge.graph
    )

    print("Posiciones iniciales Gazebo:", gazebo_positions)
    print("Vértices iniciales CBS:", robot_starts)

    pending_before_r1 = set(bridge.pending)

    pickup1 = bridge.plan_pickup(
        robot_starts,
        max_ct=max_ct,
        max_time=max_time
    )

    picked1 = tuple(name for name, _ in pickup1.assigned)

    for name in picked1:
        bridge.confirm_collected(name)

    pending_after_pick1 = set(bridge.pending)
    starts_delivery1 = tuple(path[-1] for path in pickup1.paths)
    delivery1 = bridge.plan_delivery(starts_delivery1, picked1, max_ct=max_ct, max_time=max_time)
    starts_round2 = tuple(path[-1] for path in delivery1.paths)

    pending_before_r2 = set(bridge.pending)
    pickup2 = bridge.plan_pickup(starts_round2, max_ct=max_ct, max_time=max_time)
    picked2 = tuple(name for name, _ in pickup2.assigned)

    for name in picked2:
        bridge.confirm_collected(name)

    pending_after_pick2 = set(bridge.pending)
    starts_delivery2 = tuple(path[-1] for path in pickup2.paths)
    delivery2 = bridge.plan_delivery(starts_delivery2, picked2, max_ct=max_ct, max_time=max_time)

    if bridge.pending:
        raise RuntimeError(f'Aún quedaron basuras pendientes: {sorted(bridge.pending)}')

    return {
        'pickup1': pickup1,
        'delivery1': delivery1,
        'pickup2': pickup2,
        'delivery2': delivery2,
        'picked1': picked1,
        'picked2': picked2,
        'pending_before_r1': pending_before_r1,
        'pending_after_pick1': pending_after_pick1,
        'pending_before_r2': pending_before_r2,
        'pending_after_pick2': pending_after_pick2,
    }


def plot_all_in_one(scenario, plans, output, show_graph=False):
    fig, axes = plt.subplots(2, 2, figsize=(18, 24))
    axes = axes.ravel()

    # Panel 1: Recogida ronda 1
    ax = axes[0]
    draw_background(ax, show_graph=show_graph)
    draw_objects(
        ax, scenario,
        pending=plans['pending_before_r1'],
        just_picked=plans['picked1'],
        already_removed=(),
    )
    draw_robot_paths(ax, plans['pickup1'].paths)
    ax.set_title(
        f'Recogida · Ronda 1\nCBS cost={plans["pickup1"].cost} | CT expandidos={plans["pickup1"].expanded}',
        fontsize=12, weight='bold'
    )

    # Panel 2: Entrega ronda 1
    ax = axes[1]
    draw_background(ax, show_graph=show_graph)
    draw_objects(
        ax, scenario,
        pending=plans['pending_after_pick1'],
        just_picked=(),
        already_removed=plans['picked1'],
        delivered=plans['picked1'],
    )
    draw_robot_paths(ax, plans['delivery1'].paths)
    ax.set_title(
        f'Entrega · Ronda 1\nCBS cost={plans["delivery1"].cost} | CT expandidos={plans["delivery1"].expanded}',
        fontsize=12, weight='bold'
    )

    # Panel 3: Recogida ronda 2
    ax = axes[2]
    draw_background(ax, show_graph=show_graph)
    draw_objects(
        ax, scenario,
        pending=plans['pending_before_r2'],
        just_picked=plans['picked2'],
        already_removed=plans['picked1'],
    )
    draw_robot_paths(ax, plans['pickup2'].paths)
    ax.set_title(
        f'Recogida · Ronda 2\nCBS cost={plans["pickup2"].cost} | CT expandidos={plans["pickup2"].expanded}',
        fontsize=12, weight='bold'
    )

    # Panel 4: Entrega ronda 2
    ax = axes[3]
    draw_background(ax, show_graph=show_graph)
    draw_objects(
        ax, scenario,
        pending=plans['pending_after_pick2'],
        just_picked=(),
        already_removed=plans['picked1'] + plans['picked2'],
        delivered=plans['picked1'] + plans['picked2'],
    )
    draw_robot_paths(ax, plans['delivery2'].paths)
    ax.set_title(
        f'Entrega · Ronda 2\nCBS cost={plans["delivery2"].cost} | CT expandidos={plans["delivery2"].expanded}',
        fontsize=12, weight='bold'
    )

    # Leyenda global
    legend_handles = [
        Line2D([0], [0], color=ROBOT_COLORS[0], lw=2.2, marker='o', label='Ruta robot'),
        Line2D([0], [0], marker='s', color='w', markerfacecolor='black', markersize=8, label='Inicio'),
        Line2D([0], [0], marker='*', color='w', markerfacecolor='black', markersize=10, label='Fin'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='#64748b', markeredgecolor='black', markersize=9, label='Basura pendiente'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='none', markeredgecolor='#64748b', markersize=9, label='Basura recogida en esa ronda'),
        Line2D([0], [0], marker='x', color='#64748b', markersize=9, lw=0, label='Basura ya retirada'),
        Line2D([0], [0], marker='s', color='w', markerfacecolor='#64748b', markeredgecolor='black', markersize=9, label='Depósito'),
    ]
    fig.legend(
        handles=legend_handles,
        loc='upper center',
        ncol=4,
        frameon=True,
        bbox_to_anchor=(0.5, 0.985),
    )

    text_lines = []
    text_lines.append(f'Semilla: {scenario.seed} | Intento: {scenario.attempt}')
    text_lines.extend(summary_lines(plans['pickup1'], 'R1-'))
    text_lines.extend(summary_lines(plans['delivery1'], 'E1-'))
    text_lines.extend(summary_lines(plans['pickup2'], 'R2-'))
    text_lines.extend(summary_lines(plans['delivery2'], 'E2-'))

    fig.text(
        0.02, 0.02,
        '\n'.join(text_lines),
        fontsize=10,
        family='monospace',
        va='bottom',
    )

    fig.suptitle(
        'SwarmBots · Vista completa de la misión CBS en un solo PNG',
        fontsize=18,
        weight='bold',
        y=0.995,
    )

    fig.tight_layout(rect=[0, 0.05, 1, 0.955])

    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description='Genera un PNG único con toda la misión CBS en 4 paneles.')
    ap.add_argument('--scenario', required=True, help='JSON del escenario')
    ap.add_argument('--output', default='/tmp/swarm_mission_routes.png', help='PNG de salida')
    ap.add_argument('--max-ct', type=int, default=20000)
    ap.add_argument('--max-time', type=int, default=65)
    ap.add_argument('--show-graph', action='store_true', help='Dibujar aristas del grafo en segundo plano')
    args = ap.parse_args()

    scenario = load_scenario(args.scenario)
    plans = build_mission_plans(scenario, max_ct=args.max_ct, max_time=args.max_time)
    plot_all_in_one(scenario, plans, args.output, show_graph=args.show_graph)

    print(f'PNG generado: {args.output}')
    print(f'Semilla: {scenario.seed}')
    print('Ronda 1 recogida:', [name for name, _ in plans['pickup1'].assigned])
    print('Ronda 2 recogida:', [name for name, _ in plans['pickup2'].assigned])


if __name__ == '__main__':
    main()
