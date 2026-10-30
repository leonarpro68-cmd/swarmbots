#!/usr/bin/env python3
"""Construye y visualiza el grafo de vértices de 1.25 m del almacén.

Etapa preliminar a A* y CBS: NO lanza Gazebo, NO publica cmd_vel,
NO modifica otros archivos ROS 2.

Cada nodo está en un vértice del grid; cada arista permite un movimiento
horizontal, vertical o diagonal SOLO si el segmento completo está despejado.

Uso dentro de swarm_dev:
    python3 ~/swarm_ws/scripts/grafo_cbs_125.py \
        --png ~/swarm_ws/scripts/grafo_cbs_125.png \
        --json ~/swarm_ws/scripts/grafo_cbs_125.json

Requiere: matplotlib (solo para dibujar; la geometría usa librería estándar).
Datos de entrada: medidas manuales suministradas por el usuario (aproximadas).
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

# ----- Geometría del almacén en metros (coordenadas del mundo de Gazebo) -----
X_MIN, X_MAX = -14.0, 14.0
Y_MIN, Y_MAX = -24.0, 24.0
STEP_M = 1.25

# Región centrada de 22x38 CELDAS COMPLETAS (23x39 VÉRTICES).
# El rectángulo físico tiene 0.25 m de margen exterior por cada lado.
N_CELLS_X, N_CELLS_Y = 22, 38
ORIGIN_X = -(N_CELLS_X * STEP_M) / 2.0
ORIGIN_Y = -(N_CELLS_Y * STEP_M) / 2.0

# Huella conservadora basada en mesh del chasis + posiciones de ruedas.
ROBOT_LENGTH_M = 0.7190
ROBOT_WIDTH_M = 0.5786
ROBOT_RADIUS_M = math.hypot(ROBOT_LENGTH_M, ROBOT_WIDTH_M) / 2
EXTRA_MARGIN_M = 0.30
CLEARANCE_WALL_M = ROBOT_RADIUS_M + EXTRA_MARGIN_M

# Columnas circulares aproximadas. Son obstáculos FÍSICOS estáticos.
COLUMN_DIAMETER_M = 0.35
COLUMN_RADIUS_M = COLUMN_DIAMETER_M / 2
CLEARANCE_COLUMN_M = COLUMN_RADIUS_M + CLEARANCE_WALL_M
COLUMNS_XY = (
    (7.4, 7.5),
    (-7.4, -7.5),
    (-7.4, 7.5),
    (-7.4, -15.0),
    (7.4, 0.0),
    (7.4, -7.5),
    (-7.4, 15.0),
    (7.4, -15.0),
    (7.4, 15.0),
    (-7.4, 0.0),
)

Node = tuple[int, int]
Point = tuple[float, float]


def node_position(v: Node) -> Point:
    return (ORIGIN_X + v[0] * STEP_M, ORIGIN_Y + v[1] * STEP_M)


def valid_index(v: Node) -> bool:
    return 0 <= v[0] <= N_CELLS_X and 0 <= v[1] <= N_CELLS_Y


def free_position(p: Point) -> bool:
    """¿Cabe el centro del Summit en ese punto, con margen?"""
    x, y = p
    if not (math.isfinite(x) and math.isfinite(y)):
        return False
    if not (X_MIN + CLEARANCE_WALL_M <= x <= X_MAX - CLEARANCE_WALL_M):
        return False
    if not (Y_MIN + CLEARANCE_WALL_M <= y <= Y_MAX - CLEARANCE_WALL_M):
        return False
    return all(math.hypot(x - cx, y - cy) >= CLEARANCE_COLUMN_M
               for cx, cy in COLUMNS_XY)


def segment_point_distance(a: Point, b: Point, p: Point) -> float:
    """Distancia mínima del punto p al segmento cerrado a--b."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    denom = dx*dx + dy*dy
    if denom == 0.0:
        return math.hypot(p[0] - a[0], p[1] - a[1])
    alpha = min(1.0, max(0.0, ((p[0] - a[0])*dx + (p[1] - a[1])*dy) / denom))
    return math.hypot(p[0] - (a[0] + alpha*dx),
                      p[1] - (a[1] + alpha*dy))


def segment_clear(a: Point, b: Point) -> bool:
    """Valida toda la arista (NO solo sus extremos)."""
    if not (free_position(a) and free_position(b)):
        return False
    # La región libre de paredes es convexa, por lo que basta validar extremos.
    return all(segment_point_distance(a, b, col) >= CLEARANCE_COLUMN_M
               for col in COLUMNS_XY)


def make_graph():
    """Devuelve (nodos validos, aristas, aristas rechazadas entre nodos libres,
    nodos bloqueados); aristas no dirigidas con índices del grid.
    """
    nodes = [(i, j) for i in range(N_CELLS_X + 1)
                    for j in range(N_CELLS_Y + 1)]
    valid = {v for v in nodes if free_position(node_position(v))}
    blocked = set(nodes) - valid
    edges: list[tuple[Node, Node]] = []
    rejected: list[tuple[Node, Node, str]] = []
    for v in sorted(valid):
        i, j = v
        # Única dirección por arista no dirigida; incluye ambas diagonales.
        for di, dj in ((1, 0), (0, 1), (1, 1), (1, -1)):
            w = (i + di, j + dj)
            if w not in valid:
                continue
            if di and dj and ((i+di, j) not in valid or (i, j+dj) not in valid):
                rejected.append((v, w, 'esquina bloqueada'))
                continue
            if not segment_clear(node_position(v), node_position(w)):
                rejected.append((v, w, 'intersección con columna inflada'))
                continue
            edges.append((v, w))
    return valid, edges, rejected, blocked


def write_json(path: Path, valid, edges, rejected, blocked) -> None:
    data = {
        'description': 'Grafo geometrico estático; todavía sin tiempos ni CBS',
        'world_bounds_m': [X_MIN, X_MAX, Y_MIN, Y_MAX],
        'grid_step_m': STEP_M,
        'grid_origin_m': [ORIGIN_X, ORIGIN_Y],
        'cells': [N_CELLS_X, N_CELLS_Y],
        'robot_radius_m': round(ROBOT_RADIUS_M, 6),
        'safety_margin_m': EXTRA_MARGIN_M,
        'column_radius_m': COLUMN_RADIUS_M,
        'column_exclusion_radius_m': round(CLEARANCE_COLUMN_M, 6),
        'columns_xy': [list(col) for col in COLUMNS_XY],
        'nodes': [{'id': f'{i},{j}', 'grid': [i, j],
                   'xy_m': list(node_position((i,j)))} for i, j in sorted(valid)],
        'edges': [{'from': f'{a[0]},{a[1]}', 'to': f'{b[0]},{b[1]}',
                   'length_m': round(math.dist(node_position(a), node_position(b)), 6)}
                  for a, b in edges],
        'statistics': {'all_vertices': (N_CELLS_X+1)*(N_CELLS_Y+1),
                       'free_vertices': len(valid), 'blocked_vertices': len(blocked),
                       'valid_edges': len(edges),
                       'rejected_edges_with_free_endpoints': len(rejected)},
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')


def draw(path: Path, valid, edges, rejected, blocked) -> None:
    try:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, Rectangle
        from matplotlib.collections import LineCollection
    except ImportError as e:
        raise SystemExit('Falta matplotlib: instala python3-matplotlib en tu entorno') from e

    fig, (ax, zoom) = plt.subplots(1, 2, figsize=(13.6, 9.7),
                                   gridspec_kw={'width_ratios': [1.10, 1.0]})
    fig.suptitle('SwarmBots · grafo de 1.25 m para A* y CBS', fontsize=16, weight='bold')
    all_ax = (ax, zoom)
    line_segments = [[node_position(v), node_position(w)] for v, w in edges]
    rejected_segs = [[node_position(v), node_position(w)] for v, w, reason in rejected]
    valid_pts = [node_position(v) for v in sorted(valid)]
    blocked_pts = [node_position(v) for v in sorted(blocked)]

    for cur in all_ax:
        cur.add_patch(Rectangle((X_MIN, Y_MIN), X_MAX-X_MIN, Y_MAX-Y_MIN,
                                fill=False, edgecolor='#334155', linewidth=2.0,
                                label='Paredes (límite medido)'))
        cur.add_collection(LineCollection(line_segments, colors='#64748b',
                                          linewidths=0.55 if cur is ax else 1.1,
                                          alpha=0.42 if cur is ax else 0.56,
                                          zorder=1))
        for col in COLUMNS_XY:
            cur.add_patch(Circle(col, CLEARANCE_COLUMN_M, color='#D97706',
                                 alpha=0.13, zorder=2))
            cur.add_patch(Circle(col, CLEARANCE_COLUMN_M, fill=False,
                                 edgecolor='#EA580C', linewidth=1.2, linestyle='--', zorder=3))
            cur.add_patch(Circle(col, COLUMN_RADIUS_M, color='#92400E', zorder=5))
        if valid_pts:
            xs, ys = zip(*valid_pts)
            cur.scatter(xs, ys, s=7 if cur is ax else 25,
                        color='#0F766E', label='Vértice transitable', zorder=4)
        if blocked_pts:
            xs, ys = zip(*blocked_pts)
            cur.scatter(xs, ys, s=14 if cur is ax else 45, marker='x',
                        linewidths=1.0, color='#DC2626', label='Vértice prohibido', zorder=6)
        cur.set_aspect('equal')
        cur.set_xlabel('X mundo (m)')
        cur.set_ylabel('Y mundo (m)')
        cur.grid(which='major', alpha=0.16)

    ax.set_title('Almacén completo · 28 × 48 m')
    ax.set_xlim(-14.8, 14.8)
    ax.set_ylim(-24.9, 24.9)
    ax.set_xticks(range(-14, 15, 4))
    ax.set_yticks(range(-24, 25, 6))

    zoom.set_title('Detalle · columna en (7.4, 7.5) m')
    zoom.set_xlim(3.75, 11.1)
    zoom.set_ylim(3.8, 11.1)
    zoom.set_xticks([3.75, 5, 6.25, 7.5, 8.75, 10, 11.25])
    zoom.set_yticks([3.75, 5, 6.25, 7.5, 8.75, 10, 11.25])
    zoom.tick_params(axis='both', labelsize=8)
    if rejected_segs:
        zoom.add_collection(LineCollection(rejected_segs, colors='#DC2626',
                                           alpha=0.60, linewidths=1.3,
                                           linestyles='dotted', zorder=2))
    zoom.text(0.03, 0.02,
              'Marrón: columna real aproximada\n'
              'Naranja: radio de exclusión\n'
              'Verde: vértice libre\nRojo: vértice o conexión prohibidos',
              transform=zoom.transAxes, fontsize=9, va='bottom',
              bbox=dict(facecolor='white', alpha=0.86, edgecolor='#CBD5E1', pad=7))

    fig.text(0.5, 0.025,
             f'Vértices: {len(valid)} libres + {len(blocked)} bloqueados  |  '
             f'Aristas válidas: {len(edges)}  |  '
             f'Radio exclusión columna: {CLEARANCE_COLUMN_M:.3f} m  |  '
             f'Sin A* ni CBS todavía',
             ha='center', fontsize=10, color='#334155')
    fig.subplots_adjust(left=.07, right=.98, top=.90, bottom=.09, wspace=.24)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=170, facecolor='white')
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--png', type=Path, default=Path('grafo_cbs_125.png'))
    parser.add_argument('--json', type=Path, default=Path('grafo_cbs_125.json'))
    args = parser.parse_args()
    assert math.isclose(ORIGIN_X, -13.75) and math.isclose(ORIGIN_Y, -23.75)
    valid, edges, rejected, blocked = make_graph()
    write_json(args.json, valid, edges, rejected, blocked)
    draw(args.png, valid, edges, rejected, blocked)
    print(f'Grid: {N_CELLS_X}x{N_CELLS_Y} celdas, '
          f'{(N_CELLS_X+1)*(N_CELLS_Y+1)} vértices totales')
    print(f'Vértices transitables: {len(valid)}; bloqueados: {len(blocked)}')
    print(f'Aristas transitables: {len(edges)}; rechazadas entre libres: {len(rejected)}')
    print(f'Radio de exclusión (columnas): {CLEARANCE_COLUMN_M:.3f} m')
    print(f'Imagen guardada: {args.png.resolve()}')
    print(f'Grafo guardado: {args.json.resolve()}')


if __name__ == '__main__':
    main()
