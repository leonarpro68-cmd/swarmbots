"""D.8.8 -- Temporary margin-compatible CBS graph for one planning attempt.

This module does not change the physical geometry, safety margins, or base map.
The graph is restored on exit, even if CBS or Hybrid V2 rejects the route.
"""
from contextlib import contextmanager
import math

from swarm_behavior.swarm_planning import geometry as grid_geometry


def segment_distance(a, b, center):
    return float(grid_geometry.segment_point_distance(a, b, center))


@contextmanager
def margin_compatible_graph(graph, obstacles, operational_margin_m):
    """Temporarily remove graph vertices and edges that violate V2 clearances.

    `obstacles` are the *exact* (center, exclusion radius) inputs passed to
    Hybrid V2, so physical inflation is not applied a second time.
    """
    margin = float(operational_margin_m)
    if not math.isfinite(margin) or margin < 0:
        raise ValueError('Margen invalido')
    circles = tuple(((float(c[0]), float(c[1])), float(r) + margin)
                    for c, r in obstacles)
    if any(not all(map(math.isfinite, (x, y, r))) or r <= 0
           for (x, y), r in circles):
        raise ValueError('Obstaculo invalido')
    previous_vertices, previous_edges, previous_neighbors = (
        graph.vertices, graph.edges, graph.neighbors)
    # Keep the same conservative tolerance used for route rejection.
    def clear(xy):
        return all(math.dist(xy, center) > radius + 1e-6
                   for center, radius in circles)
    try:
        safe_vertices = {v for v in previous_vertices if clear(grid_geometry.node_position(v))}
        safe_edges = []
        for a, b in previous_edges:
            if a not in safe_vertices or b not in safe_vertices:
                continue
            pa, pb = grid_geometry.node_position(a), grid_geometry.node_position(b)
            if all(segment_distance(pa, pb, c) > r + 1e-6 for c, r in circles):
                safe_edges.append((a, b))
        neighbors = {v: [] for v in safe_vertices}
        for a, b in safe_edges:
            neighbors[a].append(b)
            neighbors[b].append(a)
        # Retain deterministic CBS tie-breaking.
        for v in neighbors:
            neighbors[v].sort(key=lambda w: (
                abs(w[0] - v[0]) + abs(w[1] - v[1]) == 2,
                w[0], w[1]))
        graph.vertices, graph.edges, graph.neighbors = safe_vertices, safe_edges, neighbors
        yield {'removed_vertices': len(previous_vertices) - len(safe_vertices),
               'removed_edges': len(previous_edges) - len(safe_edges),
               'remaining_vertices': len(safe_vertices), 'remaining_edges': len(safe_edges)}
    finally:
        graph.vertices, graph.edges, graph.neighbors = (
            previous_vertices, previous_edges, previous_neighbors)
