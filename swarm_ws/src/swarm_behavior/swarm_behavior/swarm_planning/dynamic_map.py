#!/usr/bin/env python3
"""Grafo de 1.25 m con obstáculos temporales de basura (etapa offline).

No requiere ROS ni Gazebo. Reutiliza grafo_cbs_125 y las posiciones fijas
existentes. Las basuras impiden ocupar su vértice y atravesar su zona
inflada; al recoger una basura, se reconstruyen los nodos y aristas libres.
Los depósitos son marcas visuales, no obstáculos.

Uso: python3 grafo_basuras_dinamico.py
"""
from __future__ import annotations
import heapq
import math
from itertools import count

from . import geometry as geom
from .objects import ELEMENTS

# La caja 0.12 x 0.12 m se aproxima por un círculo circunscrito.
TRASH_RADIUS_M = math.hypot(0.12, 0.12) / 2.0
TRASH_EXCLUSION_M = geom.CLEARANCE_WALL_M + TRASH_RADIUS_M

class DynamicTrashGraph:
    def __init__(self, elements=None):
        self.elements = tuple(ELEMENTS if elements is None else elements)
        self.static_vertices, self.static_edges, _, _ = geom.make_graph()
        self.trash = {e.name: e for e in self.elements if e.kind == 'trash'}
        self.deposits = {e.name: e for e in self.elements if e.kind == 'deposit'}
        self.pending = set(self.trash)
        self.vertices: set[geom.Node] = set()
        self.edges: list[tuple[geom.Node, geom.Node]] = []
        self.neighbors: dict[geom.Node, list[geom.Node]] = {}
        self.rebuild()

    def active_positions(self):
        return tuple(geom.node_position(self.trash[name].node) for name in sorted(self.pending))

    def free_position(self, p: geom.Point) -> bool:
        return geom.free_position(p) and all(
            math.dist(p, q) >= TRASH_EXCLUSION_M - 1e-9
            for q in self.active_positions()
        )

    def segment_clear(self, a: geom.Point, b: geom.Point) -> bool:
        if not geom.segment_clear(a, b) or not self.free_position(a) or not self.free_position(b):
            return False
        return all(
            geom.segment_point_distance(a, b, q) >= TRASH_EXCLUSION_M - 1e-9
            for q in self.active_positions()
        )

    def rebuild(self):
        self.vertices = {v for v in self.static_vertices if self.free_position(geom.node_position(v))}
        self.edges = [(a, b) for a, b in self.static_edges
                      if a in self.vertices and b in self.vertices
                      and self.segment_clear(geom.node_position(a), geom.node_position(b))]
        self.neighbors = {v: [] for v in self.vertices}
        for a, b in self.edges:
            self.neighbors[a].append(b)
            self.neighbors[b].append(a)
        for neighbors in self.neighbors.values():
            neighbors.sort()

    def collect(self, name: str):
        """Actualiza el grafo lógico; NO elimina automáticamente modelos en Gazebo."""
        if name not in self.trash:
            raise ValueError(f'Basura desconocida: {name}')
        if name not in self.pending:
            raise ValueError(f'Basura ya recogida: {name}')
        self.pending.remove(name)
        self.rebuild()

    def astar(self, start: geom.Node, goal: geom.Node) -> tuple[geom.Node, ...] | None:
        """A* espacial con costo métrico: sirve para probar bloqueos y liberación."""
        if start not in self.vertices or goal not in self.vertices:
            return None
        serial = count()
        pq = [(math.dist(geom.node_position(start), geom.node_position(goal)), 0.0,
               next(serial), start)]
        best = {start: 0.0}
        parent = {}
        while pq:
            _, g_cost, _, v = heapq.heappop(pq)
            if g_cost > best[v] + 1e-9:
                continue
            if v == goal:
                route = [v]
                while v in parent:
                    v = parent[v]
                    route.append(v)
                return tuple(reversed(route))
            for w in self.neighbors[v]:
                ng = g_cost + math.dist(geom.node_position(v), geom.node_position(w))
                if ng + 1e-9 < best.get(w, math.inf):
                    best[w] = ng
                    parent[w] = v
                    h = math.dist(geom.node_position(w), geom.node_position(goal))
                    heapq.heappush(pq, (ng + h, ng, next(serial), w))
        return None


def main():
    graph = DynamicTrashGraph()
    name = 'trash_V1_verde'
    start, goal = (8, 10), (8, 14)
    print(f'Grafo base: {len(graph.static_vertices)} vértices; {len(graph.static_edges)} aristas')
    print(f'Basuras pendientes: {len(graph.pending)}')
    print(f'Grafo bloqueado: {len(graph.vertices)} vértices; {len(graph.edges)} aristas')
    print(f'Vértice de {name}: {graph.trash[name].node}; transitable: {graph.trash[name].node in graph.vertices}')
    before = graph.astar(start, goal)
    print('Ruta ANTES:', before)
    graph.collect(name)
    print(f'Basura recogida: {name}')
    print(f'Grafo actualizado: {len(graph.vertices)} vértices; {len(graph.edges)} aristas')
    print(f'Vértice liberado: {graph.trash[name].node in graph.vertices}')
    after = graph.astar(start, goal)
    print('Ruta DESPUÉS:', after)
    assert before is not None and after is not None
    assert graph.trash[name].node not in before
    assert graph.trash[name].node in graph.vertices

if __name__ == '__main__':
    main()
