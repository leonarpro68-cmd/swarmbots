#!/usr/bin/env python3
"""Pruebas de A* en grafo 1.25 m sin ROS ni Gazebo.

Poner junto a astar_grafo_125.py y grafo_cbs_125.py y ejecutar:
    python3 -m unittest -v test_astar_grafo_125
"""
import heapq
import math
import unittest

import astar_grafo_125 as solver
import grafo_cbs_125 as geom


class TestAStar125(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.graph = solver.AStarGraph()

    def test_01_grafo_exactamente_el_mismo(self):
        self.assertEqual(len(self.graph.vertices), 767)
        self.assertEqual(len(self.graph.edges), 2816)

    def test_02_costes_movimiento_125(self):
        for a, neigh in self.graph.adjacency.items():
            for b, w in neigh:
                self.assertAlmostEqual(w, 1.25 if a[0] == b[0] or a[1] == b[1]
                                       else 1.25 * math.sqrt(2), places=8)
                self.assertIn((a, w), self.graph.adjacency[b])

    def test_03_heuristica_admisible(self):
        route, cost, _ = self.graph.astar((12, 16), (19, 12))
        self.assertLessEqual(self.graph.heuristic(route[0], route[-1]), cost + 1e-9)

    def test_04_plan_sobre_vertices_y_no_suaviza(self):
        p = self.graph.plan_world((1.401336, -3.934677), goal_vertex=(19, 12))
        self.assertEqual(p.route_vertices[0], p.start_vertex)
        self.assertEqual(p.route_vertices[-1], p.goal_vertex)
        self.assertEqual(p.waypoints, [geom.node_position(n) for n in p.route_vertices])
        self.assertEqual(p.waypoints[-1], (10.0, -8.75))

    def test_05_cada_arista_de_ruta_esta_en_grafo(self):
        p = self.graph.plan_world((1.401336, -3.934677), goal_vertex=(19, 12))
        for a, b in zip(p.route_vertices, p.route_vertices[1:]):
            self.assertIn(b, [v for v, c in self.graph.adjacency[a]])

    def test_06_conector_y_segmentos_son_libres(self):
        p = self.graph.plan_world((1.401336, -3.934677), goal_vertex=(19, 12))
        for a, b in zip([p.requested_start] + p.waypoints[:-1], p.waypoints):
            self.assertTrue(geom.segment_clear(a, b), f'{a} -> {b}')

    def test_07_no_puede_seleccionar_meta_bloqueada(self):
        # Centenares de nodos bloqueados; uno es suficiente.
        v = min(self.graph.blocked)
        with self.assertRaisesRegex(ValueError, 'bloqueado'):
            self.graph.plan_world((0, 0), goal_vertex=v)

    def test_08_rechaza_meta_continua_dentro_columna(self):
        with self.assertRaises(ValueError):
            self.graph.plan_world((0, 0), goal=(7.4, -7.5))

    def test_09_rechaza_inicio_dentro_columna(self):
        with self.assertRaises(ValueError):
            self.graph.plan_world((7.4, -7.5), goal_vertex=(19, 12))

    def test_10_meta_continua_se_snappea_a_vertice_libre(self):
        p = self.graph.plan_world((1.401336, -3.934677), goal=(9.5, -8.5))
        self.assertIn(p.goal_vertex, self.graph.vertices)
        self.assertEqual(p.waypoints[-1], geom.node_position(p.goal_vertex))
        self.assertNotEqual(p.waypoints[-1], (9.5, -8.5))

    def test_11_inicio_ya_en_destino(self):
        p = self.graph.plan_world((5.0, -6.25), goal_vertex=(15, 14))
        self.assertEqual(p.route_vertices, [(15, 14)])
        self.assertEqual(p.waypoints, [])
        self.assertAlmostEqual(p.graph_cost_m, 0.0)

    def test_12_optimalidad_comparada_con_dijkstra(self):
        cases = [((12, 16), (19, 12)), ((5, 10), (18, 32)),
                 ((11, 19), (14, 14)), ((8, 29), (17, 6)), ((15, 14), (19, 12))]
        for start, goal in cases:
            if start not in self.graph.vertices or goal not in self.graph.vertices:
                continue
            route, cost, _ = self.graph.astar(start, goal)
            pq = [(0.0, start)]
            dist = {start: 0.0}
            while pq:
                d, cur = heapq.heappop(pq)
                if d > dist[cur] + 1e-8:
                    continue
                if cur == goal:
                    break
                for nxt, w in self.graph.adjacency[cur]:
                    nd = d + w
                    if nd < dist.get(nxt, float('inf')):
                        dist[nxt] = nd
                        heapq.heappush(pq, (nd, nxt))
            self.assertAlmostEqual(cost, dist[goal], places=8)
            self.assertEqual(route[0], start)
            self.assertEqual(route[-1], goal)

    def test_13_linea_directa_choca_columna_y_plan_la_evade(self):
        start = (1.401336, -3.934677)
        goal = (10.0, -8.75)
        self.assertFalse(geom.segment_clear(start, goal))
        p = self.graph.plan_world(start, goal_vertex=(19, 12))
        self.assertGreaterEqual(len(p.route_vertices), 3)
        self.assertTrue(all(geom.segment_clear(a, b) for a, b in
                            zip([start] + p.waypoints[:-1], p.waypoints)))


if __name__ == '__main__':
    unittest.main(verbosity=2)
