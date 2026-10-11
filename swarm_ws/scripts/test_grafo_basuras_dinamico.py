import unittest
import math
from grafo_basuras_dinamico import DynamicTrashGraph, TRASH_EXCLUSION_M
import grafo_cbs_125 as geom

class TestGrafoBasuras(unittest.TestCase):
    def setUp(self):
        self.g = DynamicTrashGraph()

    def test_six_blocked(self):
        self.assertEqual(len(self.g.pending), 6)
        for e in self.g.trash.values():
            self.assertNotIn(e.node, self.g.vertices)

    def test_deposits_unblocked(self):
        for e in self.g.deposits.values():
            self.assertIn(e.node, self.g.vertices)

    def test_all_edges_clear(self):
        for a, b in self.g.edges:
            self.assertTrue(self.g.segment_clear(geom.node_position(a), geom.node_position(b)))

    def test_collect_updates(self):
        name = 'trash_V1_verde'
        v = self.g.trash[name].node
        self.g.collect(name)
        self.assertIn(v, self.g.vertices)
        self.assertNotIn(name, self.g.pending)
        with self.assertRaises(ValueError): self.g.collect(name)

    def test_static_restored(self):
        for name in list(self.g.pending):
            self.g.collect(name)
        self.assertEqual(self.g.vertices, self.g.static_vertices)
        self.assertEqual(set(self.g.edges), set(self.g.static_edges))

    def test_astar_before_after(self):
        s, t = (8, 10), (8, 14)
        p1 = self.g.astar(s, t)
        self.assertIsNotNone(p1)
        self.assertNotIn((8, 12), p1)
        self.g.collect('trash_V1_verde')
        p2 = self.g.astar(s, t)
        self.assertIsNotNone(p2)
        self.assertIn((8, 12), p2)

    def test_unknown(self):
        with self.assertRaises(ValueError): self.g.collect('inexistente')

if __name__ == '__main__': unittest.main()
