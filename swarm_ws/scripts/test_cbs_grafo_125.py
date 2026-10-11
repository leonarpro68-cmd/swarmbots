#!/usr/bin/env python3
"""Pruebas offline para cbs_grafo_125.py (unittest de biblioteca estándar)."""
import unittest
import cbs_grafo_125 as c

class CBSTests(unittest.TestCase):
    def setUp(self):
        self.g=c.Graph()
    def test_graph(self):
        self.assertEqual(len(self.g.vertices),767)
    def test_initial_separation(self):
        self.assertIsNotNone(c.first_conflict((((8,18),),((9,18),))))
    def test_crossing_edges(self):
        a=(8,18); b=(9,19); d=(8,19); e=(9,18)
        self.assertLess(c.simultaneous_distance(a,b,d,e),1e-9)
    def test_moving_parallel_close(self):
        self.assertAlmostEqual(c.simultaneous_distance((8,18),(9,18),(8,19),(9,19)),1.25)
    def test_low_level_constraint(self):
        s=(8,18); goal=(10,18)
        p=c.low_level(self.g,0,s,goal,frozenset((c.Constraint(0,0,edge=(s,(9,18))),)))
        self.assertIsNotNone(p)
        self.assertNotEqual(p[1],(9,18))
    def test_wait_allowed(self):
        p=c.low_level(self.g,0,(8,18),(8,18),frozenset())
        self.assertEqual(p,((8,18),))
    def test_three_agents_cbs(self):
        starts=((8,18),(14,18),(11,14));goals=((14,18),(8,18),(11,22))
        res=c.solve(starts,goals,max_ct=5000)
        self.assertEqual(res['cost'],20)
        self.assertIsNone(c.first_conflict(res['paths']))
        self.assertGreater(res['expanded'],1)
        for i,p in enumerate(res['paths']):
            self.assertEqual(p[0],starts[i])
            self.assertEqual(p[-1],goals[i])
            for a,b in zip(p,p[1:]):
                self.assertTrue(a==b or b in self.g.neighbors[a])
    def test_raises_unsafe_starts(self):
        with self.assertRaises(ValueError):
            c.solve(((8,18),(9,18)),((8,20),(9,20)))

if __name__=='__main__':unittest.main(verbosity=2)
