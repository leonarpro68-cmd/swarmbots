import unittest
from cbs_step_executor_demo import select_alignment, two_step_route
import grafo_cbs_125 as geom

class TestDemo(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.nodes, edges, _, _=geom.make_graph()
        cls.adj={v:set() for v in cls.nodes}
        for a,b in edges:
            cls.adj[a].add(b);cls.adj[b].add(a)

    def test_three_initial_positions(self):
        points=((1.401,-3.934),(5.494,7.270),(-3.135,-0.664))
        for xy in points:
            start=select_alignment(self.nodes,xy)
            self.assertTrue(geom.segment_clear(xy,geom.node_position(start)))
            route=two_step_route(start,self.adj)
            self.assertEqual(len(route),3)
            for a,b in zip(route,route[1:]):
                self.assertIn(b,self.adj[a])
                self.assertTrue(geom.segment_clear(geom.node_position(a),geom.node_position(b)))

    def test_blocked_start_is_rejected(self):
        with self.assertRaises(ValueError):
            select_alignment(self.nodes,(7.4,7.5))

    def test_non_graph_point_rejected(self):
        with self.assertRaises(ValueError):
            two_step_route((-100,-100),self.adj)

if __name__=='__main__':unittest.main()
