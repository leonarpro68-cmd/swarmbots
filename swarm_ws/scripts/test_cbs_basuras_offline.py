import unittest
import cbs_grafo_125 as cbs
from grafo_basuras_dinamico import DynamicTrashGraph
from cbs_basuras_offline import STARTS, GOALS, TARGET, run_demo, verify

class TestCBSBasurasOffline(unittest.TestCase):
    def test_no_trash_crossings(self):
        g=DynamicTrashGraph()
        res=cbs.solve(STARTS,GOALS,graph=g)
        self.assertTrue(verify(res,g,STARTS,GOALS))
        for p in res['paths']:
            for v in p:
                self.assertNotIn(v,{e.node for name,e in g.trash.items() if name in g.pending})

    def test_collect_changes_graph_and_replans(self):
        g,before,after,original=run_demo()
        self.assertEqual(len(original),6)
        self.assertEqual(len(g.pending),5)
        self.assertIn(g.trash[TARGET].node, g.vertices)
        self.assertTrue(verify(after,g,STARTS,GOALS))
        self.assertNotIn(g.trash[TARGET].node, before['paths'][0])
        self.assertIsNone(cbs.low_level(DynamicTrashGraph(),0,STARTS[0],g.trash[TARGET].node,frozenset()))
        route_to_released = cbs.low_level(g,0,STARTS[0],g.trash[TARGET].node,frozenset())
        self.assertIsNotNone(route_to_released)
        self.assertEqual(route_to_released[-1],g.trash[TARGET].node)

    def test_old_api_unchanged(self):
        g=cbs.Graph()
        self.assertEqual(len(g.vertices),767)
        res=cbs.solve(((8,18),(14,18),(11,14)),((14,18),(8,18),(11,22)),max_ct=5000)
        self.assertEqual(res['cost'],20)

if __name__=='__main__': unittest.main(verbosity=2)
