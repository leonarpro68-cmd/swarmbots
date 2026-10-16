import unittest
import cbs_grafo_125 as cbs
import cbs_asignacion_basuras as old
from grafo_basuras_dinamico import DynamicTrashGraph
from cbs_primera_recogida_ros import plan, choose_safe_alignment
import grafo_cbs_125 as geom

class TestFirstPickup(unittest.TestCase):
    def test_plan_three_unique(self):
        graph=DynamicTrashGraph()
        ass, opts, result=plan(old.DEFAULT_STARTS, graph)
        self.assertEqual(len({name for name,_ in ass}),3)
        self.assertIsNone(cbs.first_conflict(result['paths']))
        for i, path in enumerate(result['paths']):
            self.assertIn(path[-1],opts[i])
            self.assertTrue(all(n in graph.vertices for n in path))
        self.assertEqual(len(graph.pending),6)

    def test_alignment_avoids_pending_trash(self):
        graph=DynamicTrashGraph()
        node=old.DEFAULT_STARTS[0]
        self.assertEqual(choose_safe_alignment(graph,geom.node_position(node)),node)
        with self.assertRaises(ValueError):
            choose_safe_alignment(graph,geom.node_position(graph.trash['trash_V1_verde'].node))

    def test_blocked_start_rejected(self):
        graph=DynamicTrashGraph()
        with self.assertRaises(ValueError):
            plan((graph.trash['trash_V1_verde'].node, *old.DEFAULT_STARTS[1:]),graph)

if __name__ == '__main__': unittest.main()
