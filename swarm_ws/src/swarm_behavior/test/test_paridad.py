"""Comparación directa con la versión validada, sin ROS ni Gazebo."""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LEGACY = ROOT / "legacy_reference"
sys.path.insert(0, str(LEGACY))
sys.path.insert(0, str(ROOT))

import grafo_cbs_125 as old_geom
import cbs_grafo_125 as old_cbs
from grafo_basuras_dinamico import DynamicTrashGraph as OldMap
import cbs_asignacion_basuras as old_assignment
from swarm_planning import geometry, cbs, dynamic_map, assignment


class ParidadTest(unittest.TestCase):
    def test_grafo(self):
        old = old_geom.make_graph()
        new = geometry.make_graph()
        self.assertEqual(old, new)

    def test_cbs_directo(self):
        scenarios = [
            (((12,16),(15,25),(8,18)), ((14,18),(8,18),(11,22))),
            (((8,18),(14,18),(11,14)), ((14,18),(8,18),(11,22))),
        ]
        for starts, goals in scenarios:
            with self.subTest(starts=starts):
                old = old_cbs.solve(starts, goals, max_ct=5000)
                new = cbs.solve(starts, goals, max_ct=5000)
                self.assertEqual(old['paths'], new['paths'])
                self.assertEqual(old['cost'], new['cost'])
                self.assertEqual(old['expanded'], new['expanded'])

    def test_basuras_dinamicas(self):
        a, b = OldMap(), dynamic_map.DynamicTrashGraph()
        self.assertEqual(a.pending, b.pending)
        deposit_nodes = {d.node for d in b.deposits.values()}

        self.assertEqual(
            a.vertices - b.vertices,
            deposit_nodes
        )

        self.assertTrue(b.vertices.issubset(a.vertices))
        self.assertTrue(set(b.edges).issubset(set(a.edges)))

        for deposit in b.deposits.values():
            self.assertNotIn(deposit.node, b.vertices)

        for start, end in b.edges:
            self.assertNotIn(start, deposit_nodes)
            self.assertNotIn(end, deposit_nodes)
        for name in ('trash_V1_verde', 'trash_N2_naranja', 'trash_A1_azul'):
            a.collect(name)
            b.collect(name)

            self.assertEqual(
                a.vertices - b.vertices,
                deposit_nodes
            )

            self.assertTrue(b.vertices.issubset(a.vertices))

            for deposit in b.deposits.values():
                self.assertNotIn(deposit.node, b.vertices)

    def test_asignacion(self):
        starts = ((12,16),(15,25),(8,18))
        old_graph, new_graph = OldMap(), dynamic_map.DynamicTrashGraph()
        self.assertEqual(old_assignment.assign_nearest(starts,old_graph),
                         assignment.assign_nearest(starts,new_graph))
        old_plan = old_assignment.plan_first_round(starts)
        new_plan = assignment.plan_first_round(starts)
        self.assertEqual(old_plan[0].pending, new_plan[0].pending)
        deposit_nodes = {
            d.node for d in new_graph.deposits.values()
        }

        self.assertEqual(
            old_plan[0].vertices - new_plan[0].vertices,
            deposit_nodes
        )

        self.assertTrue(
            new_plan[0].vertices.issubset(old_plan[0].vertices)
        )
        self.assertEqual(old_plan[1:3], new_plan[1:3])
        for key in ('paths', 'cost', 'expanded', 'generated'):
            self.assertEqual(old_plan[3]['cost'], new_plan[3]['cost'])

if __name__ == '__main__':
    unittest.main(verbosity=2)
