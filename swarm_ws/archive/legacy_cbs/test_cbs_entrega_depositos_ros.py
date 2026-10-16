import unittest
import cbs_grafo_125 as cbs
from grafo_basuras_dinamico import DynamicTrashGraph
from cbs_entrega_depositos_ros import deposit_goals, plan_delivery

class TestDelivery(unittest.TestCase):
    def setUp(self):
        self.g = DynamicTrashGraph()
        self.names = ('trash_V1_verde','trash_V2_verde','trash_N2_naranja')
        for n in self.names: self.g.collect(n)
        self.starts = ((7,13),(13,13),(14,26))

    def test_deposits_are_neighbors_and_free(self):
        for color in ('verde','naranja','azul'):
            cells = deposit_goals(self.g,color)
            depot = next(d for d in self.g.deposits.values() if d.color == color)
            self.assertTrue(cells)
            self.assertNotIn(depot.node,cells)
            self.assertTrue(all(v in self.g.vertices for v in cells))
            self.assertTrue(all(max(abs(v[i]-depot.node[i]) for i in (0,1))==1 for v in cells))

    def test_two_robots_same_depot(self):
        goals,r = plan_delivery(self.g,self.starts,self.names)
        self.assertIsNone(cbs.first_conflict(r['paths']))
        self.assertIn(r['paths'][0][-1],goals[0])
        self.assertIn(r['paths'][1][-1],goals[1])
        self.assertNotEqual(r['paths'][0][-1],r['paths'][1][-1])
        self.assertEqual(goals[0],goals[1])
        self.assertNotEqual(goals[0],goals[2])
        self.assertEqual(len(self.g.pending),3)
        self.assertTrue(all(all(v in self.g.vertices for v in p) for p in r['paths']))

    def test_cannot_deliver_uncollected(self):
        with self.assertRaises(ValueError):
            plan_delivery(self.g,self.starts,('trash_A1_azul',self.names[1],self.names[2]))

    def test_no_duplicate_pickup(self):
        with self.assertRaises(ValueError):
            plan_delivery(self.g,self.starts,(self.names[0],self.names[0],self.names[2]))

    def test_no_invalid_start(self):
        with self.assertRaises(ValueError):
            plan_delivery(self.g,((8,32),self.starts[1],self.starts[2]),self.names)

if __name__=='__main__': unittest.main()
