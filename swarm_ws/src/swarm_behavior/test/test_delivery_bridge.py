import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from swarm_planning.bridge import PlanningBridge
from swarm_planning import assignment, cbs


class DeliveryBridgeTest(unittest.TestCase):
    def setUp(self):
        self.bridge = PlanningBridge()
        self.pickup = self.bridge.plan_pickup(assignment.DEFAULT_STARTS)
        self.names = tuple(name for name, _ in self.pickup.assigned)
        for name in self.names:
            self.bridge.confirm_collected(name)
        self.starts = tuple(path[-1] for path in self.pickup.paths)

    def test_matches_original_delivery_solver(self):
        new = self.bridge.plan_delivery(self.starts, self.names)
        expected_goals = tuple(self.bridge.deposit_goals(
            self.bridge.graph.trash[n].color) for n in self.names)
        old_result = assignment.solve_multi(self.bridge.graph, self.starts, expected_goals)
        self.assertEqual(new.goals, expected_goals)
        self.assertEqual(new.paths, old_result['paths'])
        self.assertEqual((new.cost, new.expanded, new.generated),
                         (old_result['cost'], old_result['expanded'], old_result['generated']))
        self.assertIsNone(cbs.first_conflict(new.paths))

    def test_two_robots_same_deposit(self):
        greens = sorted(n for n, v in self.bridge.graph.trash.items() if v.color == 'verde')
        orange = next(n for n, v in self.bridge.graph.trash.items() if v.color == 'naranja')
        bridge = PlanningBridge()
        for name in greens + [orange]:
            bridge.confirm_collected(name)
        starts = ((6, 14), (8, 18), (14, 24))
        plan = bridge.plan_delivery(starts, (*greens, orange))
        self.assertEqual(plan.colors, ('verde','verde','naranja'))
        self.assertEqual(plan.goals[0], plan.goals[1])
        self.assertNotEqual(plan.paths[0][-1], plan.paths[1][-1])
        self.assertIsNone(cbs.first_conflict(plan.paths))

    def test_reject_pending_or_repeated(self):
        with self.assertRaises(ValueError):
            self.bridge.plan_delivery(self.starts, (self.names[0],)*3)
        pending = next(iter(self.bridge.pending))
        with self.assertRaises(ValueError):
            self.bridge.plan_delivery(self.starts, (self.names[0], self.names[1], pending))

    def test_does_not_change_map(self):
        before = self.bridge.pending.copy()
        self.bridge.plan_delivery(self.starts, self.names)
        self.assertEqual(self.bridge.pending, before)

    def test_two_rounds_offline(self):
        b = PlanningBridge()
        starts = assignment.DEFAULT_STARTS
        delivered=[]
        for _ in range(2):
            pickup = b.plan_pickup(starts)
            names = tuple(n for n, _ in pickup.assigned)
            for n in names:
                b.confirm_collected(n)
            delivery = b.plan_delivery(tuple(p[-1] for p in pickup.paths), names)
            starts = tuple(p[-1] for p in delivery.paths)
            delivered.extend(names)
        self.assertEqual(len(set(delivered)),6)
        self.assertEqual(len(b.pending),0)
        self.assertEqual(len(b.graph.vertices),767)

if __name__ == '__main__': unittest.main()
