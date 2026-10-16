import sys
import tempfile
import unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from swarm_planning.bridge import PlanningBridge
from swarm_planning import assignment
from swarm_planning.scenario import generate, save

class BridgeTest(unittest.TestCase):
    def test_fixed_equivalence(self):
        starts = assignment.DEFAULT_STARTS
        _, old_a, old_goals, old_solution = assignment.plan_first_round(starts)
        new = PlanningBridge().plan_pickup(starts)
        self.assertEqual(new.assigned, old_a)
        self.assertEqual(new.goals, old_goals)
        self.assertEqual(new.paths, old_solution['paths'])
        self.assertEqual((new.cost,new.expanded,new.generated), (old_solution['cost'],old_solution['expanded'],old_solution['generated']))

    def test_collection_changes_only_confirmed(self):
        bridge = PlanningBridge()
        plan = bridge.plan_pickup(assignment.DEFAULT_STARTS)
        self.assertEqual(len(bridge.pending), 6)
        name=plan.assigned[0][0]
        node=bridge.graph.trash[name].node
        self.assertNotIn(node, bridge.graph.vertices)
        bridge.confirm_collected(name)
        self.assertNotIn(name, bridge.pending)
        self.assertIn(node, bridge.graph.vertices)
        self.assertEqual(len(bridge.pending), 5)
        with self.assertRaises(ValueError): bridge.confirm_collected(name)

    def test_independent_scenarios(self):
        scenario=generate(42)
        b1=PlanningBridge(scenario)
        b2=PlanningBridge(scenario)
        name=sorted(b1.pending)[0]
        b1.confirm_collected(name)
        self.assertNotIn(name,b1.pending)
        self.assertIn(name,b2.pending)

    def test_json(self):
        scenario=generate(7)
        with tempfile.TemporaryDirectory() as tmp:
            file=Path(tmp)/'scenario.json'
            save(scenario,file)
            bridge=PlanningBridge.from_json(file)
        self.assertEqual(bridge.scenario.seed,7)
        self.assertEqual(len(bridge.pending),6)

if __name__=='__main__': unittest.main()
