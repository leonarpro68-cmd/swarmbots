import pathlib
import sys
import unittest
import json

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from swarm_planning.ros_plan_helpers import RobotSample, snapshot, planning_starts, select_alignment
from swarm_planning.ros_planning_probe import plan_report
from swarm_planning.bridge import PlanningBridge


class TestProbe(unittest.TestCase):
    def test_missing_odom(self):
        self.assertIsNone(snapshot({}, 1., now=10.))

    def test_expired_odom(self):
        x = {f'summit{i}': RobotSample(i * 3., 0., 1.) for i in range(3)}
        self.assertIsNone(snapshot(x, 1., now=3.))
        self.assertEqual(len(snapshot(x, 1., now=1.5)), 3)

    def test_align_to_free_nodes(self):
        g = PlanningBridge().graph
        from swarm_planning.geometry import node_position
        starts = [(12, 16), (15, 25), (8, 18)]
        samples = [RobotSample(*node_position(v), 0.) for v in starts]
        self.assertEqual(planning_starts(g, samples), tuple(starts))

    def test_trash_node_not_selected(self):
        g = PlanningBridge().graph
        name = 'trash_V1_verde'
        blocked = g.trash[name].node
        self.assertNotIn(blocked, g.vertices)
        self.assertNotEqual(select_alignment(g, (-3.75, -8.75), radius=3.), blocked) if g.free_position((-3.75,-8.75)) else None

    def test_report_serializable(self):
        b = PlanningBridge()
        plan = b.plan_pickup(((12, 16), (15, 25), (8, 18)))
        doc = plan_report(plan)
        self.assertEqual(doc['status'], 'planned_only')
        self.assertEqual(len(doc['paths']), 3)
        json.dumps(doc)


if __name__ == '__main__':
    unittest.main()
