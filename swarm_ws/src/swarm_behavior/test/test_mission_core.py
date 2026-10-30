import sys
import unittest
from pathlib import Path
from threading import Event
from time import monotonic, sleep
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from swarm_planning.bridge import PlanningBridge
from swarm_planning.mission_core import MissionPlanningCoordinator
from swarm_planning import assignment


def await_result(coordinator, timeout=30):
    deadline = monotonic() + timeout
    while monotonic() < deadline:
        result = coordinator.poll()
        if result is not None:
            if result.error:
                raise result.error
            return result
        sleep(.015)
    raise AssertionError('Timeout en prueba')

class TestMissionCore(unittest.TestCase):
    def setUp(self):
        self.co = MissionPlanningCoordinator()

    def tearDown(self):
        self.co.close()

    def test_deposit_eight_neighbors_matches_legacy(self):
        b = PlanningBridge()
        # El original permite hasta 8 vecinos, no solo 4
        for color in ('verde', 'naranja', 'azul'):
            deposit = next(d for d in b.graph.deposits.values() if d.color == color)
            i,j = deposit.node
            expected = tuple(sorted((i+dx,j+dy) for dx in (-1,0,1) for dy in (-1,0,1)
                                    if (dx or dy) and (i+dx,j+dy) in b.graph.vertices))
            self.assertEqual(b.deposit_goals(color), expected)

    def test_two_rounds(self):
        starts = assignment.DEFAULT_STARTS
        for _ in range(2):
            self.co.submit_pickup(starts)
            pickup = await_result(self.co)
            self.assertEqual(pickup.kind, 'pickup')
            names = tuple(n for n, _ in pickup.value.assigned)
            self.co.confirm_pickups(names)
            self.co.submit_delivery(tuple(path[-1] for path in pickup.value.paths))
            delivery = await_result(self.co)
            self.assertEqual(delivery.kind, 'delivery')
            starts = tuple(path[-1] for path in delivery.value.paths)
            self.co.confirm_delivery()
        self.assertTrue(self.co.mission_complete)
        self.assertEqual(len(set(self.co.delivered)), 6)

    def test_reject_graph_mutation_during_cbs(self):
        self.co.submit_pickup(assignment.DEFAULT_STARTS)
        with self.assertRaises(RuntimeError):
            self.co.confirm_pickups(('trash_V1_verde',))
        await_result(self.co)

    def test_poll_nonblocking_while_worker_busy(self):
        ready, release = Event(), Event()
        original = self.co.bridge.plan_pickup
        def delayed(*args, **kwargs):
            ready.set()
            release.wait(timeout=5)
            return original(*args, **kwargs)
        self.co.bridge.plan_pickup = delayed
        try:
            self.co.submit_pickup(assignment.DEFAULT_STARTS)
            self.assertTrue(ready.wait(timeout=2))
            t=monotonic()
            self.assertIsNone(self.co.poll())
            self.assertLess(monotonic()-t,.3)
        finally:
            release.set()
        await_result(self.co)

    def test_no_delivery_before_pickup(self):
        with self.assertRaises(RuntimeError):
            self.co.submit_delivery(assignment.DEFAULT_STARTS)
        self.assertFalse(self.co.mission_complete)

if __name__ == '__main__': unittest.main()
