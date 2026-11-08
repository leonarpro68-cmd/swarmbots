import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from swarm_planning.gazebo_removal import find_remove_service, remove_model, remove_batch
from swarm_planning.ros_pickup_removal import PickupRemovalExecutor


class RemovalTests(unittest.TestCase):
    def test_service_discovery(self):
        fake = lambda *a, **k: SimpleNamespace(returncode=0, stdout='/world/world_demo/remove\n', stderr='')
        self.assertEqual(find_remove_service(run=fake), '/world/world_demo/remove')

    def test_reject_ambiguous_world(self):
        fake = lambda *a, **k: SimpleNamespace(returncode=0, stdout='/world/a/remove\n/world/b/remove\n', stderr='')
        with self.assertRaises(RuntimeError):
            find_remove_service(run=fake)

    def test_confirm_explicit_true_only(self):
        ok = lambda *a, **k: SimpleNamespace(returncode=0, stdout='data: true\n', stderr='')
        false = lambda *a, **k: SimpleNamespace(returncode=0, stdout='data: false\n', stderr='')
        remove_model('trash_V1_verde', '/world/world_demo/remove', run=ok)
        with self.assertRaises(RuntimeError):
            remove_model('trash_V1_verde', '/world/world_demo/remove', run=false)

    def test_partial_failure_never_confirmed(self):
        def remover(name, service):
            if name == 'b':
                raise TimeoutError('Service call timed out')
        result = remove_batch(('a', 'b', 'c'), find_service=lambda w: '/world/world_demo/remove', remove=remover)
        self.assertEqual(result.confirmed, ('a', 'c'))
        self.assertEqual(set(result.failures), {'b'})

    def test_duplicate_names_rejected(self):
        with self.assertRaises(ValueError):
            remove_batch(('a', 'a'))

    def test_state_update_only_after_success(self):
        # Prueba del resultado sin levantar rclpy ni Gazebo.
        from swarm_planning.bridge import PlanningBridge
        from swarm_planning.mission_core import MissionPlanningCoordinator
        from swarm_planning.gazebo_removal import RemovalResult
        from concurrent.futures import Future
        import tempfile
        bridge = PlanningBridge()
        coordinator = MissionPlanningCoordinator(bridge)
        f = Future()
        names = sorted(coordinator.pending)
        f.set_result(RemovalResult(tuple(names[:2]), {names[2]: 'timeout'}, '/world/world_demo/remove'))
        exe = PickupRemovalExecutor.__new__(PickupRemovalExecutor)
        exe.removal_future = f
        exe.coordinator = coordinator
        exe.state = 'REMOVING'
        class Log:
            def info(self, *args): pass
            def error(self, *args): pass
        exe.node = SimpleNamespace(get_logger=lambda: Log())
        with tempfile.TemporaryDirectory() as d:
            exe.state_file = Path(d)/'status.json'
            exe._poll_removal()
            self.assertEqual(exe.state, 'REMOVAL_FAILED')
            self.assertEqual(set(names[:2]).intersection(coordinator.pending), set())
            self.assertIn(names[2], coordinator.pending)
            self.assertEqual(len(coordinator.pending), 4)
        coordinator.close()


if __name__ == '__main__':
    unittest.main()
