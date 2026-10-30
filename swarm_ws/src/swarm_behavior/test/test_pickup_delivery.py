import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from concurrent.futures import Future

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from swarm_planning.bridge import PlanningBridge
from swarm_planning.mission_core import MissionPlanningCoordinator
from swarm_planning.gazebo_removal import RemovalResult
from swarm_planning.ros_pickup_delivery import PickupDeliveryExecutor
from swarm_planning.ros_plan_helpers import RobotSample, AGENTS
from swarm_planning.geometry import node_position

class FakeLog:
    def info(self, *args): pass
    def error(self, *args): pass

class Test6D(unittest.TestCase):
    def setUp(self):
        self.coordinator = MissionPlanningCoordinator(PlanningBridge())
        self.addCleanup(self.coordinator.close)
        self.exe = PickupDeliveryExecutor.__new__(PickupDeliveryExecutor)
        self.exe.coordinator = self.coordinator
        self.exe.node = SimpleNamespace(get_logger=lambda: FakeLog())
        self.exe.state = 'REMOVING'
        self.exe.samples = {}
        self.exe.velocities = {}
        self.exe.odom_timeout = 2.
        self.exe.delivery_active = False
        self.exe.removal_future = Future()
        from tempfile import TemporaryDirectory
        self.tmp = TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.exe.state_file = Path(self.tmp.name) / 'pickup.json'
        self.exe.delivery_state_file = Path(self.tmp.name) / 'delivery.json'
        self.exe.stop = lambda: None
        self.exe.fail = lambda message: setattr(self.exe, 'state', 'FAILED')

    def test_only_after_three_confirmations(self):
        self.exe.removal_future.set_result(RemovalResult(('trash_V1_verde','trash_V2_verde'), {'trash_N1_naranja':'timeout'}, '/world/world_demo/remove'))
        self.exe._poll_removal()
        self.assertEqual(self.exe.state, 'REMOVAL_FAILED')
        self.assertEqual(len(self.coordinator.pending), 4)

    def test_all_confirmations_start_delivery_phase(self):
        names = tuple(sorted(self.coordinator.pending)[:3])
        self.exe.removal_future.set_result(RemovalResult(names, {}, '/world/world_demo/remove'))
        self.exe._poll_removal()
        self.assertEqual(self.exe.state, 'DELIVERY_WAIT_ODOM')
        self.assertEqual(len(self.coordinator.pending), 3)

    def test_delivery_planning_async(self):
        names = ('trash_V1_verde', 'trash_V2_verde', 'trash_N1_naranja')
        self.coordinator.confirm_pickups(names)
        starts = ((7,13), (13,13), (14,26))
        for a, n in zip(AGENTS, starts):
            x,y = node_position(n)
            import time
            self.exe.samples[a] = RobotSample(x,y,time.monotonic(),'odom')
        self.exe.state = 'DELIVERY_WAIT_ODOM'
        self.exe.tick()
        self.assertEqual(self.exe.state, 'DELIVERY_PLANNING')
        from time import sleep
        for _ in range(250):
            self.exe.tick()
            if self.exe.state == 'DELIVERY_REFRESH_ODOM':
                break
            sleep(.01)
        self.assertEqual(self.exe.state, 'DELIVERY_REFRESH_ODOM')
        self.assertEqual(len(self.exe.routes),3)
        self.assertNotEqual(self.exe.routes[0][-1], self.exe.routes[1][-1])

    def test_delivery_confirmation_and_json(self):
        names = ('trash_V1_verde','trash_V2_verde','trash_N1_naranja')
        self.coordinator.confirm_pickups(names)
        self.coordinator.confirm_delivery()
        self.exe.delivery_plan = SimpleNamespace(
            colors=('verde','verde','naranja'),
            paths=(((1,1),(2,2)), ((1,2),(2,3)), ((2,1),(3,2))),
            cost=3, expanded=1)
        self.exe._save_delivery()
        import json
        data=json.loads(self.exe.delivery_state_file.read_text())
        self.assertEqual(data['status'],'delivered')
        self.assertEqual(len(data['delivered']),3)
        self.assertEqual(len(data['pending']),3)

if __name__ == '__main__': unittest.main()
