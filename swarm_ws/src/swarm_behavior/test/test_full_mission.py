import sys
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from swarm_planning.bridge import PlanningBridge
from swarm_planning.mission_core import MissionPlanningCoordinator
from swarm_planning.ros_full_mission import FullMissionExecutor


class Log:
    def info(self, *args): pass
    def error(self, *args): pass


class Test6E(unittest.TestCase):
    def setUp(self):
        self.temp = TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.coord = MissionPlanningCoordinator(PlanningBridge())
        self.addCleanup(self.coord.close)
        self.exe = FullMissionExecutor.__new__(FullMissionExecutor)
        self.exe.coordinator = self.coord
        self.exe.node = SimpleNamespace(get_logger=lambda: Log())
        self.exe.round_index = 1
        self.exe.rounds = []
        self.exe.mission_state_file = Path(self.temp.name) / 'mission.json'
        self.exe.stop = lambda: None
        self.exe.fail = lambda msg: setattr(self.exe, 'state', 'FAILED')
        self.exe.state = 'DELIVERY_DONE'
        self.exe.removal_future = SimpleNamespace(done=lambda: True)
        self.exe.result = SimpleNamespace(assigned=(( 'trash_V1_verde', (1,1)), ('trash_V2_verde', (2,2)), ('trash_N1_naranja', (3,3))), cost=10)
        self.exe.delivery_plan = SimpleNamespace(cost=20, paths=(((1,1),), ((2,2),), ((3,3),)))

    def deliver(self, names):
        self.coord.confirm_pickups(names)
        self.coord.confirm_delivery()

    def test_primera_ronda_reinicia_y_preserva_mapa(self):
        names = tuple(sorted(self.coord.pending)[:3])
        self.deliver(names)
        self.exe._finish_round()
        self.assertEqual(self.exe.state, 'WAIT_ODOM')
        self.assertEqual(self.exe.round_index, 2)
        self.assertEqual(len(self.coord.pending), 3)
        self.assertEqual(len(self.coord.delivered), 3)
        self.assertNotIn(names[0], self.coord.bridge.graph.pending)
        self.assertEqual(len(self.exe.rounds), 1)

    def test_segunda_ronda_completa_solo_despues_de_entregar(self):
        names = tuple(sorted(self.coord.pending)[:3])
        self.deliver(names)
        self.exe._finish_round()
        remaining = tuple(sorted(self.coord.pending))
        self.coord.confirm_pickups(remaining)
        self.assertFalse(self.coord.mission_complete)
        self.coord.confirm_delivery()
        self.exe.round_index = 2
        self.exe.result = SimpleNamespace(assigned=tuple((name,(1,1)) for name in remaining), cost=12)
        self.exe.delivery_plan = SimpleNamespace(cost=23, paths=(((1,1),), ((2,2),), ((3,3),)))
        self.exe._finish_round()
        self.assertEqual(self.exe.state, 'MISSION_DONE')
        self.assertEqual(len(self.coord.delivered), 6)
        self.assertEqual(len(self.coord.pending), 0)
        self.assertEqual(len(self.coord.bridge.graph.vertices), 764)

        for deposit in self.coord.bridge.graph.deposits.values():
            self.assertNotIn(
                deposit.node,
                self.coord.bridge.graph.vertices
            )
        import json
        data=json.loads(self.exe.mission_state_file.read_text())
        self.assertEqual(data['status'],'completed')
        self.assertEqual(data['rounds_completed'],2)

    def test_no_hay_fin_si_falta_entrega(self):
        self.coord.confirm_pickups(tuple(sorted(self.coord.pending)[:3]))
        self.exe._finish_round()
        self.assertEqual(self.exe.state, 'FAILED')
        self.assertEqual(len(self.exe.rounds), 0)

    def test_no_se_duplica_transicion(self):
        names = tuple(sorted(self.coord.pending)[:3])
        self.deliver(names)
        self.exe._finish_round()
        self.assertEqual(self.exe.state, 'WAIT_ODOM')
        self.assertEqual(len(self.exe.rounds), 1)
        self.assertIsNone(self.exe.result)
        self.assertIsNone(self.exe.removal_future)


if __name__ == '__main__':
    unittest.main()
