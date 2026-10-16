import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

from cbs_mision_completa_ros import FullMissionExecutor
from grafo_basuras_dinamico import DynamicTrashGraph
from cbs_primera_recogida_ros import plan
from cbs_entrega_depositos_ros import plan_delivery


class TestFullMission(unittest.TestCase):
    def test_offline_two_rounds(self):
        g = DynamicTrashGraph()
        starts = ((12,16), (15,25), (8,18))
        chosen = []
        for round_number in (1,2):
            assignments, _, result = plan(starts, g)
            names = [n for n, _ in assignments]
            self.assertEqual(len(names), 3)
            self.assertEqual(len(set(names)), 3)
            self.assertFalse(set(names) & set(chosen))
            for n in names:
                g.collect(n)
            chosen += names
            pickup_ends = tuple(p[-1] for p in result['paths'])
            _, delivery = plan_delivery(g, pickup_ends, names)
            starts = tuple(p[-1] for p in delivery['paths'])
            self.assertEqual(len(g.pending), 6 - 3 * round_number)
        self.assertEqual(len(set(chosen)), 6)
        self.assertEqual(len(g.vertices), 767)

    def fake_executor(self, folder):
        m = FullMissionExecutor.__new__(FullMissionExecutor)
        m.round_index = 1
        m.history = []
        m.summary_file = Path(folder) / 'summary.json'
        m.state_file = Path(folder) / 'state.json'
        m.state_file.write_text('old state')
        m.node = MagicMock()
        m.agents = ('summit0','summit1','summit2')
        m.dynamic = DynamicTrashGraph()
        m.assignments = [(n, 1.0) for n in sorted(m.dynamic.pending)[:3]]
        for n, _ in m.assignments:
            m.dynamic.collect(n)
        m.stage = 'COMPLETE'
        m.phase = 'DONE'
        m.stop_all = MagicMock()
        return m

    def test_transition_keeps_dynamic_map(self):
        with tempfile.TemporaryDirectory() as folder:
            m = self.fake_executor(folder)
            initial_graph_id = id(m.dynamic)
            old_pending = set(m.dynamic.pending)
            m.completed_round()
            m.next_round()
            self.assertEqual(m.round_index, 2)
            self.assertEqual(m.stage, 'PICKUP')
            self.assertEqual(m.phase, 'WAIT_ODOM')
            self.assertFalse(m.finished_removal)
            self.assertEqual(id(m.dynamic), initial_graph_id)
            self.assertEqual(m.dynamic.pending, old_pending)
            self.assertFalse(m.state_file.exists())

    def test_summary_after_two_rounds(self):
        with tempfile.TemporaryDirectory() as folder:
            m = self.fake_executor(folder)
            m.completed_round()
            m.next_round()
            m.assignments = [(n, 1.0) for n in sorted(m.dynamic.pending)]
            for n, _ in m.assignments:
                m.dynamic.collect(n)
            m.completed_round()
            m.finish_mission()
            summary = json.loads(m.summary_file.read_text())
            self.assertEqual(summary['status'], 'completed')
            self.assertEqual(summary['collected_and_delivered'], 6)
            self.assertEqual(summary['pending'], [])
            self.assertEqual(m.stage, 'MISSION_COMPLETE')
            self.assertEqual(summary['vertices_free'], 767)
            self.assertEqual(len(summary['rounds']), 2)

    def test_no_early_finish(self):
        with tempfile.TemporaryDirectory() as folder:
            m = self.fake_executor(folder)
            m.completed_round()
            with self.assertRaises(RuntimeError):
                m.finish_mission()


if __name__ == '__main__':
    unittest.main()
