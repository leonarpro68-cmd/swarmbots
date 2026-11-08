"""Regresiones de la limpieza del runner legado de semilla 100 (sin ROS)."""
from __future__ import annotations
import pathlib
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

WS = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(WS/'scripts/supermarket_seed_pipeline'))
import ros_preflight
import run_prepared_mission

class LegacyCleanupTest(unittest.TestCase):
    def test_common_preflight_same_readiness_gates(self):
        with tempfile.TemporaryDirectory() as tmp:
            ws = pathlib.Path(tmp)
            for p in (ws/'scripts/supermarket_preflight/ros_observer_readonly.py',
                      ws/'scripts/supermarket_support/check_support_graph.py'):
                p.parent.mkdir(parents=True,exist_ok=True)
                p.touch()
            msgs = [SimpleNamespace(returncode=0,stdout='PASS_SUPPORT_GRAPH_NOT_EXECUTION_AUTHORIZATION',stderr=''),
                    SimpleNamespace(returncode=0,stdout='PASS_READ_ONLY_NOT_EXECUTION_AUTHORIZATION',stderr='')]
            with patch.object(ros_preflight.subprocess,'run',side_effect=msgs) as run:
                ros_preflight.check_live(ws,ws/'scenario.json',10)
            self.assertEqual(run.call_count,2)
            cmd = run.call_args_list[-1].args[0]
            self.assertIn('ros_preflight_readonly.json',str(cmd))
            self.assertNotIn('supermarket_one_launch',str(cmd))

    def test_rejects_missing_support_without_attempting_mission(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(FileNotFoundError):
                ros_preflight.check_live(pathlib.Path(tmp),pathlib.Path(tmp)/'scene.json',2)

    def test_legacy_entrypoints_removed(self):
        self.assertFalse((WS/'src/swarm_worlds/launch/supermarket_full_experiment.launch.py').exists())
        self.assertFalse((WS/'scripts/supermarket_one_launch/run_unique_mission.py').exists())
        self.assertTrue(callable(ros_preflight.check_live))
        src = pathlib.Path(run_prepared_mission.__file__).read_text()
        self.assertIn('from ros_preflight import check_live',src)
        self.assertNotIn('from run_unique_mission import',src)

if __name__=='__main__': unittest.main()
