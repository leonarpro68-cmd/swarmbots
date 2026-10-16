"""Regresión: /odom envejece mientras se eliminan modelos en Gazebo."""
import json
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from cbs_entrega_depositos_ros import PickupDeliveryExecutor


class DummyLog:
    def __init__(self):
        self.lines = []
    def info(self, msg): self.lines.append(('info', msg))
    def error(self, msg): self.lines.append(('error', msg))


class Tests(unittest.TestCase):
    def make_demo(self, state_file):
        demo = PickupDeliveryExecutor.__new__(PickupDeliveryExecutor)
        demo.stage = 'PICKUP'
        demo.delivery_odom_wait_timeout = 10.0
        demo.phase = 'DONE'
        demo.finished_removal = True
        demo.state_file = Path(state_file)
        demo.assignments = [('trash_V2_verde', 1), ('trash_N2_naranja', 2), ('trash_V1_verde', 3)]
        demo.node = SimpleNamespace(get_logger=lambda: self.log)
        demo.agents = ('summit0','summit1','summit2')
        demo.fresh = lambda: False
        demo.stop_all = lambda: self.stopped.append(True)
        return demo

    def setUp(self):
        self.log = DummyLog()
        self.stopped = []

    def test_stale_odom_after_pickup_waits_instead_of_failing(self):
        with tempfile.TemporaryDirectory() as folder:
            state_file = Path(folder)/'state.json'
            state_file.write_text(json.dumps({
                'removed_confirmed':['trash_V2_verde','trash_N2_naranja','trash_V1_verde'],
                'failed': {}, 'pending': []}))
            demo = self.make_demo(state_file)
            # Parent class already completed the removal in this synthetic scenario.
            with patch('cbs_entrega_depositos_ros.PickupAndRemoveExecutor.tick'):
                demo.tick()
            self.assertEqual(demo.stage, 'WAIT_DELIVERY_ODOM')
            self.assertFalse(self.stopped)
            demo.tick()  # odom still stale
            self.assertEqual(demo.stage, 'WAIT_DELIVERY_ODOM')
            self.assertFalse(self.stopped)

    def test_wait_timeout_fails_safely(self):
        demo = self.make_demo('/tmp/not-used')
        demo.stage = 'WAIT_DELIVERY_ODOM'
        demo.delivery_odom_wait_started = time.monotonic() - 15.0
        demo.delivery_odom_wait_timeout = 10.0
        demo.tick()
        self.assertEqual(demo.stage, 'FAILED')
        self.assertTrue(self.stopped)

if __name__=='__main__':
    unittest.main()
