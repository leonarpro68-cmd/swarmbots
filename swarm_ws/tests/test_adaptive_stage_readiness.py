"""Readiness gates and adaptive deadline, without requiring ROS 2 or Gazebo."""
import importlib.util
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

WS = Path(__file__).resolve().parents[1]
COORD = WS / 'scripts/d9913_stage_coordinator.py'
spec = importlib.util.spec_from_file_location('adaptive_stage_coordinator_test', COORD)
coordinator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(coordinator)


class FakeNode:
    def __init__(self):
        self.clock_times = []
        self.clock_progress_wall = None
        self.odoms = {}
        self.publishers = {
            n: [SimpleNamespace(node_name='path_follower', node_namespace=f'/{n}')]
            for n in coordinator.ROBOTS
        }

    def get_publishers_info_by_topic(self, topic):
        return self.publishers[topic.strip('/').split('/')[0]]


def inputs():
    node = FakeNode()
    data = {n: (np.zeros((1, 6, 2)), np.array([1.0])) for n in coordinator.ROBOTS}
    report = {'robots': {n: {} for n in coordinator.ROBOTS}, 'commands_sent': 0}
    args = SimpleNamespace(observe_seconds=10.0, start_tolerance=.05, max_stationary_speed=.01)
    return node, data, report, args


def ready(node, wall_s, *, missing=()):
    node.clock_times = [0.0, .6, 1.2]
    node.clock_progress_wall = wall_s
    for n in coordinator.ROBOTS:
        if n not in missing:
            node.odoms[n] = {'xy': (0.0, 0.0), 'speed': 0.0, 'frame': f'{n}/odom', 'stamp': 1.2}


class TestReadinessGates(unittest.TestCase):
    def test_ready_preserves_original_thresholds(self):
        node, data, report, a = inputs()
        ready(node, 1)
        self.assertEqual(coordinator.check_initial_readiness(node, data, a, report, now_wall=1.2, adaptive=True), 1.2)
        self.assertEqual(set(report['robots']), set(coordinator.ROBOTS))

    def test_clock_absent_or_not_advancing(self):
        node, data, report, a = inputs()
        with self.assertRaisesRegex(RuntimeError, 'CLOCK_NOT_ADVANCING'):
            coordinator.check_initial_readiness(node, data, a, report, now_wall=1, adaptive=True)
        ready(node, .1)
        node.clock_times = [1.2, 1.2, 1.2]
        with self.assertRaisesRegex(RuntimeError, 'CLOCK_NOT_ADVANCING'):
            coordinator.check_initial_readiness(node, data, a, report, now_wall=.2, adaptive=True)

    def test_stale_clock_after_initial_advance(self):
        node, data, report, a = inputs()
        ready(node, 0.1)
        with self.assertRaisesRegex(RuntimeError, 'CLOCK_NOT_ADVANCING'):
            coordinator.check_initial_readiness(node, data, a, report, now_wall=2.2, adaptive=True)
        # Normal profile continues using its original check (no new clock-age gate).
        self.assertEqual(coordinator.check_initial_readiness(node, data, a, report), 1.2)

    def test_missing_odom(self):
        node, data, report, a = inputs()
        ready(node, 1, missing={'summit0'})
        with self.assertRaisesRegex(RuntimeError, 'summit0: ODOM_MISSING'):
            coordinator.check_initial_readiness(node, data, a, report, now_wall=1, adaptive=True)

    def test_stale_odom(self):
        node, data, report, a = inputs()
        ready(node, 1)
        node.odoms['summit1']['stamp'] = .1
        with self.assertRaisesRegex(RuntimeError, 'summit1: ODOM_STALE'):
            coordinator.check_initial_readiness(node, data, a, report, now_wall=1, adaptive=True)

    def test_owner_wrong_or_duplicate(self):
        node, data, report, a = inputs()
        ready(node, 1)
        node.publishers['summit0'].append(SimpleNamespace(node_name='other', node_namespace='/summit0'))
        with self.assertRaisesRegex(RuntimeError, 'summit0: CMD_VEL_OWNER_MISMATCH'):
            coordinator.check_initial_readiness(node, data, a, report, now_wall=1, adaptive=True)

    def test_position_speed_and_frame_still_checked(self):
        for change in ({'xy': (.10, 0.0)}, {'speed': .02}, {'frame': '/map'}):
            with self.subTest(change=change):
                node, data, report, a = inputs()
                ready(node, 1)
                node.odoms['summit2'].update(change)
                with self.assertRaisesRegex(RuntimeError, 'summit2: ALIGNMENT_OR_STATIONARY_CHECK_FAILED'):
                    coordinator.check_initial_readiness(node, data, a, report, now_wall=1, adaptive=True)


class TestAdaptiveWaiting(unittest.TestCase):
    def test_early_exit_without_waiting_full_budget(self):
        node, data, report, a = inputs()
        wall = [0.0]

        def spin(seconds):
            wall[0] += seconds
            if wall[0] >= .8:
                ready(node, wall[0])

        self.assertEqual(coordinator.await_adaptive_readiness(node, data, a, report, spin, now=lambda: wall[0]), 1.2)
        self.assertLess(wall[0], 1.0)
        self.assertEqual(report['readiness_observation']['result'], 'READY')
        self.assertEqual(report['commands_sent'], 0)

    def test_slow_odom_recovers_before_deadline(self):
        node, data, report, a = inputs()
        wall = [0.0]

        def spin(seconds):
            wall[0] += seconds
            if wall[0] >= 1:
                ready(node, wall[0], missing={'summit0'} if wall[0] < 7.5 else ())

        self.assertEqual(coordinator.await_adaptive_readiness(node, data, a, report, spin, now=lambda: wall[0]), 1.2)
        self.assertGreaterEqual(wall[0], 7.5)
        self.assertLess(wall[0], 10.0)

    def test_timeout_remains_fail_closed(self):
        node, data, report, a = inputs()
        wall = [0.0]
        with self.assertRaisesRegex(RuntimeError, 'CLOCK_NOT_ADVANCING'):
            coordinator.await_adaptive_readiness(node, data, a, report, lambda secs: wall.__setitem__(0, wall[0]+secs), now=lambda: wall[0])
        self.assertEqual(report['readiness_observation']['result'], 'TIMEOUT')
        self.assertGreaterEqual(wall[0], 10.0)
        self.assertEqual(report['commands_sent'], 0)

    def test_timeout_reports_last_missing_robot(self):
        node, data, report, a = inputs()
        wall = [0.0]

        def spin(secs):
            wall[0] += secs
            ready(node, wall[0], missing={'summit0'})

        with self.assertRaisesRegex(RuntimeError, 'summit0: ODOM_MISSING'):
            coordinator.await_adaptive_readiness(node, data, a, report, spin, now=lambda: wall[0])
        self.assertEqual(report['readiness_observation']['last_problem'], 'summit0: ODOM_MISSING')

    def test_safety_watchdogs_and_arm_barrier_stay_in_source(self):
        source = COORD.read_text()
        for gate in ('ARM_BARRIER_TIMEOUT', 'FOLLOWER_REJECTED_START', 'SIM_CLOCK_STALLED_20_WALL_SECONDS',
                     'FOLLOWER_WALL_WATCHDOG_TIMEOUT', 'SIM_CLOCK_JUMPED_BACKWARD',
                     'FOLLOWER_COMPLETION_TIMEOUT_SIM_CLOCK', 'START_DELAY_OUT_OF_RANGE'):
            with self.subTest(gate=gate):
                self.assertIn(gate, source)


if __name__ == '__main__':
    unittest.main()
