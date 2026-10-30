"""ROS-independent regression tests for bounded D8.6 near-field handoff."""
import ast
import math
import types
import numpy as np
import unittest
from pathlib import Path

SOURCE = (Path(__file__).resolve().parents[1] / 'swarm_ws/src/swarm_behavior/swarm_behavior/visual_grasp.py')


def load_selected_functions():
    asttree = ast.parse(SOURCE.read_text(encoding='utf-8'))
    picked = [n for n in asttree.body if isinstance(n, ast.FunctionDef) and n.name == '_near_field_geometry']
    cl = next(n for n in asttree.body if isinstance(n, ast.ClassDef) and n.name == 'VisualGrasp')
    picked.extend(n for n in cl.body if isinstance(n, ast.FunctionDef) and n.name in ('_nearfield_handoff', '_step'))
    compiled = ast.fix_missing_locations(ast.Module(body=picked, type_ignores=[]))
    ns = {'math': math, 'np': np, 'Twist': FakeTwist}
    exec(compile(compiled, str(SOURCE), 'exec'), ns)
    return ns


class FakeTwist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0)


class FakePublisher:
    def __init__(self):
        self.msgs = []
    def publish(self, msg):
        self.msgs.append(msg)


class FakeTime:
    def __init__(self, ns):
        self.nanoseconds = ns

    def __sub__(self, other):
        return FakeTime(self.nanoseconds - other.nanoseconds)


class FakeLogger:
    def __init__(self):
        self.warnings = []
    def warn(self, msg, **kwargs):
        self.warnings.append(msg)
    def error(self, msg, **kwargs):
        self.warnings.append(msg)


class GateTests(unittest.TestCase):
    def setUp(self):
        self.fn = load_selected_functions()
        self.now = FakeTime(10_000_000_000)
        rx, ry, yaw = -8.403964781862577, -19.67541924717309, -2.404192064594712
        self.pose = (rx, ry, yaw)
        self.target = (-8.75, -20)

    def fake_node(self):
        n = types.SimpleNamespace(
            near_field_enabled=True, pose=self.pose, target=self.target,
            camera_forward=0.35, near_min_range=0.43, near_max_range=0.50,
            near_max_lateral=0.045, near_max_camera_depth=0.16,
            sensor_timeout=0.5, odom_stamp=FakeTime(9_900_000_000),
            near_recent_vision_s=3.0,
            near_vision_history=[(8_700_000_000, -21, .34),
                                 (8_900_000_000, -19, .33),
                                 (9_400_000_000, -21, .311)],
            get_logger=lambda: FakeLogger(), result=None)
        n._finish = lambda value: setattr(n, 'result', value)
        return n

    def test_logged_pose_is_inside_geometric_envelope(self):
        ok, distance, lateral, camera = self.fn['_near_field_geometry'](
            self.pose, self.target, .35, .43, .50, .045, .16)
        self.assertTrue(ok)
        self.assertAlmostEqual(distance, .47444, places=4)
        self.assertLess(abs(lateral), .01)
        self.assertAlmostEqual(camera, .12438, places=3)

    def test_success_requires_recent_rgbd_history_and_fresh_odom(self):
        n = self.fake_node()
        self.assertTrue(self.fn['_nearfield_handoff'](n, self.now))
        self.assertEqual(n.result, 1)

    def test_no_handoff_outside_gripper_envelope(self):
        n = self.fake_node()
        n.pose = (-8.75, -19.1, -math.pi / 2)  # robot center 0.9 m away
        self.assertFalse(self.fn['_nearfield_handoff'](n, self.now))
        self.assertIsNone(n.result)

    def test_no_handoff_with_wrong_lateral_alignment(self):
        n = self.fake_node()
        n.pose = (n.pose[0], n.pose[1], n.pose[2] + .38)
        self.assertFalse(self.fn['_nearfield_handoff'](n, self.now))

    def test_no_handoff_with_stale_or_missing_vision(self):
        n = self.fake_node()
        n.near_vision_history = [(6_000_000_000, 0, .2),
                                 (6_100_000_000, 0, .2),
                                 (6_200_000_000, 0, .2)]
        self.assertFalse(self.fn['_nearfield_handoff'](n, self.now))
        n.near_vision_history = []
        self.assertFalse(self.fn['_nearfield_handoff'](n, self.now))

    def test_no_handoff_with_stale_odometry(self):
        n = self.fake_node()
        n.odom_stamp = FakeTime(8_000_000_000)
        self.assertFalse(self.fn['_nearfield_handoff'](n, self.now))

    def test_disabled_stays_disabled(self):
        n = self.fake_node()
        n.near_field_enabled = False
        self.assertFalse(self.fn['_nearfield_handoff'](n, self.now))

    def test_single_spurious_depth_frame_not_enough(self):
        n = self.fake_node()
        n.near_vision_history = [(9_400_000_000, -21, .311)]
        self.assertFalse(self.fn['_nearfield_handoff'](n, self.now))
        n.near_vision_history = [(9_400_000_000, -21, .311),
                                 (9_450_000_000, -20, .315),
                                 (9_480_000_000, -19, .311)]
        self.assertFalse(self.fn['_nearfield_handoff'](n, self.now))


    def test_lost_depth_never_commands_blind_motion(self):
        n = self.fake_node()
        n.now = self.now
        n.get_clock = lambda: types.SimpleNamespace(now=lambda: n.now)
        n.get_logger = lambda: FakeLogger()
        n.active = True
        n.started = FakeTime(0)
        n.timeout = 15.0
        n.depth = np.full((100, 100), np.nan)
        n.depth_stamp = FakeTime(9_990_000_000)
        n.fov = 1.047
        n.good_frames = 0
        n.near_missing_since = None
        n.near_depth_loss_s = 0.75
        n.cmd_pub = FakePublisher()
        n._nearfield_handoff = types.MethodType(self.fn['_nearfield_handoff'], n)
        step = self.fn['_step']
        step(n)
        self.assertIsNone(n.result)
        self.assertEqual(len(n.cmd_pub.msgs), 1)
        self.assertEqual(n.cmd_pub.msgs[0].linear.x, 0.0)
        n.now = FakeTime(10_800_000_000)
        n.depth_stamp = n.now
        n.odom_stamp = n.now
        step(n)
        self.assertEqual(n.result, 1)
        self.assertEqual(len(n.cmd_pub.msgs), 2)
        self.assertTrue(all(m.linear.x == m.linear.y == 0.0 for m in n.cmd_pub.msgs))

    def test_launch_retains_all_prior_fixes(self):
        src = (SOURCE.parents[2] / 'swarm_worlds/launch/swarm_cbs_grasp_base.launch.py').read_text()
        self.assertIn("'hybrid_enabled': hybrid_enabled", src)
        self.assertIn("'max_speed': hybrid_max_speed if hybrid_enabled else 0.60", src)
        self.assertIn("'attach_timeout': 5.0", src)
        self.assertIn("'near_field_handoff_enabled': True", src)


if __name__ == '__main__':
    unittest.main()
