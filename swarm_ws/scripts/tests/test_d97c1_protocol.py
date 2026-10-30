"""Regression tests for simulated D9.7C.1 normal completion contract.
No ROS or Gazebo needed."""
import ast
import pathlib
import unittest

ROOT=pathlib.Path(__file__).resolve().parents[1]

class TestProtocolContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.relay= (ROOT/'d97c_watchdog3.py').read_text()
        cls.follower= (ROOT/'d97c_follower.py').read_text()
    def test_parseable(self):
        for name in ('d97c_watchdog3.py','d97c_follower.py'):
            ast.parse((ROOT/name).read_text())
    def test_both_use_same_stop_topic(self):
        self.assertIn('/d97c/session_end',self.relay)
        self.assertIn('/d97c/session_end',self.follower)
    def test_same_stop_token(self):
        self.assertIn('D97C_STOP_ALL',self.relay)
        self.assertIn('D97C_STOP_ALL',self.follower)
    def test_requires_measured_stopping(self):
        self.assertIn('self.stopped_samples>=8',self.relay)
        self.assertIn('stop_samples>=8',self.follower)
    def test_timeout_remains(self):
        self.assertIn("':INPUT_TIMEOUT'",self.relay)
        self.assertIn('now-self.msgstamp[r]>.30',self.relay)
    def test_end_not_issued_on_fault(self):
        self.assertIn("if stop_confirmed and state in ('SHORT_CHECKOUT_COMPLETED','FINISHED_TRAJECTORY_NOT_MISSION')",self.follower)
    def test_full_goal_gate(self):
        self.assertIn('FINAL_GOAL_NOT_REACHED',self.follower)
    def test_only_gazebo_explicit(self):
        self.assertIn('--enable-sim-relay',self.relay)
        self.assertIn('--execute-in-gazebo',self.follower)

if __name__=='__main__':unittest.main()
