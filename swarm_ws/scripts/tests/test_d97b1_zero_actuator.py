import importlib.util
import unittest
from pathlib import Path
P=Path(__file__).resolve().parents[1]/'d97b1_zero_actuator.py'
spec=importlib.util.spec_from_file_location('d97b1',P)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class SafetyTest(unittest.TestCase):
 def setUp(self):
  self.obs={f'summit{i}':[(0,0),(0.001,0)] for i in range(3)}
  self.graph={t:[] for t in m.WATCH_TOPICS}
 def test_normal(self):self.assertFalse(m.decide(self.obs,self.graph,5))
 def test_conflict(self):
  self.graph['/summit1/cmd_vel']=['follower'];self.assertIn('EXTERNAL_PUBLISHER:/summit1/cmd_vel',m.decide(self.obs,self.graph,5))
 def test_drift(self):
  self.obs['summit0'][-1]=(0.04,0);self.assertIn('DRIFT:summit0',m.decide(self.obs,self.graph,5))
 def test_clock(self):self.assertIn('CLOCK_MISSING',m.decide(self.obs,self.graph,1))
 def test_missing(self):
  self.obs.pop('summit2');self.assertIn('MISSING_ODOMETRY',m.decide(self.obs,self.graph,5))
 def test_short_odom(self):
  self.obs['summit2']=[(0,0)];self.assertIn('INSUFFICIENT_ODOM:summit2',m.decide(self.obs,self.graph,5))
if __name__=='__main__':unittest.main()
