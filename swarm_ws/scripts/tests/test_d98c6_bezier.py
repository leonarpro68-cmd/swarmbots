import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from d98c6_bezier_phase_preview import controls_for_path,derivative_bounds,evaluate,make_phase
class TestD98C6(unittest.TestCase):
 def test_stop_edges(self):
  c=controls_for_path([[0,0],[1,0]])
  self.assertTrue(np.allclose(c[0,:3],0))
  self.assertTrue(np.allclose(c[0,3:],[1,0]))
 def test_endpoints(self):
  c=controls_for_path([[0,0],[2,0]])
  self.assertTrue(np.allclose(evaluate(c,[5.],0),[0,0]))
  self.assertTrue(np.allclose(evaluate(c,[5.],5),[2,0]))
 def test_stationary(self):
  r,c,d,_=make_phase('wave2',{'summit1':[[5,5],[5,6]]},{'summit0':[0,0],'summit1':[5,5],'summit2':[-5,-5]})
  self.assertTrue(r['limits_preview_passed'])
  self.assertEqual(len(c['summit0']),1)
 def test_reject_bad_limit(self):
  with self.assertRaises(ValueError):make_phase('wave2',{'summit1':[[5,5],[5,6]]},{'summit0':[0,0],'summit1':[5,5],'summit2':[-5,-5]},step_duration=0)
if __name__=='__main__':unittest.main()
