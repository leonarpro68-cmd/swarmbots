import unittest,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d98c7_phase_retime import analyze
from d98c7_d92_verifier import verify
class Test(unittest.TestCase):
 def test_stationary_safe(self):
  cs=[np.tile(np.array([[[i*5.,0.]]]),(1,6,1)) for i in range(3)]
  ok,cert=verify(cs,[1]*3,[1]*3,[(0.,0.)]*3,1.2,1.,1.15,.1)
  self.assertTrue(ok)
 def test_stationary_collision(self):
  cs=[np.zeros((1,6,2)) for _ in range(3)]
  ok,_=verify(cs,[1]*3,[1]*3,[(0.,0.)]*3,1.2,1.,1.15,.1)
  self.assertFalse(ok)
 def test_bad_preview(self):
  with self.assertRaises(ValueError):analyze({'status':'INVALID'}, {},.08)
 def test_bad_step(self):
  with self.assertRaises(ValueError):analyze({'status':'BEZIER_PREVIEW_LIMITS_PASS_NOT_EXECUTABLE','execution_authorized':False}, {},.2)
if __name__=='__main__':unittest.main()
