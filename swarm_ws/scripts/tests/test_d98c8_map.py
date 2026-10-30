import sys, unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d98c8_hybrid_map_audit import scene,bezier
import numpy as np
class Tests(unittest.TestCase):
 def test_scene(self):
  b,o=scene({'extent':[-2,-2,2,2],'obstacles':[{'outer':[[0,0],[1,0],[1,1],[0,1]]}]})
  self.assertTrue(b.covers(o))
 def test_invalid_extent(self):
  with self.assertRaises(ValueError):scene({'extent':[0,1]})
 def test_bezier(self):
  a=np.array([[0.,0.]]*3+[[1.,1.]]*3)
  np.testing.assert_allclose(bezier(a,0),[0,0]);np.testing.assert_allclose(bezier(a,1),[1,1])
 def test_outside(self):
  b,_=scene({'extent':[-2,-2,2,2],'obstacles':[]})
  self.assertFalse(b.covers(__import__('shapely').geometry.Point(3,0).buffer(.6)))
if __name__=='__main__':unittest.main()
