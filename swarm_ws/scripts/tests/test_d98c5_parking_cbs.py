import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import d98c5_parking_cbs as m
class Tests(unittest.TestCase):
 def test_api(self):self.assertTrue(callable(m.build))
 def test_contract(self):
  self.assertTrue(m.CLEARANCE>1)
  self.assertEqual(len(m.ROBOTS),3)
if __name__=='__main__':unittest.main()
