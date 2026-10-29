import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d98c1_delivery_snapshot import assess
N=['trash_V2_verde','trash_V1_verde','trash_A2_azul']
P={f'summit{i}':[10*i,0,0,0.001] for i in range(3)}
O={n:[10*i+.45,0] for i,n in enumerate(N)}
class Tests(unittest.TestCase):
 def test_valid(self):
  r=assess(N,P,O);self.assertEqual(r['status'],'LOAD_GEOMETRY_COMPATIBLE_NOT_JOINT_CERTIFIED');self.assertFalse(r['delivery1_execution_authorized'])
 def test_far(self):
  o={**O,N[0]:[3,0]};self.assertIn('summit0:OBJECT_NOT_NEAR_TIP',assess(N,P,o)['issues'])
 def test_moving(self):
  p={**P,'summit0':[0,0,0,.2]};self.assertIn('summit0:ROBOT_MOVING',assess(N,p,O)['issues'])
 def test_missing(self):
  self.assertIn('summit0:NO_FRESH_ODOMETRY',assess(N,{},O)['issues'])
 def test_duplicate(self):
  self.assertIn('INVALID_ASSIGNED_OBJECTS',assess([N[0],N[0],N[2]],P,O)['issues'])
if __name__=='__main__':unittest.main()
