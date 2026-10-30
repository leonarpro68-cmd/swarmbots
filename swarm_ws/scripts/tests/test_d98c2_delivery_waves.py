import unittest,copy,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d98c2_delivery_waves import make_waves
S={'status':'LOAD_GEOMETRY_COMPATIBLE_NOT_JOINT_CERTIFIED','issues':[],'robots':{r:{'object':n,'distance_compatible_with_load':True,'observed_speed_m_s':0} for r,n in [('summit0','trash_V2_verde'),('summit1','trash_V1_verde'),('summit2','trash_A2_azul')]}}
class Tests(unittest.TestCase):
 def test_two_waves(self):
  x=make_waves(S);self.assertEqual([[p['robot'] for p in w['robots']] for w in x['waves']],[['summit2','summit0'],['summit1']]);self.assertFalse(x['release_authorized'])
 def test_duplicate(self):
  x=copy.deepcopy(S);x['robots']['summit1']['object']='trash_V2_verde'
  with self.assertRaises(ValueError):make_waves(x)
 def test_moving(self):
  x=copy.deepcopy(S);x['robots']['summit1']['observed_speed_m_s']=0.2
  with self.assertRaises(ValueError):make_waves(x)
 def test_blocked(self):
  x=copy.deepcopy(S);x['issues']=['not fresh']
  with self.assertRaises(ValueError):make_waves(x)
if __name__=='__main__':unittest.main()
