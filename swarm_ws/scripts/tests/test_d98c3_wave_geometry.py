import unittest,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parents[1]))
from d98c3_wave_geometry import point_segment,segments,validate
class TestGeometry(unittest.TestCase):
 def test_point_segment(self):self.assertAlmostEqual(point_segment((2,1),(0,0),(4,0)),1)
 def test_segments_cross(self):self.assertEqual(segments((0,0),(2,2),(0,2),(2,0)),0)
 def test_segments_parallel(self):self.assertAlmostEqual(segments((0,0),(2,0),(0,3),(2,3)),3)
 def sample(self):
  snap={'status':'LOAD_GEOMETRY_COMPATIBLE_NOT_JOINT_CERTIFIED','issues':[],'robots':{f'summit{i}':{'object':o,'odom_xy_m':pos} for i,(o,pos) in enumerate([('trash_V2_verde',[0,0]),('trash_V1_verde',[0,20]),('trash_A2_azul',[20,0])])}}
  waves={'status':'PROPOSED_WAVES_ONLY_BLOCKED_FOR_EXECUTION','waves':[{'number':1,'robots':[{'robot':'summit0','object':'trash_V2_verde','deposit_color':'verde'},{'robot':'summit2','object':'trash_A2_azul','deposit_color':'azul'}]},{'number':2,'robots':[{'robot':'summit1','object':'trash_V1_verde','deposit_color':'verde'}]}]}
  scenario={'elements':[{'name':'deposit_verde','type':'deposit','color':'verde','xy_m':[2,1]},{'name':'deposit_azul','type':'deposit','color':'azul','xy_m':[22,1]}]}
  return snap,waves,scenario
 def test_results_blocked(self):
  result=validate(*self.sample());self.assertFalse(result['cbs_routes_computed']);self.assertEqual(len(result['waves']),2)
 def test_wrong_cargo_rejected(self):
  a,b,c=self.sample();b['waves'][0]['robots'][0]['object']='wrong'
  with self.assertRaisesRegex(ValueError,'CARGO_ID_MISMATCH'):validate(a,b,c)
if __name__=='__main__':unittest.main()
