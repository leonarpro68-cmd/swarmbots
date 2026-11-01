import unittest
from pathlib import Path
from audit_phase4 import create_mission,read,check_corridors
HERE=Path(__file__).resolve().parent
class Tests(unittest.TestCase):
 def test_four_stages(self):
  m=create_mission(read(HERE/'reference_milp_cbs_seed100.json'))
  self.assertEqual([x['stage'] for x in m['best']['stages']],['pickup1','delivery1','pickup2','delivery2'])
 def test_blocks_uncertified(self):
  c=read(HERE/'reference_milp_cbs_seed100.json');m=create_mission(c)
  rep=check_corridors(read(HERE/'reference_seed100.json'),m,read(HERE/'supermarket_obstaculos.json'))
  self.assertFalse(rep['pass_all'])
  self.assertEqual(len(rep['stages']),4)
 def test_never_authorizes(self):
  self.assertFalse(read(HERE/'reference_milp_cbs_seed100.json')['execution_authorized'])
if __name__=='__main__': unittest.main()
