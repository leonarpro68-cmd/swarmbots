import importlib.util
import unittest
from pathlib import Path

FILE = Path(__file__).resolve().parents[1]/'swarm_ws'/'scripts'/'d6_grasp_existing.py'
spec = importlib.util.spec_from_file_location('d6_existing', FILE)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

class Tests(unittest.TestCase):
    def setUp(self):
        self.odom={n:{'xy':(10*i,0), 'yaw':0.0} for i,n in enumerate(m.ROBOTS)}
        self.json={'trash_0':(1.5,0)}
        self.actual=dict(self.json)
    def test_normal_preapproach_not_at_tip_yet(self):
        x=m.check_preapproach('summit0','trash_0',self.odom,self.json,self.actual)
        self.assertAlmostEqual(x['center_distance_m'],1.5)
        self.assertEqual(x['robot'],'summit0')
    def test_reject_far(self):
        with self.assertRaisesRegex(ValueError,'fuera'):
            m.check_preapproach('summit0','trash_0',self.odom,{'trash_0':(2.4,0)}, {'trash_0':(2.4,0)})
    def test_reject_moved_trash(self):
        with self.assertRaisesRegex(ValueError,'desplazada'):
            m.check_preapproach('summit0','trash_0',self.odom,self.json,{'trash_0':(1.9,0)})
    def test_reject_other_robot_near_approach(self):
        self.odom['summit1']['xy']=(1.0,.4)
        with self.assertRaisesRegex(ValueError,'aproximación'):
            m.check_preapproach('summit0','trash_0',self.odom,self.json,self.actual)
    def test_assigned_name_follows_cbs(self):
        report={'stages': {'pickup1': {'certified': True,
             'hybrid_v2':{'certified':True,'scenario_json':'/tmp/scenario.json'},
             'assigned_names':['trash_0','trash_1','trash_2']}}}
        self.assertEqual(m.assigned_target(report,'pickup1','summit2',Path('/tmp/scenario.json')),'trash_2')

if __name__=='__main__':unittest.main()
