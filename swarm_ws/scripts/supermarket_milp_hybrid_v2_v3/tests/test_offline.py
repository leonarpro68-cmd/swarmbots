import json,sys,unittest,tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from audit_phase4 import create_mission
from run_phase4 import config
from audit_d8_proximity import run as d8run
from verify_offline import verify
class OfflineContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cbs=ROOT/'reference_cbs_seed100.json'
        cls.scene=ROOT/'seed_100_reference.json'
        cls.geometry=ROOT/'supermarket_obstaculos.json'
    def test_corridors(self):
        scene,geo,mission,audit=config(self.cbs,self.scene,self.geometry)
        self.assertTrue(audit['pass_all']);self.assertEqual(len(mission['best']['stages']),4)
    def test_assignment(self):
        mission=create_mission(json.loads(self.cbs.read_text()))
        picks=[p for s in mission['best']['stages'] if s['stage'].startswith('pickup') for p in s['targets']]
        self.assertEqual(len(picks),6);self.assertEqual(len(set(picks)),6)
    def test_d8_not_certified(self):
        d8=d8run(json.loads(self.cbs.read_text()),json.loads(self.scene.read_text()),json.loads(self.geometry.read_text()))
        self.assertEqual(len(d8['stages']),4)
        self.assertFalse(any(x['d8_reach_verified'] for rows in d8['stages'].values() for x in rows))
    def test_summit_limits(self):
        from clearance_config import RECTANGLE_CENTER_CLEARANCE_M,INTERROBOT_CENTER_SEPARATION_M, TRASH_CENTER_EXCLUSION_M
        self.assertAlmostEqual(RECTANGLE_CENTER_CLEARANCE_M,0.60)
        self.assertAlmostEqual(INTERROBOT_CENTER_SEPARATION_M,1.15)
        self.assertAlmostEqual(TRASH_CENTER_EXCLUSION_M,0.55+0.05+(2**.5)*0.09/2)
    def test_unknown_cbs_rejected(self):
        o=json.loads(self.cbs.read_text());o['status']='EXECUTABLE'
        with self.assertRaises(ValueError):create_mission(o)
    def test_reference_four_stages(self):
        out=verify(ROOT/'reference_outputs')
        self.assertEqual(len(out['stages']),4)
        self.assertEqual(out['status'],'PASS_OFFLINE_ARTIFACTS_NOT_EXECUTABLE',out['problems'])
if __name__=='__main__':unittest.main()
