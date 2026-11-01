import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from prepare_phase5 import prepare,audit_local,sha

PHASE4=ROOT.parent/'supermarket_milp_hybrid_v2_v3'
if not PHASE4.exists():
    PHASE4=ROOT.parent/'supermarket_milp_hybrid_v2_v3_fase4_v2'

class Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.paths={
          'cbs':PHASE4/'reference_cbs_seed100.json',
          'scenario':PHASE4/'seed_100_reference.json',
          'geometry':PHASE4/'supermarket_obstaculos.json',
          'results':PHASE4/'reference_outputs',
        }
        for name,path in cls.paths.items():
            if not path.exists():raise unittest.SkipTest(f'Referencia fase4 ausente: {name}={path}')
        cls.cbs=json.loads(cls.paths['cbs'].read_text())
        cls.scene=json.loads(cls.paths['scenario'].read_text())
        cls.geometry=json.loads(cls.paths['geometry'].read_text())

    def call(self,cbs=None,scene=None,geometry=None,results=None):
        p=self.paths
        return prepare(cbs or self.cbs,scene or self.scene,geometry or self.geometry,results or p['results'],p['cbs'],p['scenario'],p['geometry'])

    def test_four_stages_twelve_rows_pass(self):
        rep,assign=self.call()
        self.assertEqual(rep['status'],'PASS_STATIC_PRECHECK_NOT_EXECUTABLE')
        self.assertFalse(rep['execution_authorized'])
        self.assertFalse(rep['physical_D8_reach_verified'])
        self.assertEqual(len(rep['d8_tasks']),12)
        self.assertEqual(len(assign),4)

    def test_expected_milp_allocation(self):
        rep,assign=self.call()
        self.assertEqual(assign['pickup1']['robots']['summit0']['trash'],'trash_V1_verde')
        self.assertEqual(assign['pickup2']['robots']['summit2']['trash'],'trash_A2_azul')
        self.assertEqual(assign['delivery2']['robots']['summit0']['target'],'deposit_naranja')

    def test_unsafe_goal_d8_rejected(self):
        x=copy.deepcopy(self.cbs)
        x['stages']['pickup1']['goals_grid'][0]=[20,40]
        rep,_=self.call(cbs=x)
        self.assertEqual(rep['status'],'REJECTED_STATIC_PRECHECK')
        self.assertTrue(rep['errors'])

    def test_duplicate_assignment_rejected(self):
        x=copy.deepcopy(self.cbs)
        x['assignment']['summit0']['pickup2']=x['assignment']['summit0']['pickup1']
        rep,_=self.call(cbs=x)
        self.assertEqual(rep['status'],'REJECTED_STATIC_PRECHECK')
        self.assertTrue(any('repetidas' in e for e in rep['errors']))

    def test_shelf_collision_rejected(self):
        out=audit_local([9.0,0.0],[11.0,0.0],'pickup1','trash_A1_azul',[],self.scene,self.geometry)
        self.assertFalse(out['straight_line_geometric_screen_pass'])
        self.assertTrue(out['shelf_hits'])

    def test_clock_hash_mismatch_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            for stage in ('pickup1','delivery1','pickup2','delivery2'):
                folder=root/stage;folder.mkdir()
                for p in (self.paths['results']/stage).glob('*'):
                    (folder/p.name).symlink_to(p)
            cl=root/'pickup1'/'pickup1_MILP_V3_clock.json'
            d=json.loads(cl.read_text());cl.unlink();d['input_sha256']='0'*64;cl.write_text(json.dumps(d))
            rep,_=self.call(results=root)
            self.assertEqual(rep['status'],'REJECTED_STATIC_PRECHECK')
            self.assertTrue(any('reloj V3' in e for e in rep['errors']))

if __name__=='__main__':unittest.main()
