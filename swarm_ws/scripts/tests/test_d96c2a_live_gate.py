import sys, unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d96c2a_live_start_gate import evaluate_starts

ROBOT=('summit0','summit1','summit2')
def fixture(d=0.):
    nominal={n:(10*i,0.) for i,n in enumerate(ROBOT)}
    captured={n:(10*i+.12,0.) for i,n in enumerate(ROBOT)}
    refs={n:{'x_m':10*i+.12,'y_m':0.} for i,n in enumerate(ROBOT)}
    live={n:{'x_m':10*i+.12+d,'y_m':0.,'frame_id':f'{n}/odom',
             'child_frame_id':f'{n}/base_footprint','quaternion_norm':1.,
             'yaw_rad':0.,'age_wall_s':0.01} for i,n in enumerate(ROBOT)}
    return nominal,refs,captured,live

class GateTests(unittest.TestCase):
    def test_matches_snapshot_not_motion(self):
        result=evaluate_starts(*fixture(.01))
        self.assertEqual(result['status'],'ARCHIVED_START_MATCHES_LIVE_READ_ONLY')
        self.assertFalse(result['execution_authorized'])
        self.assertFalse(result['coordinate_frame_equivalence_proven'])
    def test_stationary_relaunch_offset_blocked(self):
        result=evaluate_starts(*fixture(.12))
        self.assertEqual(result['status'],'REQUIRES_LIVE_REPLAN')
    def test_original_capture_mismatch_blocks(self):
        a,b,c,d=fixture(0.)
        c['summit0']=(0.,0.)
        self.assertEqual(evaluate_starts(a,b,c,d)['status'],'REQUIRES_LIVE_REPLAN')
    def test_bad_frame_blocks(self):
        a,b,c,d=fixture(0.)
        d['summit1']['frame_id']='world'
        self.assertEqual(evaluate_starts(a,b,c,d)['status'],'REQUIRES_LIVE_REPLAN')
    def test_stale_blocks(self):
        a,b,c,d=fixture(0.)
        d['summit1']['age_wall_s']=3.
        self.assertEqual(evaluate_starts(a,b,c,d)['status'],'REQUIRES_LIVE_REPLAN')

if __name__=='__main__':unittest.main()
