import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d96c2c_prearm_readonly import decide

NAMES=('summit0','summit1','summit2')
def fixtures():
    starts={n:{'x_m':i*3.,'y_m':0.} for i,n in enumerate(NAMES)}
    live={n:{'x_m':i*3.,'y_m':0.,'vx_m_s':0.,'vy_m_s':0.,'frame_id':f'{n}/odom',
             'child_frame_id':f'{n}/base_footprint','age_wall_s':.01,'quaternion_norm':1.} for i,n in enumerate(NAMES)}
    return {'snapshot_time_s':10.}, starts,live,{'current_sim_s':15.,'progressing':True}
class TestPrearm(unittest.TestCase):
    def test_pass_non_executable(self):
        self.assertEqual(decide(*fixtures())[0],'PASS_SHADOW_PREARM_ONLY')
    def test_offset(self):
        r,s,l,c=fixtures();l['summit0']['x_m']+=.12
        self.assertEqual(decide(r,s,l,c)[0],'REJECT_START_OR_ODOMETRY')
    def test_stale(self):
        r,s,l,c=fixtures();c['current_sim_s']=500.
        self.assertEqual(decide(r,s,l,c)[0],'REJECT_STALE_OR_RESET_CAPTURE')
    def test_clock_pause(self):
        r,s,l,c=fixtures();c['progressing']=False
        self.assertEqual(decide(r,s,l,c)[0],'REJECT_CLOCK_NOT_PROGRESSING')
    def test_frame(self):
        r,s,l,c=fixtures();l['summit1']['frame_id']='map'
        self.assertEqual(decide(r,s,l,c)[0],'REJECT_START_OR_ODOMETRY')
    def test_speed(self):
        r,s,l,c=fixtures();l['summit2']['vx_m_s']=.15
        self.assertEqual(decide(r,s,l,c)[0],'REJECT_START_OR_ODOMETRY')
    def test_missing(self):
        r,s,l,c=fixtures();del l['summit2']
        self.assertEqual(decide(r,s,l,c)[0],'REJECT_MISSING_ODOMETRY')
if __name__=='__main__':unittest.main()
