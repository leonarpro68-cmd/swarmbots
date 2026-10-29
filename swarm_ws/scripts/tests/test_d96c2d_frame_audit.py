import sys, math, unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d96c2d_frame_dynamics_audit import yaw_from_q,audit_samples
R=('summit0','summit1','summit2')
class AuditTests(unittest.TestCase):
    def setUp(self):
        self.starts={r:(i*4.,0.) for i,r in enumerate(R)}
        self.samples={r:[{'x':i*4.,'y':0.,'yaw':0.,'vx':0.,'vy':0.,'frame_id':r+'/odom','child_frame_id':r+'/base_footprint','age_wall_s':0.05} for _ in range(3)] for i,r in enumerate(R)}
        self.limits={'max_drift_m':.025,'max_start_error_m':.05,'max_speed_m_s':.03,'max_yaw_drift_rad':.05,'min_separation_m':1.15}
    def test_quaternion(self):
        self.assertAlmostEqual(yaw_from_q((0,0,1,0)),math.pi)
    def test_accept_stationary_unproven(self):
        s,d=audit_samples(self.samples,self.starts,[3,3.4],self.limits)
        self.assertEqual(s,'PASS_PASSIVE_AUDIT_FRAMES_UNPROVEN')
    def test_detect_drift(self):
        self.samples['summit0'][-1]['x']=.1
        self.assertEqual(audit_samples(self.samples,self.starts,[3,3.4],self.limits)[0],'REJECT_PASSIVE_AUDIT')
    def test_detect_frame(self):
        self.samples['summit0'][0]['frame_id']='map'
        self.assertEqual(audit_samples(self.samples,self.starts,[3,3.4],self.limits)[0],'REJECT_PASSIVE_AUDIT')
    def test_detect_clock_reset(self):
        self.assertEqual(audit_samples(self.samples,self.starts,[3,2,4],self.limits)[0],'REJECT_PASSIVE_AUDIT')
    def test_detect_speed(self):
        self.samples['summit1'][1]['vx']=.2
        self.assertEqual(audit_samples(self.samples,self.starts,[3,3.4],self.limits)[0],'REJECT_PASSIVE_AUDIT')
    def test_detect_yaw(self):
        self.samples['summit1'][1]['yaw']=.2
        self.assertEqual(audit_samples(self.samples,self.starts,[3,3.4],self.limits)[0],'REJECT_PASSIVE_AUDIT')
if __name__=='__main__':unittest.main()
