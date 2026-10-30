import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
# Fake d96a import to test gating without ROS installed.
import types
m=types.ModuleType('d96a_readonly_follower');m.load_stage=lambda *a:None;m.sample=lambda *a:None
sys.modules['d96a_readonly_follower']=m
from d96c1_shadow import decide, ROBOTS, GAZEBO_FRAMES
class TestGating(unittest.TestCase):
    def setUp(self):
        self.refs={r:{'x_m':3*i,'y_m':0.} for i,r in enumerate(ROBOTS)}
        self.obs={r:{'x_m':3*i,'y_m':0.,'yaw_rad':0.,'frame_id':GAZEBO_FRAMES[r]} for i,r in enumerate(ROBOTS)}
        self.last={r:4. for r in ROBOTS}
    def chk(self,**kw):return decide(self.refs,self.obs,0.,4.,self.last,expected_frames=GAZEBO_FRAMES,**kw)
    def test_default_blocks(self):self.assertEqual(self.chk(),'BLOCKED_REFERENCE_FRAME_MISMATCH')
    def test_opt_in_without_alignment_blocks(self):self.assertEqual(self.chk(gazebo_aligned=True,alignment_ok=False),'BLOCKED_REFERENCE_FRAME_MISMATCH')
    def test_opt_in_aligned_shadow(self):self.assertEqual(self.chk(gazebo_aligned=True,alignment_ok=True),'SHADOW_OK')
    def test_frame_change_blocks(self):
        self.obs['summit0']['frame_id']='bad'
        self.assertEqual(self.chk(gazebo_aligned=True,alignment_ok=True),'BLOCKED_FRAME_CHANGE')
    def test_tracking_error_blocks(self):
        self.obs['summit0']['x_m']=4.
        self.assertEqual(self.chk(gazebo_aligned=True,alignment_ok=True),'BLOCKED_TRACKING_ERROR')
if __name__=='__main__':unittest.main()
