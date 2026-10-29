import sys,unittest,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d98b1_turn_align_grasp import plan_turn,tip_check,wrap

class D98B1GeometryTests(unittest.TestCase):
    def setUp(self):
        self.names=['trash_a','trash_b','trash_c']
        self.poses={f'summit{i}':(i*10,0.,math.pi/2,0.,0.) for i in range(3)}
        self.objects={f'trash_{c}':(i*10+1.75,0.) for i,c in enumerate('abc')}
    def test_phase_b_is_not_grasp_radius(self):
        r=plan_turn(self.names,self.poses,self.objects)
        self.assertEqual(r['status'],'READY_FOR_TURN_AND_RGBD_ONLY')
        self.assertEqual(r['mapping']['summit0'],'trash_a')
        self.assertTrue(r['robots']['summit0']['requires_rgbd_approach'])
        self.assertAlmostEqual(r['robots']['summit0']['turn_degrees'],-90.,places=2)
    def test_post_align_tip_close(self):
        r=tip_check('summit0','trash_a',(1.28,0.,0.,0.,0.),self.objects)
        self.assertTrue(r['ok'])
    def test_turn_only_not_enough(self):
        r=tip_check('summit0','trash_a',(0.,0.,0.,0.,0.),self.objects)
        self.assertFalse(r['ok'])
    def test_wrong_object_blocked(self):
        self.objects['trash_other']=(1.0,0.)
        self.assertEqual(plan_turn(self.names,self.poses,self.objects)['status'],'BLOCKED')
    def test_too_far(self):
        self.objects['trash_a']=(2.7,0.)
        self.assertIn('summit0:TOO_FAR_FOR_VISUAL_APPROACH',plan_turn(self.names,self.poses,self.objects)['reasons'])
    def test_too_close(self):
        self.objects['trash_a']=(.35,0.)
        self.assertIn('summit0:TOO_CLOSE_FOR_VISUAL_APPROACH',plan_turn(self.names,self.poses,self.objects)['reasons'])
    def test_angular_motion_blocked(self):
        self.poses['summit0']=(0.,0.,0.,0.,.4)
        self.assertIn('summit0:NOT_STOPPED',plan_turn(self.names,self.poses,self.objects)['reasons'])
    def test_tip_nearest_wrong(self):
        self.objects['trash_other']=(1.73,0.)
        self.assertFalse(tip_check('summit0','trash_a',(1.28,0.,0.,0.,0.),self.objects)['ok'])
    def test_angle_wrapping(self):
        self.assertAlmostEqual(wrap(3*math.pi),math.pi,places=6)
    def test_next_robot_after_first_has_gripped(self):
        # summit0 has already driven close to its target and attached it.
        self.poses['summit0']=(1.28,0.,0.,0.,0.)
        r=plan_turn(self.names,self.poses,self.objects,selected=('summit1',))
        self.assertEqual(r['status'],'READY_FOR_TURN_AND_RGBD_ONLY')
        self.assertEqual(r['mapping'],{'summit1':'trash_b'})
    def test_missing_odometry_blocks(self):
        del self.poses['summit2']
        self.assertEqual(plan_turn(self.names,self.poses,self.objects)['status'],'BLOCKED')

if __name__=='__main__':unittest.main()
