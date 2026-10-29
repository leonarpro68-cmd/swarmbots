import math
import unittest
from d97_executor_core import calculate_command, verify_observations, ROBOTS

class TestCore(unittest.TestCase):
    def setUp(self):
        self.ref={'x_m':0.,'y_m':0.,'vx_m_s':0.,'vy_m_s':0.}
        self.obs={'x_m':0.,'y_m':0.,'yaw_rad':0.}
    def test_zero(self):
        self.assertEqual(calculate_command(self.ref,self.obs)['command_speed_m_s'],0.)
    def test_yaw_rotation(self):
        self.ref['vx_m_s']=1.
        c=calculate_command(self.ref,{**self.obs,'yaw_rad':math.pi/2},dt=.1)
        self.assertAlmostEqual(c['vx_body_m_s'],0,places=9)
        self.assertLess(c['vy_body_m_s'],0)
    def test_accel_limit(self):
        self.ref['vx_m_s']=100
        c=calculate_command(self.ref,self.obs,dt=.02)
        self.assertLessEqual(c['command_speed_m_s'],.02000001)
    def test_speed_limit(self):
        self.ref['vx_m_s']=100
        c=calculate_command(self.ref,self.obs,dt=.1,previous=(1.2,0.))
        self.assertLessEqual(c['command_speed_m_s'],1.2)
    def test_nonfinite(self):
        self.ref['vx_m_s']=math.nan
        with self.assertRaises(ValueError): calculate_command(self.ref,self.obs)
    def test_negative_dt(self):
        with self.assertRaises(ValueError): calculate_command(self.ref,self.obs,dt=-1)
    def test_broken_frame(self):
        ref={r:dict(self.ref,x_m=i*5.) for i,r in enumerate(ROBOTS)}
        od={r:{**self.obs,'x_m':i*5.,'frame_id':r+'/odom','child_frame_id':r+'/base_footprint'} for i,r in enumerate(ROBOTS)}
        self.assertEqual(verify_observations(ref,od),[])
        od['summit1']['frame_id']='map'
        self.assertTrue(any('FRAME_MISMATCH' in v for v in verify_observations(ref,od)))
    def test_collision(self):
        ref={r:dict(self.ref) for r in ROBOTS}
        od={r:{**self.obs,'frame_id':r+'/odom','child_frame_id':r+'/base_footprint'} for r in ROBOTS}
        self.assertTrue(any('SEPARATION' in v for v in verify_observations(ref,od)))
if __name__=='__main__':unittest.main()
