import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d96c1_shadow import shadow_command,decide,ROBOTS

class ShadowTests(unittest.TestCase):
    def setUp(self):
        self.ref={n:{'x_m':i*3.,'y_m':0.,'vx_m_s':0.3,'vy_m_s':0.} for i,n in enumerate(ROBOTS)}
        self.ob={n:{'x_m':i*3.,'y_m':0.,'yaw_rad':0.,'frame_id':'map'} for i,n in enumerate(ROBOTS)}
        self.ts={n:5. for n in ROBOTS}
    def test_feedforward_rotation(self):
        cmd,err=shadow_command({'x_m':0.,'y_m':0.,'vx_m_s':1.,'vy_m_s':0.}, {'x_m':0.,'y_m':0.,'yaw_rad':1.5707963267948966})
        self.assertAlmostEqual(cmd[0],0.,places=6);self.assertAlmostEqual(cmd[1],-1.,places=6)
    def test_limits(self):
        ref={'x_m':10.,'y_m':0.,'vx_m_s':2.,'vy_m_s':0.}
        cmd,_=shadow_command(ref,{'x_m':0.,'y_m':0.,'yaw_rad':0.},previous=(0.,0.),dt=.1)
        self.assertLessEqual((cmd[0]**2+cmd[1]**2)**.5,.100000001)
    def test_block_no_odom(self):
        self.assertEqual(decide(self.ref,{},10.,5.,{}),'BLOCKED_NO_ODOM')
    def test_block_unsafe(self):
        ob={k:dict(v) for k,v in self.ob.items()};ob['summit1']['x_m']=.5
        self.assertEqual(decide(self.ref,ob,10.,5.,self.ts,max_error=100),'BLOCKED_REAL_SEPARATION')
    def test_block_stale_and_errors(self):
        self.assertEqual(decide(self.ref,self.ob,10.,6.,self.ts),'BLOCKED_STALE_ODOM')
        ob={k:dict(v) for k,v in self.ob.items()};ob['summit0']['x_m']+=1.
        self.assertEqual(decide(self.ref,ob,10.,5.,self.ts),'BLOCKED_TRACKING_ERROR')
    def test_frame_mismatch(self):
        self.ob['summit0']['frame_id']='odom'
        self.assertEqual(decide(self.ref,self.ob,10.,5.,self.ts),'BLOCKED_REFERENCE_FRAME_MISMATCH')
    def test_no_publishers_in_source(self):
        s=Path(__file__).resolve().parents[1].joinpath('d96c1_shadow.py').read_text()
        self.assertNotIn('.create_publisher(',s)
        self.assertNotIn('publish(',s)

if __name__=='__main__':unittest.main()
