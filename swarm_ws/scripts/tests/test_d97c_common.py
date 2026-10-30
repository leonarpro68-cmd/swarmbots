import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d97c_common import *
class TestD97C(unittest.TestCase):
    def test_cap(self):self.assertAlmostEqual(clamp_xy(3,4,1.2)[0],.72)
    def test_nan(self):
        with self.assertRaises(ValueError):clamp_xy(float('nan'),0,1.)
    def test_frames(self):
        refs={r:{'x_m':float(i*5),'y_m':0.} for i,r in enumerate(ROBOTS)}
        obs={r:{'x':float(i*5),'y':0.,'yaw':0.,'speed':0.,'age':.01,'frame_id':r+'/odom','child_frame_id':r+'/base_footprint'} for i,r in enumerate(ROBOTS)}
        self.assertEqual(gate(obs,refs,start=True),[])
        obs['summit0']['frame_id']='map';self.assertIn('summit0:FRAME',gate(obs,refs,start=True))
    def test_separation(self):
        refs={r:{'x_m':float(i*5),'y_m':0.} for i,r in enumerate(ROBOTS)}
        obs={r:{'x':float(i*5),'y':0.,'yaw':0.,'speed':0.,'age':.01,'frame_id':r+'/odom','child_frame_id':r+'/base_footprint'} for i,r in enumerate(ROBOTS)}
        obs['summit1']['x']=.5; self.assertTrue(any('SEPARATION' in x for x in gate(obs,refs)))
if __name__=='__main__':unittest.main()
