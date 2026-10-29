import sys, unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import d96c2f_compare_frames as m
SAMPLE='''header { stamp { sec: 123 } }\npose {\n name: "summit0"\n id: 4\n position { x: -6.25 y: 8.75 z: 0 }\n orientation { x: 0 y: 0 z: 0 w: 1 }\n}\npose { name: "summit1"\n position {\n x: 11.25\n y: 8.75\n z: 0\n }\n orientation { z: 0 w: 1 } }\n'''
# Actual Gazebo protobuf text usually formats each axis on its own line.
SAMPLE2='''pose {\n name: "summit0"\n position {\n x: -6.25\n y: 8.75\n z: 0\n }\n orientation {\n x: 0\n y: 0\n z: 0\n w: 1\n }\n}\n'''
class T(unittest.TestCase):
 def test_blocks(self):self.assertEqual(len(m.blocks(SAMPLE2,'pose')),1)
 def test_named_poses(self):self.assertEqual(len(m.parse_pose_messages(SAMPLE2)['summit0']),1)
 def test_yaw(self):self.assertAlmostEqual(m.yaw([0,0,0,1]),0)
 def test_wrapping(self):self.assertAlmostEqual(m.angle_diff(-3.14,3.14),.003185307179586,places=5)
 def test_missing_reject(self):
  d={r:[] for r in m.ROBOTS}; self.assertEqual(m.analyze('',d,[])['status'],'REJECT_FRAME_COMPARISON')
 def test_no_motion_authorized(self):
  d={r:[] for r in m.ROBOTS}; self.assertFalse(m.analyze('',d,[])['execution_authorized'])
 def test_nested_quotes(self): self.assertEqual(len(m.blocks('pose { name: "hello { world }" position {} }','pose')),1)
if __name__=='__main__': unittest.main()
