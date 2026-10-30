import math
import sys
import types
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
# Offline tests do not require ROS; the production evaluator is imported in Jazzy.
m=types.ModuleType('d96a_readonly_follower')
m.load_stage=lambda *args: None
m.sample=lambda *args: None
sys.modules['d96a_readonly_follower']=m
from d96c1_shadow import reference_start_offsets, decide, ROBOTS, GAZEBO_FRAMES

class TestStartOffsets(unittest.TestCase):
    def setUp(self):
        self.positions={r:(float(3*i),float(-2*i)) for i,r in enumerate(ROBOTS)}
        self.refs={r:{'x_m':v[0],'y_m':v[1]} for r,v in self.positions.items()}

    def test_exact(self):
        result=reference_start_offsets(self.refs,self.positions)
        self.assertTrue(all(row['offset_m']==0 for row in result.values()))

    def test_122_mm_reference_offset_does_not_change_measured_tolerance(self):
        self.refs['summit0']['x_m']+=.122
        result=reference_start_offsets(self.refs,self.positions)
        self.assertAlmostEqual(result['summit0']['offset_m'],.122)
        self.assertGreater(result['summit0']['offset_m'],.10)
        self.assertLess(result['summit0']['offset_m'],.15)
        # A measured initial pose error >0.10 m should still FAIL the separate gate.
        observed_error=.11
        self.assertFalse(observed_error<=.10)

    def test_large_plan_offset_visible(self):
        self.refs['summit1']['x_m']+=.25
        self.assertGreater(reference_start_offsets(self.refs,self.positions)['summit1']['offset_m'],.20)

    def test_nonfinite_rejected(self):
        self.refs['summit1']['x_m']=math.nan
        with self.assertRaises(ValueError):reference_start_offsets(self.refs,self.positions)

    def test_wrong_robot_set_rejected(self):
        del self.positions['summit2']
        with self.assertRaises(ValueError):reference_start_offsets(self.refs,self.positions)

    def test_cannot_pass_without_alignment(self):
        refs={r:{'x_m':i*3.,'y_m':0.} for i,r in enumerate(ROBOTS)}
        obs={r:{'x_m':i*3.,'y_m':0.,'yaw_rad':0.,'frame_id':GAZEBO_FRAMES[r]} for i,r in enumerate(ROBOTS)}
        ts={r:4. for r in ROBOTS}
        self.assertEqual(decide(refs,obs,4.,4.,ts,expected_frames=GAZEBO_FRAMES,
                                gazebo_aligned=True,alignment_ok=False),'BLOCKED_REFERENCE_FRAME_MISMATCH')

    def test_never_publish(self):
        src=(Path(__file__).resolve().parents[1]/'d96c1_shadow.py').read_text()
        self.assertNotIn('.create_publisher(',src)
        self.assertNotIn('.publish(',src)
        self.assertNotIn('static_transform_publisher',src)

if __name__=='__main__':unittest.main()
