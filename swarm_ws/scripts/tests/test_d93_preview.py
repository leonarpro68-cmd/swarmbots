import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d92_smooth_retime import phase,phase_d1

class D93Tests(unittest.TestCase):
    def test_phase_endpoints(self):
        for a,b in ((0,0),(.3,.12),(-.2,0)):
            self.assertAlmostEqual(phase(0,a,b),0)
            self.assertAlmostEqual(phase(1,a,b),1)
            self.assertTrue(np.min(phase_d1(np.linspace(0,1,100),a,b))>0)
    def test_no_ros_command_generation(self):
        text=(Path(__file__).resolve().parents[1]/'d93_validate_preview.py').read_text()
        self.assertNotIn('TimedCommand(',text)
        self.assertNotIn('rclpy',text)

if __name__=='__main__': unittest.main()
