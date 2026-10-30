import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d97b2_stop_watchdog import sanitize,cmd_value
class Tests(unittest.TestCase):
    def test_accepted(self):self.assertEqual(sanitize(.08),(.08,True))
    def test_limit(self):self.assertFalse(sanitize(.3)[1])
    def test_sideways(self):self.assertFalse(sanitize(.08,.01,0)[1])
    def test_nan(self):self.assertFalse(sanitize(float('nan'))[1])
    def test_fresh(self):self.assertEqual(cmd_value(.08,.1,.2,.05,.05,True,False,.01)[0],.08)
    def test_timeout(self):self.assertEqual(cmd_value(.08,.31,.2,.05,.05,True,False,.01)[0],0)
    def test_stale_clock(self):self.assertEqual(cmd_value(.08,.05,.2,.05,.4,False,False,.01)[0],0)
    def test_travel(self):self.assertEqual(cmd_value(.08,.05,.2,.05,.05,True,False,.26)[0],0)
    def test_other_publisher(self):self.assertEqual(cmd_value(.08,.05,.2,.05,.05,True,True,.01)[0],0)
    def test_expired_motion(self):self.assertEqual(cmd_value(.08,.05,1.1,.05,.05,True,False,.01)[0],0)
if __name__=='__main__':unittest.main()
