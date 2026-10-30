import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d96b_clock_readonly import ClockGate

class TestGate(unittest.TestCase):
    def test_progress_and_hold(self):
        g=ClockGate(2)
        self.assertIsNone(g.observe(0.0))
        self.assertEqual(g.observe(5.0),0)
        self.assertAlmostEqual(g.observe(6),1)
        self.assertEqual(g.observe(8),2)
        self.assertEqual(g.state,'COMPLETE_READ_ONLY')
    def test_rewind_rejected(self):
        g=ClockGate(4)
        g.observe(10);g.observe(11)
        with self.assertRaises(RuntimeError):g.observe(9)
        self.assertEqual(g.state,'CLOCK_REWIND_ABORT')
    def test_duplicate_clock_safe(self):
        g=ClockGate(4)
        g.observe(10)
        self.assertEqual(g.observe(10),0)
        self.assertAlmostEqual(g.observe(10.00000001),1e-8,places=7)
    def test_bad_values(self):
        with self.assertRaises(ValueError):ClockGate(0)
        g=ClockGate(2)
        self.assertIsNone(g.observe(float('nan')))

if __name__=='__main__':unittest.main()
