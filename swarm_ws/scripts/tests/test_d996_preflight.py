import json
import sys
import tempfile
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d996_preflight import static_check

class PreflightTests(unittest.TestCase):
    def setUp(self):
        self.root=Path(__file__).resolve().parents[1]
        self.d=self.root/'d996_reference'
        self.ref=json.loads((self.root/'auditoria_identidad_d99.json').read_text())
        self.args=[self.d/'world.sdf',self.d/'scenario.json',self.d/'pickup1_d994_NON_EXECUTABLE.npz',self.d/'pickup1_d994_conversion_report.json',self.d/'pickup1_d995_preflight.json']
    def test_reference_passes(self):
        problems,_=static_check(self.ref,*self.args)
        self.assertEqual(problems,[])
    def test_world_tampering_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'world.sdf';p.write_bytes(self.args[0].read_bytes()+b'\n')
            problems,_=static_check(self.ref,p,*self.args[1:])
            self.assertIn('WORLD_HASH_MISMATCH',problems)
    def test_trajectory_tampering_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'traj.npz';p.write_bytes(self.args[2].read_bytes()+b'corrupt')
            problems,_=static_check(self.ref,*self.args[:2],p,*self.args[3:])
            self.assertIn('CONVERSION_NPZ_HASH_MISMATCH',problems)
    def test_missing_preflight_rejected(self):
        problems,_=static_check(self.ref,*self.args[:4],Path('/not/a/file'))
        self.assertIn('MISSING_OFFLINE_PREFLIGHT',problems)

if __name__=='__main__':unittest.main()
