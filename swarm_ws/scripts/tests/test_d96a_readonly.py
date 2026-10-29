import sys
from pathlib import Path
import json
import tempfile
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d94_convert_offline import hermite_quintic
from d95_protocol_preflight import check_stage
from d96a_readonly_follower import load_stage, sample, run_preview

class TestD96A(unittest.TestCase):
    def fixture(self,base):
        out=base/'d94_conversion';out.mkdir()
        data={}
        for i in range(3):
            p=np.array([i*3.,0.])
            if i==0:
                q=p+np.array([0.02,0.]);p1=hermite_quintic(p,np.zeros(2),np.zeros(2),q,np.zeros(2),np.zeros(2),1.)
            else:
                p1=hermite_quintic(p,np.zeros(2),np.zeros(2),p,np.zeros(2),np.zeros(2),1.)
            data[f'R{i}_controls']=np.stack([p1]);data[f'R{i}_durations_s']=np.array([1.])
        np.savez_compressed(out/'pickup1_converted_NON_EXECUTABLE.npz',**data)
        (out/'pickup1_conversion_report.json').write_text(json.dumps({'stage':'pickup1','status':'CONVERTED_OFFLINE_CHECKS_PASSED','ros_executable':False,'checks':{'approximation':True}}))
        (base/'d92_offline_report.json').write_text(json.dumps({'stages':{'pickup1':{'status':'CERTIFIED_CONSERVATIVE_OFFLINE','duration_s':[1.,1.,1.]}}}))
        manifest=check_stage(base,'pickup1',base/'d95_preflight')
        self.assertEqual(manifest['status'],'PASS_READ_ONLY')
    def test_sample_and_hold(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);self.fixture(root)
            _,trajs=load_stage(root,'pickup1')
            mid=sample(trajs,0.5)
            self.assertAlmostEqual(mid['summit0']['x_m'],0.01)
            self.assertFalse(mid['summit0']['done'])
            end=sample(trajs,2.)
            self.assertTrue(all(v['done'] for v in end.values()))
            self.assertAlmostEqual(end['summit0']['x_m'],0.02)
            self.assertAlmostEqual(end['summit0']['vx_m_s'],0.)
            rep=run_preview(root,'pickup1',0.05,root/'d96a_readonly')
            self.assertEqual(rep['commands_sent'],0)
            self.assertGreater(rep['sampled_min_separation_m'],1.15)
    def test_mutated_file_refused(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);self.fixture(root)
            target=root/'d94_conversion/pickup1_conversion_report.json'
            target.write_text(target.read_text()+'\n')
            with self.assertRaisesRegex(ValueError,'Integrity mismatch'):
                load_stage(root,'pickup1')
    def test_refuse_executable_manifest(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);self.fixture(root)
            p=root/'d95_preflight/pickup1_INDEPENDENT_TIMED_BEZIER_PREVIEW_ONLY.json'
            m=json.loads(p.read_text());m['ros_executable']=True;p.write_text(json.dumps(m))
            with self.assertRaisesRegex(ValueError,'Refusing'):
                load_stage(root,'pickup1')
if __name__=='__main__':unittest.main()
