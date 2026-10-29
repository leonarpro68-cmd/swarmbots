import sys
from pathlib import Path
import tempfile
import unittest
import json
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d95_protocol_preflight import check_stage,SCHEMA
from d94_convert_offline import hermite_quintic

class TestD95(unittest.TestCase):
    def test_stage_and_integrity(self):
        with tempfile.TemporaryDirectory() as td:
            run=Path(td); base=run/'d94_conversion';base.mkdir()
            cs={};dts={}
            for i in range(3):
                p=np.array([i*3.,0.])
                cs[f'R{i}_controls']=np.stack([hermite_quintic(p,np.zeros(2),np.zeros(2),p,np.zeros(2),np.zeros(2),1.)])
                dts[f'R{i}_durations_s']=np.array([1.])
            np.savez_compressed(base/'pickup1_converted_NON_EXECUTABLE.npz',**cs,**dts)
            (base/'pickup1_conversion_report.json').write_text(json.dumps({'stage':'pickup1','status':'CONVERTED_OFFLINE_CHECKS_PASSED','ros_executable':False,'checks':{'approximation':True}}))
            (run/'d92_offline_report.json').write_text(json.dumps({'stages':{'pickup1':{'status':'CERTIFIED_CONSERVATIVE_OFFLINE','duration_s':[1.,1.,1.]}}}))
            out=run/'out'
            r=check_stage(run,'pickup1',out)
            self.assertEqual(r['schema'],SCHEMA)
            self.assertEqual(r['status'],'PASS_READ_ONLY')
            self.assertFalse(r['ros_executable'])
            self.assertTrue((out/'pickup1_INDEPENDENT_TIMED_BEZIER_PREVIEW_ONLY.json').exists())
    def test_end_velocity_nonzero_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            run=Path(td);base=run/'d94_conversion';base.mkdir()
            data={}
            for i in range(3):
                p=np.array([i*3.,0.]);q=p+np.array([.1,0.])
                data[f'R{i}_controls']=np.stack([hermite_quintic(p,np.zeros(2),np.zeros(2),q,np.array([.2,0.]),np.zeros(2),1.)])
                data[f'R{i}_durations_s']=np.array([1.])
            np.savez_compressed(base/'pickup1_converted_NON_EXECUTABLE.npz',**data)
            (base/'pickup1_conversion_report.json').write_text(json.dumps({'stage':'pickup1','status':'CONVERTED_OFFLINE_CHECKS_PASSED','ros_executable':False,'checks':{'approximation':True}}))
            (run/'d92_offline_report.json').write_text(json.dumps({'stages':{'pickup1':{'status':'CERTIFIED_CONSERVATIVE_OFFLINE','duration_s':[1.,1.,1.]}}}))
            self.assertEqual(check_stage(run,'pickup1',run/'out')['status'],'REJECT')
if __name__=='__main__':unittest.main()
