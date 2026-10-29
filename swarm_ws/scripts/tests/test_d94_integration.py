import json
import sys
import tempfile
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d94_convert_offline import inspect

def test_synthetic_integration():
    with tempfile.TemporaryDirectory() as root:
        root=Path(root);stage='pickup1';folder=root/'mission'/stage/'protocol';folder.mkdir(parents=True)
        (folder/f'{stage}_protocol_report.json').write_text(json.dumps({'stages':{stage:{'certified':True,'hybrid_v2':{'certified':True}}}}))
        # Smoothstep Bézier control polygon; zero endpoint velocity/acceleration.
        ctrl=[]
        for i in range(3):
            p0=np.array([i*5.,0.]);p1=p0+np.array([1.,0.])
            ctrl.append(np.stack([p0,p0,p0,p1,p1,p1])[None,:,:])
        np.savez(folder/f'{stage}_protocol_controls.npz',dt_s=np.array(4.),R0=ctrl[0],R1=ctrl[1],R2=ctrl[2])
        (root/'d92_offline_report.json').write_text(json.dumps({'stages':{stage:{'status':'CERTIFIED_CONSERVATIVE_OFFLINE','active_pieces':[1,1,1], 'duration_s':[4.,4.,4.], 'harmonics':[[0.,0.]]*3}}}))
        report=inspect(root,stage,root/'out',subdivisions=2)
        assert report['status']=='CONVERTED_OFFLINE_CHECKS_PASSED',report
        assert not report['ros_executable']
        assert report['certified_lower_separation_m']>1.15
        assert len(np.load(root/'out'/f'{stage}_converted_NON_EXECUTABLE.npz')['R0_durations_s'])==2

if __name__=='__main__':test_synthetic_integration();print('D9.4 integration OK')
