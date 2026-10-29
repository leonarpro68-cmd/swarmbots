import importlib.util
from pathlib import Path
import numpy as np
f=Path(__file__).resolve().parents[1]/'d9_offline_retime.py'
spec=importlib.util.spec_from_file_location('d9',f); d9=importlib.util.module_from_spec(spec);spec.loader.exec_module(d9)

def line(x0,y0,x1,y1):
    return np.linspace([x0,y0],[x1,y1],6)[None,:,:]

def test_stretch_lowers_bounds():
    c=line(0,0,1,0)
    v,a=d9.max_bounds(c,1,1)
    assert abs(v-1)<1e-10 and a<1e-10
    assert np.allclose(d9.position(c,1,1,.5,.25), [.5,0])

def test_hold_after_arrival():
    c=line(0,0,2,0)
    assert np.allclose(d9.position(c,2,1,.5,100),[2,0])

def test_reject_crossing_and_proximity():
    cs=[line(-1,0,1,0),line(0,-1,0,1),line(5,5,6,5)]
    ok,info=d9.check(cs,4,[1,1,1],[1,1,1],1.2,1,1.15,.05)
    assert not ok and info['reason']=='separation'

def test_independent_endings_and_no_collision():
    # Dos robots fijos lejos y uno recorre tramo corto, permitido acelerar
    cs=[line(0,0,2,0),line(0,10,1,10),line(0,20,0.5,20)]
    result=d9.solve_stage(cs,4,1.2,1.,1.15,.1,5)
    assert result['status']=='CERTIFIED_CONSERVATIVE'
    assert result['optimized_duration_s']<result['original_common_duration_s']

def test_failure_never_claims_certificate():
    cs=[line(0,0,1,0),line(0,.1,1,.1),line(10,10,11,10)]
    x=d9.solve_stage(cs,3,1.2,1,1.15,.1,3)
    assert x['status']=='UNVERIFIED_BASELINE'

if __name__=='__main__':
    for name,obj in list(globals().items()):
        if name.startswith('test_') and callable(obj):obj();print('OK',name)
