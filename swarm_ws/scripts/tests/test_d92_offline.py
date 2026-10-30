import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d92_smooth_retime import phase,phase_d1,values_for_robot,verify


def quintic_line(x0,x1):
    # p(u) = x0 +(x1-x0)*(10u^3-15u^4+6u^5): endpoint velocity=0.
    xs=np.array([0.,0.,0.,1.,1.,1.]);return np.column_stack([x0+(x1-x0)*xs,np.zeros(6)])


def tests():
    x=np.linspace(0,1,501)
    for a,b in [(0.,0.),(.2,-.1),(-.25,.1)]:
        assert np.max(np.abs(np.diff(phase(x,a,b))<=0))==0
        assert np.all(phase_d1(x,a,b)>.05)
        assert abs(phase(0,a,b))<1e-12 and abs(phase(1,a,b)-1)<1e-12
    c=quintic_line(0,1)[None,:,:]
    p,v,acc=values_for_robot(c,1,5.,.2,0.,np.linspace(0,5,101))
    assert np.linalg.norm(p[0]-[0,0])<1e-10 and np.linalg.norm(p[-1]-[1,0])<1e-10
    assert np.max(np.linalg.norm(v,axis=1))>0
    cs=[c,c+np.array([0,10]),c+np.array([0,20])]
    ok,r=verify(cs,[1]*3,[5.]*3,[(0.,0.)]*3,2.,2.,1.15,.04)
    assert ok,r
    ok,r=verify(cs,[1]*3,[5.]*3,[(.3,0.)]*3,2.,2.,1.15,.04)
    assert ok,r
    close=[c,c+np.array([0,.5]),c+np.array([0,20])]
    ok,r=verify(close,[1]*3,[5.]*3,[(0.,0.)]*3,2.,2.,1.15,.04)
    assert not ok and r['reason']=='separation',r
    print('D9.2 5 pruebas OK')
if __name__=='__main__':tests()
