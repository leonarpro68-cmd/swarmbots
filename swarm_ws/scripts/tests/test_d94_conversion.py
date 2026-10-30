import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d94_convert_offline import hermite_quintic, bernstein_eval, converted_values, polynomial_bounds

def test_quintic_endpoints():
    p0=np.array([0.,0.]); p1=np.array([1.,2.]);v0=np.array([.2,0.]);v1=np.array([0.,-.2]);a0=np.array([0.,.3]);a1=np.array([.1,0.]);h=2.
    c=hermite_quintic(p0,v0,a0,p1,v1,a1,h)
    assert np.allclose(c[0],p0) and np.allclose(c[-1],p1)
    assert np.allclose(5*(c[1]-c[0])/h,v0)
    assert np.allclose(5*(c[-1]-c[-2])/h,v1)
    assert np.allclose(20*(c[2]-2*c[1]+c[0])/h**2,a0)
    assert np.allclose(20*(c[-1]-2*c[-2]+c[-3])/h**2,a1)
    out=converted_values(c[None],np.array([h]),np.array([0.,h]))
    assert np.allclose(out[0][0],p0) and np.allclose(out[0][-1],p1)
    assert np.allclose(out[1][0],v0)

def test_join_and_hold():
    z=np.zeros(2);p1=np.array([1.,0.]);p2=np.array([2.,0.])
    c=np.stack([hermite_quintic(z,z,z,p1,z,z,1.),hermite_quintic(p1,z,z,p2,z,z,1.)])
    p,v,a=converted_values(c,np.array([1.,1.]),np.array([.99999,1.,1.00001,2.5]))
    assert np.linalg.norm(p[0]-p[1])<1e-5
    assert np.linalg.norm(p[1]-p[2])<1e-5
    assert np.array_equal(v[-1],z) and np.array_equal(a[-1],z)
    assert polynomial_bounds(c,np.array([1.,1.]))[0]>0

if __name__=='__main__':
    test_quintic_endpoints();test_join_and_hold();print('D9.4 tests OK')
