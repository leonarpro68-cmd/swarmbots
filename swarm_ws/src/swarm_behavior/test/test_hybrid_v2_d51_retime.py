import importlib.util
from pathlib import Path
import numpy as np

SCRIPT=Path(__file__).resolve().parents[3]/'scripts'/'retime_hybrid_v2_d51.py'
spec=importlib.util.spec_from_file_location('retime_d51',SCRIPT)
mod=importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def test_split_matches_curve():
    from math import comb
    rng=np.random.default_rng(3)
    c=rng.normal(size=(6,2));left,right=mod.split_half(c)
    def ev(p,t): return sum(comb(5,j)*t**j*(1-t)**(5-j)*p[j] for j in range(6))
    for u in np.linspace(0,1,25):
        np.testing.assert_allclose(ev(c,u/2),ev(left,u),atol=1e-12)
        np.testing.assert_allclose(ev(c,(u+1)/2),ev(right,u),atol=1e-12)


def test_monotonic_bounds_and_sampling():
    rng=np.random.default_rng(12)
    c=rng.normal(size=(3,2,6,2))
    vals=[mod.bounds(c,d) for d in (0,1,2,3)]
    assert all(vals[k+1][j]<=vals[k][j]+1e-9 for k in range(3) for j in (0,1))
    from math import comb
    for piece in c.reshape(-1,6,2):
        v=5*np.diff(piece,axis=0);a=20*np.diff(piece,n=2,axis=0)
        for u in np.linspace(0,1,201):
            speed=sum(comb(4,j)*u**j*(1-u)**(4-j)*v[j] for j in range(5))
            acc=sum(comb(3,j)*u**j*(1-u)**(3-j)*a[j] for j in range(4))
            assert np.linalg.norm(speed)<=vals[-1][0]+1e-9
            assert np.linalg.norm(acc)<=vals[-1][1]+1e-9


def test_time_limits_and_shared_horizon():
    c=np.zeros((3,3,6,2));c[0,:, :,0]=np.linspace(0,1,6)
    x=mod.retime(c,8.,speed=1.2,accel=1.,factor=1.08,depth=4)
    assert x['speed_bound_m_s']<=1.2 and x['acceleration_bound_m_s2']<=1.
    assert abs(x['new_total_s']-3*x['new_dt_s'])<1e-12
