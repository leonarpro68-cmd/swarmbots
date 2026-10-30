import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d91_segment_retime import blocks,check_piecewise,solve

def piece(x,y,dist=1.):
    return np.array([[x+dist*j/5,y] for j in range(6)],float)

def test_blocks_and_continuity():
    # Two smoothly connected linear Béziers must share one scaling block.
    c=np.stack((piece(0,0),piece(1,0)))
    assert blocks(c,2)==[[0,1]]
    bad=c.copy();bad[1,0,0]+=0.1
    try:blocks(bad,2);assert False
    except ValueError:pass
    # A full stop permits independent time blocks.
    stop=np.array([[0.,0.],[.1,0],[.4,0],[.7,0],[1.,0],[1.,0]])
    restart=np.array([[1.,0],[1.,0],[1.2,0],[1.5,0],[1.8,0],[2.,0]])
    assert blocks(np.stack([stop,restart]),2)==[[0],[1]]

def test_conservative_three_robots():
    cs=[np.stack([piece(0,y)] ) for y in (0.,5.,10.)]
    ok,info=check_piecewise(cs,[1]*3,[np.array([2.])]*3)
    assert ok and info['certified_lower_m']>1.15
    r=solve(cs,2.)
    assert r['status']=='CERTIFIED_CONSERVATIVE_OFFLINE'
    assert r['optimized_makespan_s']<2.
    # Close parallel trajectories violate required separation.
    near=[np.stack([piece(0,y)]) for y in (0.,.6,10.)]
    ok,_=check_piecewise(near,[1]*3,[np.array([2.])]*3)
    assert not ok

if __name__=='__main__':
    test_blocks_and_continuity();test_conservative_three_robots();print('D9.1 tests OK')
