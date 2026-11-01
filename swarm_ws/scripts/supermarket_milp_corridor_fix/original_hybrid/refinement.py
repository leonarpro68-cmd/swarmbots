"""Sampled corridor rebuilding from the original refinar.py, without script imports."""
import numpy as np
from shapely.geometry import Point, LineString
from shapely.ops import nearest_points
from . import geometry_reference as geom
from . import geometry_corridors as opt
from .slsqp import solve_all


def sampled_corridors(controls, obstacles, *, samples=32, operational_margin_m=0.0):
    """Rebuild separators as in refinar.py, optionally with extra reserve."""
    if not np.isfinite(operational_margin_m) or operational_margin_m < 0:
        raise ValueError("operational_margin_m must be nonnegative")
    margin = float(operational_margin_m)
    if samples<3:raise ValueError('samples must be >=3')
    N=len(controls);K=len(controls[0]);R=geom.CLEARANCE_WALL_M
    sampled=[[np.array([opt.bernstein(5,u)@controls[i][k]
             for u in np.linspace(0,1,samples)]) for k in range(K)] for i in range(N)]
    hs=[[[] for _ in range(K)] for _ in range(N)]
    for k in range(K):
        shapes=[LineString(sampled[i][k]) if np.ptp(sampled[i][k],axis=0).max()>1e-9
                else Point(sampled[i][k][0]) for i in range(N)]
        for i in range(N):
            hs[i][k].extend([(np.array([1.,0.]),geom.X_MAX-R-margin),
                (np.array([-1.,0.]),-geom.X_MIN-R-margin),
                (np.array([0.,1.]),geom.Y_MAX-R-margin),
                (np.array([0.,-1.]),-geom.Y_MIN-R-margin)])
            for center,radius in obstacles:
                aa,bb=nearest_points(shapes[i],Point(center))
                a=np.array(aa.coords[0]);b=np.array(bb.coords[0])
                diff=b-a;dist=np.linalg.norm(diff)
                if dist<=radius+margin+1e-7:raise ValueError(f'robot {i} obstacle conflict k={k}')
                n=diff/dist;hs[i][k].append((n,float(n@center-radius-margin)))
        for i in range(N):
            for j in range(i+1,N):
                aa,bb=nearest_points(shapes[i],shapes[j]);a=np.array(aa.coords[0]);b=np.array(bb.coords[0]);diff=b-a;dist=np.linalg.norm(diff)
                if dist<=2*R+margin+1e-7:raise ValueError(f'robot conflict {i}-{j} k={k}')
                n=diff/dist;mid=(a+b)/2
                hs[i][k].append((n,float(n@mid-R-margin/2)))
                hs[j][k].append((-n,float(-n@mid-R-margin/2)))
    return hs


def cost(controls):
    return float(sum(opt._acceleration_cost(s,1) for s in controls))


def refine(points, paths, obstacles, controls, *, iterations=5, samples=32,
           require_improvement=True, operational_margin_m=0.0, maxiter=450):
    current=[np.asarray(c,float).copy() for c in controls]
    history=[{'iteration':0,'cost':cost(current)}]
    for it in range(1,iterations+1):
        hs=sampled_corridors(current,obstacles,samples=samples,
                             operational_margin_m=operational_margin_m)
        new,stats=solve_all(points,paths,hs,require_success=True,maxiter=maxiter)
        old_cost=cost(current); new_cost=cost(new)
        if require_improvement and new_cost>old_cost+1e-8:
            history.append({'iteration':it,'accepted':False,'reason':'cost increased','cost':old_cost})
            break
        current=new
        history.append({'iteration':it,'accepted':True,'cost':new_cost,'solver':stats})
    return current,history
