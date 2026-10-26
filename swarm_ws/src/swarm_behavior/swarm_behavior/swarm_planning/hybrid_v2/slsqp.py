"""SLSQP nullspace formulation from ejecutar.py; no side effects on import.

Numerical objective and constraints are kept identical to the supplied experiment.
"""
import time
import numpy as np
from scipy.linalg import null_space
from scipy.optimize import minimize
from . import geometry_corridors as opt


def solve_all(points, node_paths, halfspaces, *, require_success=True, maxiter=450):
    """Optimize a synchronized (robots, ticks+1, 2) schedule.

    ``node_paths`` determines the active portion for each robot; the tail
    after arrival remains stationary. Returns controls and solver metadata.
    """
    pts=np.asarray(points,dtype=float)
    if pts.ndim!=3 or pts.shape[2]!=2 or len(node_paths)!=len(pts):
        raise ValueError('points must have shape (robots,ticks+1,2)')
    K=pts.shape[1]-1
    if K<1 or any(len(p)<2 or len(p)>K+1 for p in node_paths):
        raise ValueError('invalid active paths')
    if len(halfspaces)!=len(pts) or any(len(h)!=K for h in halfspaces):
        raise ValueError('corridor dimensions do not match')
    def solve_robot(i,hs,init=None):
      active=len(node_paths[i])-1; nv=active*12; p=pts[i,:active+1]
      x,w=np.polynomial.legendre.leggauss(10);x=(x+1)/2;w=w/2
      Q=np.zeros((6,6))
      for t,v in zip(x,w):
       bb=opt.bernstein(3,t);vec=20*(np.pad(bb,(2,0))-2*np.pad(bb,(1,1))+np.pad(bb,(0,2)))
       Q+=v*np.outer(vec,vec)
      H=np.zeros((nv,nv))
      for k in range(active):
       for c in range(2):
        ix=np.arange(k*12+c,k*12+12,2);H[np.ix_(ix,ix)]=Q
      rows=[];rhs=[]
      def row(terms,target):
       v=np.zeros(nv)
       for k,j,c,coef in terms:v[12*k+2*j+c]+=coef
       rows.append(v);rhs.append(target)
      for c in range(2):
       row([(0,0,c,1)],p[0,c]);row([(active-1,5,c,1)],p[-1,c]);row([(0,1,c,1),(0,0,c,-1)],0)
       row([(active-1,5,c,1),(active-1,4,c,-1)],0)
       row([(0,2,c,1),(0,1,c,-2),(0,0,c,1)],0)
       row([(active-1,5,c,1),(active-1,4,c,-2),(active-1,3,c,1)],0)
      for k in range(active-1):
       for c in range(2):
        row([(k,5,c,1),(k+1,0,c,-1)],0)
        row([(k,5,c,1),(k,4,c,-1),(k+1,1,c,-1),(k+1,0,c,1)],0)
        row([(k,5,c,1),(k,4,c,-2),(k,3,c,1),(k+1,2,c,-1),(k+1,1,c,2),(k+1,0,c,-1)],0)
      E=np.array(rows);e=np.array(rhs);Z=null_space(E);v=np.linalg.lstsq(E,e,rcond=None)[0]
      A=[];b=[]
      for k in range(active):
       for n,limit in hs[k]:
        for j in range(6):
         ar=np.zeros(nv);ar[12*k+2*j:12*k+2*j+2]=n;A.append(ar);b.append(limit)
      A=np.array(A);b=np.array(b);G=A@Z;g=b-A@v
      HH=Z.T@H@Z;hc=Z.T@H@v
      if init is None:
       weights=np.array([0,0,0,1,1,1]);init=np.array([p[k]*(1-weights[:,None])+p[k+1]*weights[:,None] for k in range(active)])
      y0=Z.T@(init[:active].ravel()-v)
      start=time.time()
      # With one active Bézier piece, fixed endpoint positions and zero
      # endpoint velocity/acceleration determine all six control points.
      # SciPy SLSQP cannot optimize an empty null-space vector.
      if Z.shape[1] == 0:
       z=v.copy()
       feasible=(np.max(A@z-b,initial=-np.inf)<=1e-6 and
                 np.max(np.abs(E@z-e),initial=0.0)<=1e-6)
       solver_success=bool(feasible)
       solver_message=('Fully constrained Bézier piece; no SLSQP variables' if feasible
                       else 'Fully constrained Bézier piece violates corridor constraints')
       solver_iterations=0
      else:
       result=minimize(lambda y:.5*y@HH@y+hc@y,y0,jac=lambda y:HH@y+hc,
            constraints=[{'type':'ineq','fun':lambda y:g-G@y,'jac':lambda y:-G}],method='SLSQP',
            options={'maxiter':maxiter,'ftol':1e-10,'disp':False})
       z=v+Z@result.x
       solver_success=bool(result.success)
       solver_message=str(result.message)
       solver_iterations=int(result.nit)
      sol=z.reshape(active,6,2)
      if active<K:sol=np.concatenate([sol,np.broadcast_to(p[-1],(K-active,6,2)).copy()])
      maxv=max(float(np.max(A@z-b)),0)
      eq=float(np.max(np.abs(E@z-e)))
      return sol,{'robot':i,'success':solver_success,'message':solver_message,'iterations':solver_iterations,'duration_sec':time.time()-start,'cost':float(opt._acceleration_cost(sol,1)),'inequality_violation':maxv,'equality_violation':eq}
    sols=[]; meta=[]
    for i in range(len(node_paths)):
        sol, status=solve_robot(i,halfspaces[i]);sols.append(sol);meta.append(status)
        if require_success and (not status['success'] or status['inequality_violation']>1e-6 or status['equality_violation']>1e-6):
            raise RuntimeError(f'Robot {i} optimization rejected: {status}')
    return sols,meta
