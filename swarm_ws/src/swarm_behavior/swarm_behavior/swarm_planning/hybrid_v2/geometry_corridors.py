"""Offline corridor-constrained degree-7 Bezier quadratic optimization.

Inspired by Hoenig et al. (2018), section VII. No ROS motion commands.
Requires numpy and scipy (sparse linear algebra). Collision validation uses
Bezier convex hulls and conservative separating half-spaces for circular robots.
"""
from __future__ import annotations
import math
import numpy as np
from scipy.special import comb
from scipy.sparse import csc_matrix, vstack, eye
from scipy.sparse.linalg import splu
from scipy.optimize import minimize
from . import geometry_reference as geom
TRASH_EXCLUSION_M=0.4
DEPOSIT_RADIUS_M=0.5

D = 5
R = geom.CLEARANCE_WALL_M


def bernstein(d, s):
    return np.array([comb(d, j) * s**j * (1-s)**(d-j) for j in range(d+1)])


def closest_segments(a,b):
    """Closest points on two segments; 2D convex bounded least squares."""
    a,b=np.asarray(a,float),np.asarray(b,float)
    u=a[1]-a[0];v=b[1]-b[0]
    def val(z):
        r=a[0]+z[0]*u-b[0]-z[1]*v
        return .5*r@r
    def jac(z):
        r=a[0]+z[0]*u-b[0]-z[1]*v
        return np.array([u@r,-v@r])
    candidates=[]
    for x in ((0,0),(0,1),(1,0),(1,1),(.5,.5)):
        result=minimize(val,x,jac=jac,bounds=((0,1),(0,1)),method='L-BFGS-B')
        candidates.append(result.x)
    p,q=min(((a[0]+z[0]*u,b[0]+z[1]*v) for z in candidates), key=lambda pq:np.linalg.norm(pq[0]-pq[1]))
    return p,q


def disk_halfspace(seg, center, exclusion):
    """Separates an entire segment from an exclusion disk by a supporting line."""
    center=np.asarray(center,float); a,b=np.asarray(seg,float)
    delta=b-a; d2=delta@delta
    t=0 if d2<1e-14 else np.clip((center-a)@delta/d2,0,1)
    q=a+t*delta
    vec=center-q; dist=np.linalg.norm(vec)
    if dist <= exclusion+1e-7:
        raise ValueError(f'No hay separacion espacial respecto al obstaculo (d={dist:.3f}, r={exclusion:.3f})')
    n=vec/dist
    return n,float(n@center-exclusion)


def obstacles(graph):
    """Actual phase graph; deposits intentionally use their existing 0.50m exclusion."""
    out=[(c,geom.CLEARANCE_COLUMN_M) for c in geom.COLUMNS_XY]
    out += [(geom.node_position(graph.trash[name].node),TRASH_EXCLUSION_M) for name in graph.pending]
    out += [(geom.node_position(d.node),DEPOSIT_RADIUS_M) for d in graph.deposits.values()]
    return out


def corridors(points, circles, *, operational_margin_m=0.0):
    """Original corridor construction with optional *additional* execution reserve.

    Existing obstacle radii already include geometric exclusion; add margin once.
    Robot-pair total extra separation is margin, shared equally by the halfplanes.
    """
    if not np.isfinite(operational_margin_m) or operational_margin_m < 0:
        raise ValueError("operational_margin_m must be nonnegative")
    m = float(operational_margin_m)
    n,steps,_=points.shape
    result=[[[] for _ in range(steps-1)] for _ in range(n)]
    for k in range(steps-1):
        for i in range(n):
            segment=points[i,k:k+2]
            hs=result[i][k]
            hs.extend([(np.array([1.,0.]),geom.X_MAX-R-m),(-np.array([1.,0.]),-(geom.X_MIN+R+m)),
                       (np.array([0.,1.]),geom.Y_MAX-R-m),(-np.array([0.,1.]),-(geom.Y_MIN+R+m))])
            for center,exclusion in circles:
                hs.append(disk_halfspace(segment,center,exclusion+m))
        for i in range(n):
            for j in range(i+1,n):
                pa,pb=closest_segments(points[i,k:k+2],points[j,k:k+2])
                d=pb-pa; length=float(np.linalg.norm(d))
                if length < 2*R+m+1e-7:
                    raise ValueError(f'Robot {i}-{j}: segmentos no separables en subpaso {k} (d={length:.3f})')
                normal=d/length; midpoint=(pa+pb)/2
                result[i][k].append((normal,float(normal@midpoint-R-m/2)))
                result[j][k].append((-normal,float(-normal@midpoint-R-m/2)))
    return result



def spacious_corridors(points, circles, padding=1.8, window=2):
    """Larger *certified* corridors with per-piece look-ahead envelopes.

    The trust rectangle for interval k encloses the nearby CBS vertices
    [k-window, k+1+window] padded by `padding` meters.  Obstacles outside
    this rectangle are provably irrelevant; nearby circular obstacles use
    a supporting halfspace (as in the original implementation).

    For each synchronized robot pair, if their trust rectangles already
    have sufficient separation, they need no extra SVM halfspace. Otherwise
    the usual maximum-margin separating line for the CBS segments is used.

    The continuous Bezier control hull is later checked against *all* these
    inequalities. Because controls lie in their rectangles, omitting
    outside disks is safe, with respect to the modeled circular obstacles.
    """
    if not np.isfinite(padding) or padding <= 0 or not isinstance(window,int) or window < 0:
        raise ValueError('padding debe ser positivo y window un entero no negativo')
    n, steps, _ = points.shape
    K = steps - 1
    out = [[[] for _ in range(K)] for _ in range(n)]
    rectangles = [[None for _ in range(K)] for _ in range(n)]
    xlo, xhi = geom.X_MIN+R, geom.X_MAX-R
    ylo, yhi = geom.Y_MIN+R, geom.Y_MAX-R
    for k in range(K):
        for i in range(n):
            track = points[i, max(0,k-window): min(steps,k+2+window)]
            lo = track.min(axis=0) - padding
            hi = track.max(axis=0) + padding
            lo = np.maximum(lo, (xlo,ylo))
            hi = np.minimum(hi, (xhi,yhi))
            if np.any(hi < lo):
                raise ValueError('Rectángulo de confianza vacío')
            rectangles[i][k] = (lo,hi)
            hs=out[i][k]
            hs.extend([(np.array([1.,0.]), float(hi[0])),
                       (np.array([-1.,0.]), float(-lo[0])),
                       (np.array([0.,1.]), float(hi[1])),
                       (np.array([0.,-1.]), float(-lo[1]))])
            seg = points[i,k:k+2]
            for center, exclusion in circles:
                center=np.asarray(center,float)
                # Exact min center-to-box distance. If larger than exclusion,
                # the whole box is already outside this forbidden disk.
                box_closest=np.maximum(lo,np.minimum(center,hi))
                if np.linalg.norm(center-box_closest) >= exclusion+1e-7:
                    continue
                hs.append(disk_halfspace(seg,center,exclusion))
        for i in range(n):
            for j in range(i+1,n):
                loi,hii=rectangles[i][k]
                loj,hij=rectangles[j][k]
                # Euclidean distance between two axis-aligned rectangles.
                gap=np.maximum(0,np.maximum(loj-hii,loi-hij))
                if np.linalg.norm(gap)>=2*R+1e-7:
                    continue
                pa,pb=closest_segments(points[i,k:k+2],points[j,k:k+2])
                diff=pb-pa;dist=float(np.linalg.norm(diff))
                if dist <= 2*R+1e-7:
                    raise ValueError(f'Robot {i}-{j}: segmentos no separables en k={k}, d={dist:.3f}')
                normal=diff/dist
                beta=float(normal@(pa+pb)/2)
                out[i][k].append((normal,beta-R))
                out[j][k].append((-normal,-beta-R))
    return out


def spacious_corridors(points, circles, padding=1.8, window=2):
    """Larger *certified* corridors with per-piece look-ahead envelopes.

    The trust rectangle for interval k encloses the nearby CBS vertices
    [k-window, k+1+window] padded by `padding` meters.  Obstacles outside
    this rectangle are provably irrelevant; nearby circular obstacles use
    a supporting halfspace (as in the original implementation).

    For each synchronized robot pair, if their trust rectangles already
    have sufficient separation, they need no extra SVM halfspace. Otherwise
    the usual maximum-margin separating line for the CBS segments is used.

    The continuous Bezier control hull is later checked against *all* these
    inequalities. Because controls lie in their rectangles, omitting
    outside disks is safe, with respect to the modeled circular obstacles.
    """
    if not np.isfinite(padding) or padding <= 0 or not isinstance(window,int) or window < 0:
        raise ValueError('padding debe ser positivo y window un entero no negativo')
    n, steps, _ = points.shape
    K = steps - 1
    out = [[[] for _ in range(K)] for _ in range(n)]
    rectangles = [[None for _ in range(K)] for _ in range(n)]
    xlo, xhi = geom.X_MIN+R, geom.X_MAX-R
    ylo, yhi = geom.Y_MIN+R, geom.Y_MAX-R
    for k in range(K):
        for i in range(n):
            track = points[i, max(0,k-window): min(steps,k+2+window)]
            lo = track.min(axis=0) - padding
            hi = track.max(axis=0) + padding
            lo = np.maximum(lo, (xlo,ylo))
            hi = np.minimum(hi, (xhi,yhi))
            if np.any(hi < lo):
                raise ValueError('Rectángulo de confianza vacío')
            rectangles[i][k] = (lo,hi)
            hs=out[i][k]
            hs.extend([(np.array([1.,0.]), float(hi[0])),
                       (np.array([-1.,0.]), float(-lo[0])),
                       (np.array([0.,1.]), float(hi[1])),
                       (np.array([0.,-1.]), float(-lo[1]))])
            seg = points[i,k:k+2]
            for center, exclusion in circles:
                center=np.asarray(center,float)
                # Exact min center-to-box distance. If larger than exclusion,
                # the whole box is already outside this forbidden disk.
                box_closest=np.maximum(lo,np.minimum(center,hi))
                if np.linalg.norm(center-box_closest) >= exclusion+1e-7:
                    continue
                hs.append(disk_halfspace(seg,center,exclusion))
        for i in range(n):
            for j in range(i+1,n):
                loi,hii=rectangles[i][k]
                loj,hij=rectangles[j][k]
                # Euclidean distance between two axis-aligned rectangles.
                gap=np.maximum(0,np.maximum(loj-hii,loi-hij))
                if np.linalg.norm(gap)>=2*R+1e-7:
                    continue
                pa,pb=closest_segments(points[i,k:k+2],points[j,k:k+2])
                diff=pb-pa;dist=float(np.linalg.norm(diff))
                if dist <= 2*R+1e-7:
                    raise ValueError(f'Robot {i}-{j}: segmentos no separables en k={k}, d={dist:.3f}')
                normal=diff/dist
                beta=float(normal@(pa+pb)/2)
                out[i][k].append((normal,beta-R))
                out[j][k].append((-normal,-beta-R))
    return out

def subdivide(paths, sub):
    K=max(map(len,paths))-1
    arr=[]
    for path in paths:
        xy=np.asarray([geom.node_position(v) for v in path],float)
        xy=np.concatenate((xy,np.repeat(xy[-1:], K+1-len(xy),axis=0)),axis=0)
        arr.append(np.array([(1-k/sub)*xy[t]+(k/sub)*xy[t+1] for t in range(K) for k in range(sub)]+[xy[-1]]))
    return np.asarray(arr)


def optimize_robot(pts, corridors_for_robot, dt, iterations=6000, tolerance=8e-5,
                   active_pieces=None):
    """Convex QP via sparse ADMM, optionally fixing the post-arrival tail.

    active_pieces is the number of intervals before the robot's discrete arrival.
    Keeping the tail exactly constant preserves early goal arrival semantics.
    The QP optimizes only active intervals; all later controls are set to goal.
    """
    total_pieces=len(pts)-1
    active_pieces = total_pieces if active_pieces is None else int(active_pieces)
    if not (0 <= active_pieces <= total_pieces):
        raise ValueError('active_pieces fuera de rango')
    d=D
    if not total_pieces:
        return np.empty((0,d+1,2)),0.,'STATIC'
    if active_pieces == 0:
        return np.broadcast_to(pts[0], (total_pieces,d+1,2)).copy(),0.,'STATIC'
    if active_pieces < total_pieces:
        # Recursion with active-only portion: endpoint velocity is zero there.
        active, residual, status=optimize_robot(
            pts[:active_pieces+1], corridors_for_robot[:active_pieces], dt,
            iterations=iterations, tolerance=tolerance)
        parked=np.broadcast_to(pts[active_pieces],
                               (total_pieces-active_pieces,d+1,2)).copy()
        return np.concatenate((active,parked),axis=0),residual,status
    K=total_pieces; nv=K*(d+1)*2
    # Feasible stationary-at-each-knot initialization, C2 at joins.
    weights=np.array([0.,0.,0.,1.,1.,1.])
    initial=np.array([pts[k][None,:]*(1-weights[:,None])+pts[k+1][None,:]*weights[:,None] for k in range(K)])
    xx,ww=np.polynomial.legendre.leggauss(10);xx=(xx+1)/2;ww=ww/2
    Q=np.zeros((d+1,d+1))
    for s,w in zip(xx,ww):
        b=bernstein(d-2,s)
        dd=d*(d-1)*(np.pad(b,(2,0))-2*np.pad(b,(1,1))+np.pad(b,(0,2)))
        Q+=w*np.outer(dd,dd)/dt**3
    # Multiplicar el objetivo por una constante positiva conserva el minimizador.
    # La normalizacion mejora enormemente el balance numerico con ADMM.
    Q /= max(float(np.linalg.norm(Q, ord=2)), 1e-12)
    from scipy.sparse import lil_matrix
    H=lil_matrix((nv,nv))
    for k in range(K):
        for c in range(2):
            ids=[(k*(d+1)+j)*2+c for j in range(d+1)]
            H[np.ix_(ids,ids)]=Q
    H=H.tocsc()+eye(nv,format='csc')*1e-7
    eq=[];be=[];ineq=[];ub=[]
    def idx(k,j,c):return (k*(d+1)+j)*2+c
    def row(values,target,kind):
        inds=[idx(k,j,c) for k,j,c,w in values]
        z=np.zeros(nv)
        for (k,j,c,w),pos in zip(values,inds):z[pos]+=w
        (eq if kind=='eq' else ineq).append(z)
        (be if kind=='eq' else ub).append(target)
    for c in range(2):
        row([(0,0,c,1)],pts[0,c],'eq');row([(K-1,d,c,1)],pts[-1,c],'eq')
        row([(0,1,c,1),(0,0,c,-1)],0,'eq')
        row([(K-1,d,c,1),(K-1,d-1,c,-1)],0,'eq')
        # Acceleration is also zero at phase endpoints; ensures C2 continuity
        # when an early-arriving robot's remaining intervals are stationary.
        row([(0,2,c,1),(0,1,c,-2),(0,0,c,1)],0,'eq')
        row([(K-1,d,c,1),(K-1,d-1,c,-2),(K-1,d-2,c,1)],0,'eq')
    for k in range(K-1):
        for c in range(2):
            row([(k,d,c,1),(k+1,0,c,-1)],0,'eq')
            row([(k,d,c,1),(k,d-1,c,-1),(k+1,1,c,-1),(k+1,0,c,1)],0,'eq')
            row([(k,d,c,1),(k,d-1,c,-2),(k,d-2,c,1),(k+1,2,c,-1),(k+1,1,c,2),(k+1,0,c,-1)],0,'eq')
    for k,halfspaces in enumerate(corridors_for_robot):
        for normal, bound in halfspaces:
            for j in range(d+1):
                row([(k,j,0,float(normal[0])),(k,j,1,float(normal[1]))],bound,'ub')
    Aeq=csc_matrix(np.array(eq));Aub=csc_matrix(np.array(ineq));A=vstack([Aeq,Aub]).tocsc()
    be=np.array(be);ub=np.array(ub);m=len(be)
    rho=8.; solve=splu(H+rho*(A.T@A)+eye(nv,format='csc')*1e-9)
    z=initial.ravel().copy();Az=np.asarray(A@z).ravel();q=Az.copy();q[:m]=be;q[m:]=np.minimum(q[m:],ub)
    dual=np.zeros_like(q)
    for iteration in range(iterations):
        z=solve.solve(rho*np.asarray(A.T@(q-dual)).ravel())
        Az=np.asarray(A@z).ravel();previous=q.copy()
        q=Az+dual;q[:m]=be;q[m:]=np.minimum(q[m:],ub)
        dual+=Az-q
        if iteration%50==0 and np.max(np.abs(Az-q))<tolerance/10 and np.max(np.abs(q-previous))<tolerance/10:break
    violation=max(float(np.max(np.abs(Aeq@z-be))),float(np.max(Aub@z-ub)))
    if violation>tolerance:
        # Known feasible baseline is not silently described as optimized.
        baseline=initial.ravel(); baseline_violation=max(float(np.max(np.abs(Aeq@baseline-be))),float(np.max(Aub@baseline-ub)))
        if baseline_violation>tolerance:
            raise RuntimeError(f'QP no factible: violacion={violation:g}, inicial={baseline_violation:g}')
        z=baseline;violation=baseline_violation
        status='BASELINE'
    else:status='QP'
    return z.reshape(K,d+1,2),float(violation),status


def sample(controls, per_piece=15):
    if not len(controls):return np.empty((0,2))
    return np.vstack([bernstein(D,s)@c for c in controls for s in np.linspace(0,1,per_piece,endpoint=False)]+[controls[-1,-1]])


def _hull_edges(points):
    """Convex hull of control points; unlike curve samples this preserves feasibility."""
    from scipy.spatial import ConvexHull, QhullError
    pts=np.unique(np.round(np.asarray(points).reshape(-1,2),11),axis=0)
    if len(pts)>2:
        try:pts=pts[ConvexHull(pts).vertices]
        except QhullError:
            direction=pts[np.argmax(np.linalg.norm(pts-pts[0],axis=1))]-pts[0]
            ids=np.argsort(pts@direction)
            pts=pts[[ids[0],ids[-1]]]
    if len(pts)==1:return [(pts[0],pts[0])]
    if len(pts)==2:return [(pts[0],pts[1])]
    return [(pts[k],pts[(k+1)%len(pts)]) for k in range(len(pts))]


def _segment_nearest(a,b,c,d):
    """Closest points between line segments via endpoint projections (2D)."""
    a,b,c,d=map(lambda v:np.asarray(v,float),(a,b,c,d))
    def projection(p,u,v):
        w=v-u;den=float(w@w)
        return u+np.clip((p-u)@w/den,0,1)*w if den>1e-18 else u
    candidates=[(a,projection(a,c,d)),(b,projection(b,c,d)),
                (projection(c,a,b),c),(projection(d,a,b),d)]
    return min(candidates,key=lambda pq:float(np.sum((pq[0]-pq[1])**2)))


def _convex_separators(controls,circles):
    """Repartition using convex hulls of current Bezier controls.

    Unlike sampling the curve, hulls contain *all* previous controls, so the
    current solution is feasible in the rebuilt corridor (up to solver tolerance).
    """
    n=len(controls);K=len(controls[0]);out=[[[] for _ in range(K)] for _ in range(n)]
    polygons=[[_hull_edges(controls[i][k]) for k in range(K)] for i in range(n)]
    for k in range(K):
        for i in range(n):
            pts=np.asarray(controls[i][k]);hs=out[i][k]
            hs.extend([(np.array([1.,0.]),geom.X_MAX-R),
                       (np.array([-1.,0.]),-(geom.X_MIN+R)),
                       (np.array([0.,1.]),geom.Y_MAX-R),
                       (np.array([0.,-1.]),-(geom.Y_MIN+R))])
            for center,exclusion in circles:
                center=np.asarray(center,float)
                closest=min((_segment_nearest(a,b,center,center)[0]
                             for a,b in polygons[i][k]),
                            key=lambda p:float(np.sum((p-center)**2)))
                delta=center-closest;dist=np.linalg.norm(delta)
                if dist<=exclusion+2e-4:raise ValueError('Poliedro demasiado proximo a obstaculo')
                normal=delta/dist
                bound=float(normal@center-exclusion)
                if np.max(pts@normal)>bound+2e-4:
                    raise ValueError('No se pudo certificar corredor frente a obstaculo')
                hs.append((normal,bound))
        for i in range(n):
            for j in range(i+1,n):
                candidates=[_segment_nearest(a,b,c,d)
                            for a,b in polygons[i][k] for c,d in polygons[j][k]]
                pa,pb=min(candidates,key=lambda pq:float(np.sum((pq[0]-pq[1])**2)))
                diff=pb-pa;dist=np.linalg.norm(diff)
                if dist <= 2*R+2e-4:raise ValueError('Hulls no separables')
                normal=diff/dist
                beta=float(normal@(pa+pb)/2)
                if np.max(controls[i][k]@normal)>beta-R+2e-4 or np.min(controls[j][k]@normal)<beta+R-2e-4:
                    raise ValueError('Plano no separa ambos hulls')
                out[i][k].append((normal,beta-R))
                out[j][k].append((-normal,-beta-R))
    return out


def _sample_separators(controls,circles,sample_count=32):
    """VII-D: repartition using S uniformly spaced samples of each Bezier piece.

    Sample hulls may exclude previous Bezier control points. Thus previous
    QP controls are NOT guaranteed feasible in the new corridors. Accept only
    new QP solutions whose control points satisfy all inequalities.
    """
    if sample_count < 3:
        raise ValueError("sample_count must be >= 3")
    n=len(controls);K=len(controls[0]);out=[[[] for _ in range(K)] for _ in range(n)]
    samples=[[np.array([bernstein(D,t)@controls[i][k] for t in np.linspace(0,1,sample_count)])
              for k in range(K)] for i in range(n)]
    polygons=[[_hull_edges(samples[i][k]) for k in range(K)] for i in range(n)]
    for k in range(K):
        for i in range(n):
            pts=samples[i][k];hs=out[i][k]
            hs.extend([(np.array([1.,0.]),geom.X_MAX-R),
                       (np.array([-1.,0.]),-(geom.X_MIN+R)),
                       (np.array([0.,1.]),geom.Y_MAX-R),
                       (np.array([0.,-1.]),-(geom.Y_MIN+R))])
            for center,exclusion in circles:
                center=np.asarray(center,float)
                closest=min((_segment_nearest(a,b,center,center)[0]
                             for a,b in polygons[i][k]),
                            key=lambda p:float(np.sum((p-center)**2)))
                delta=center-closest;dist=np.linalg.norm(delta)
                if dist<=exclusion+2e-4:raise ValueError('Poliedro demasiado proximo a obstaculo')
                normal=delta/dist
                bound=float(normal@center-exclusion)
                if np.max(pts@normal)>bound+2e-4:
                    raise ValueError('No se pudo certificar corredor frente a obstaculo')
                hs.append((normal,bound))
        for i in range(n):
            for j in range(i+1,n):
                candidates=[_segment_nearest(a,b,c,d)
                            for a,b in polygons[i][k] for c,d in polygons[j][k]]
                pa,pb=min(candidates,key=lambda pq:float(np.sum((pq[0]-pq[1])**2)))
                diff=pb-pa;dist=np.linalg.norm(diff)
                if dist <= 2*R+2e-4:raise ValueError('Hulls no separables')
                normal=diff/dist
                beta=float(normal@(pa+pb)/2)
                if np.max(samples[i][k]@normal)>beta-R+2e-4 or np.min(samples[j][k]@normal)<beta+R-2e-4:
                    raise ValueError('Plano no separa ambos hulls')
                out[i][k].append((normal,beta-R))
                out[j][k].append((-normal,-beta-R))
    return out


def _acceleration_cost(controls,dt):
    """Physical integrated squared acceleration, not the solver-normalized cost."""
    xx,ww=np.polynomial.legendre.leggauss(9)
    cost=0.
    for p in controls:
        diff=np.diff(p,n=2,axis=0)*D*(D-1)
        for s,w in zip((xx+1)/2,ww/2):
            acc=bernstein(D-2,s)@diff/dt**2
            cost+=float(w*acc@acc*dt)
    return cost


def _metrics(ctrls,sub,samples_per_piece):
    arr=np.array([sample(c,samples_per_piece) for c in ctrls])
    dist=min((float(np.linalg.norm(arr[i]-arr[j],axis=1).min())
              for i in range(len(arr)) for j in range(i+1,len(arr))),default=float('inf'))
    cost=sum(_acceleration_cost(c,1/sub) for c in ctrls)
    return arr,dist,cost


def optimize_cbs_trajectories(paths, graph, subdivision=1, max_subdivision=16,
                              samples_per_piece=12, refinement_iterations=0, refinement_samples=32,
                              corridor_mode="legacy", corridor_padding=1.8, corridor_window=2):
    if corridor_mode not in ('legacy','spacious'):
        raise ValueError('corridor_mode desconocido')
    if corridor_mode not in ('legacy','spacious'):
        raise ValueError('corridor_mode desconocido')
    if subdivision<1 or max_subdivision<subdivision or refinement_iterations<0 or refinement_samples<3:
        raise ValueError('Parametros de optimizacion invalidos')
    circles=obstacles(graph)
    last_error=None
    for sub in (subdivision,subdivision*2,subdivision*4,subdivision*8,subdivision*16):
        if sub>max_subdivision:break
        points=subdivide(paths,sub)
        try:
            halfspaces = (spacious_corridors(points,circles,padding=corridor_padding,window=corridor_window)
                          if corridor_mode == 'spacious' else corridors(points,circles))
        except ValueError as ex:last_error=ex;continue
        ctrls=[];statuses=[];residuals=[]
        for i in range(len(paths)):
            control,residual,status=optimize_robot(
                points[i],halfspaces[i],1/sub,active_pieces=(len(paths[i])-1)*sub)
            ctrls.append(control);residuals.append(residual);statuses.append(status)
        # Certify the complete continuous curves by their control hulls.
        for i,ctrl in enumerate(ctrls):
            for k,hs in enumerate(halfspaces[i]):
                worst=max((float(np.max(ctrl[k]@normal-bound)) for normal,bound in hs),default=0.)
                if worst>1.2e-4:
                    raise RuntimeError(f'Corredor no satisfecho: robot={i}, intervalo={k}, violation={worst:.3g}')
        sampled,min_pair,score=_metrics(ctrls,sub,samples_per_piece)
        if min_pair<2*R-1e-4:raise RuntimeError('Separacion interrobot insuficiente')
        history=[{'iteration':0,'cost_acceleration':score,'min_pair_m':min_pair,
                  'status':list(statuses)}]
        for iteration in range(1,refinement_iterations+1):
            try:
                new_halfspaces=_sample_separators(ctrls,circles,refinement_samples)
                new_ctrls=[];new_status=[];new_res=[]
                for i in range(len(paths)):
                    # The next QP retains start/end points but can modify interior knots.
                    solution,residual,status=optimize_robot(
                        points[i],new_halfspaces[i],1/sub,
                        active_pieces=(len(paths[i])-1)*sub)
                    if status not in ('QP','STATIC'):raise RuntimeError('QP no convergio en el corredor muestreado')
                    # Strong convex-hull certificate: *every new control point* is
                    # inside the new corridor, hence whole Bezier piece is safe.
                    worst=max((float(np.max(solution[k]@n-bound))
                               for k,hs in enumerate(new_halfspaces[i])
                               for n,bound in hs),default=0.)
                    if worst>8e-5:raise RuntimeError(f'Violacion de corredor: {worst:.3g}')
                    new_ctrls.append(solution);new_status.append(status);new_res.append(residual)
                new_sampled,new_pair,new_score=_metrics(new_ctrls,sub,samples_per_piece)
                if new_pair<2*R-1e-4 or new_score>score*(1+1e-3):
                    raise RuntimeError('No mejora costo o distancia de seguridad')
                if new_score>=score*(1-1e-5):
                    history.append({'iteration':iteration,'accepted':False,
                                    'reason':'convergencia','cost_acceleration':score,'sample_count':refinement_samples})
                    break
                ctrls,statuses,residuals=new_ctrls,new_status,new_res
                sampled,min_pair,score=new_sampled,new_pair,new_score
                history.append({'iteration':iteration,'accepted':True,
                                'cost_acceleration':score,'min_pair_m':min_pair,'status':list(statuses),'sample_count':refinement_samples})
            except (ValueError,RuntimeError) as ex:
                history.append({'iteration':iteration,'accepted':False,'reason':str(ex),'sample_count':refinement_samples})
                break
        return {'sampled':sampled,'controls':ctrls,'subdivision':sub,'min_pair':min_pair,
                'status':statuses,'residuals':residuals,'clearance':2*R,
                'refinement_history':history,'cost_acceleration':score,
                'arrival_intervals':[len(p)-1 for p in paths], 'corridor_mode':corridor_mode}
    raise RuntimeError(f'No existen corredores espaciales separables: {last_error}')


# Previous conservative variant kept only as reference for regression tests.
def _convex_separators(controls,circles):
    """Repartition using convex hulls of current Bezier controls.

    Unlike sampling the curve, hulls contain *all* previous controls, so the
    current solution is feasible in the rebuilt corridor (up to solver tolerance).
    """
    n=len(controls);K=len(controls[0]);out=[[[] for _ in range(K)] for _ in range(n)]
    polygons=[[_hull_edges(controls[i][k]) for k in range(K)] for i in range(n)]
    for k in range(K):
        for i in range(n):
            pts=np.asarray(controls[i][k]);hs=out[i][k]
            hs.extend([(np.array([1.,0.]),geom.X_MAX-R),
                       (np.array([-1.,0.]),-(geom.X_MIN+R)),
                       (np.array([0.,1.]),geom.Y_MAX-R),
                       (np.array([0.,-1.]),-(geom.Y_MIN+R))])
            for center,exclusion in circles:
                center=np.asarray(center,float)
                closest=min((_segment_nearest(a,b,center,center)[0]
                             for a,b in polygons[i][k]),
                            key=lambda p:float(np.sum((p-center)**2)))
                delta=center-closest;dist=np.linalg.norm(delta)
                if dist<=exclusion+2e-4:raise ValueError('Poliedro demasiado proximo a obstaculo')
                normal=delta/dist
                bound=float(normal@center-exclusion)
                if np.max(pts@normal)>bound+2e-4:
                    raise ValueError('No se pudo certificar corredor frente a obstaculo')
                hs.append((normal,bound))
        for i in range(n):
            for j in range(i+1,n):
                candidates=[_segment_nearest(a,b,c,d)
                            for a,b in polygons[i][k] for c,d in polygons[j][k]]
                pa,pb=min(candidates,key=lambda pq:float(np.sum((pq[0]-pq[1])**2)))
                diff=pb-pa;dist=np.linalg.norm(diff)
                if dist <= 2*R+2e-4:raise ValueError('Hulls no separables')
                normal=diff/dist
                beta=float(normal@(pa+pb)/2)
                if np.max(controls[i][k]@normal)>beta-R+2e-4 or np.min(controls[j][k]@normal)<beta+R-2e-4:
                    raise ValueError('Plano no separa ambos hulls')
                out[i][k].append((normal,beta-R))
                out[j][k].append((-normal,-beta-R))
    return out

