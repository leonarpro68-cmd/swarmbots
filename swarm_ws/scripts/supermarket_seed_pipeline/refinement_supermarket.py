"""Adaptador aislado del refinamiento de Hybrid V2 para estanterías rectangulares.

Conserva el algoritmo de 32 muestras, separación por intervalo y SLSQP
original. Solo sustituye la generación de semiplanos de obstáculos por las
geometrías de supermarket y añade certificado antes de aceptar refinamientos.
"""
import numpy as np
from shapely.geometry import Point, MultiPoint
from shapely.ops import nearest_points
from corridors_supermarket import rectangle_halfspace
from original_hybrid.geometry_corridors import bernstein
from original_hybrid.slsqp import solve_all
from original_hybrid.refinement import cost

def rectangle_support_allow_tangency(sampled, rectangle, radius, tol=1e-7):
    """Accept a tangent to the exclusion set, reject genuine overlap."""
    track=MultiPoint(sampled).convex_hull
    a,b=nearest_points(track,rectangle.shape())
    delta=np.asarray(b.coords[0])-np.asarray(a.coords[0]);distance=float(np.linalg.norm(delta))
    if distance<radius-tol or distance<=1e-12:
        raise ValueError(f"{rectangle.name}: convex hull inside clearance, distance={distance:.9f} < {radius:.9f}")
    n=delta/distance
    bound=float(np.min(rectangle.corners@n)-radius)
    if np.max(sampled@n-bound)>tol:
        raise ValueError(f"{rectangle.name}: sampled track not in tangent halfspace")
    return n,bound



def halfspaces_from_curves(controls, *, rectangles, bounds, static_clearance_m,
                           circles_by_robot, wall_clearance_m, robot_radius_m,
                           samples=32, margin=0.0):
    """Regenera separadores desde 32 muestras/pieza, sin sustituir el solver."""
    if samples < 3 or margin < 0:
        raise ValueError('samples>=3 y margin>=0 requeridos')
    controls=[np.asarray(c,float) for c in controls]
    N=len(controls); K=len(controls[0])
    if not all(c.shape==(K,6,2) for c in controls) or len(circles_by_robot)!=N:
        raise ValueError('Dimensiones incompatibles')
    ts=np.linspace(0,1,samples)
    curves=[[np.asarray([bernstein(5,t)@controls[i][k] for t in ts]) for k in range(K)] for i in range(N)]
    corridors=[[[] for _ in range(K)] for _ in range(N)]
    xmin,xmax,ymin,ymax=bounds
    for k in range(K):
        shapes=[MultiPoint(curves[i][k]).convex_hull for i in range(N)]
        for i in range(N):
            segment=curves[i][k]; h=corridors[i][k]; wall=wall_clearance_m+margin
            h.extend([(np.array([1.,0.]),xmax-wall),(-np.array([1.,0.]),-xmin-wall),
                      (np.array([0.,1.]),ymax-wall),(-np.array([0.,1.]),-ymin-wall)])
            for rect in rectangles:
                n,b=rectangle_support_allow_tangency(segment,rect,static_clearance_m+margin)
                h.append((n,b))
            for center,radius in circles_by_robot[i]:
                # Convex hull prevents a separator whose samples are safe but whose
                # convex envelope intersects a circular exclusion.
                a,b=nearest_points(shapes[i],Point(center))
                pa=np.asarray(a.coords[0]); pb=np.asarray(b.coords[0]); delta=pb-pa
                dist=np.linalg.norm(delta)
                exclusion = radius + margin
                # Tangency is a valid supporting separator; reject only real
                # intrusion. Keep a tiny rounding tolerance (1e-10 m), not a
                # smaller safety margin. Validate the entire sampled hull.
                if dist <= 1e-12 or dist < exclusion - 1e-10:
                    raise ValueError(f'R{i} k{k}: basura/depósito sin separador convexo '
                                     f'(d={dist:.12g}, exclusion={exclusion:.12g})')
                n=delta/dist
                bound=float(n@np.asarray(center)-exclusion)
                max_violation=float(np.max(segment@n-bound))
                if max_violation > 1e-10:
                    raise ValueError(f'R{i} k{k}: separación circular inválida '
                                     f'(violación={max_violation:.12g})')
                h.append((n,bound))
        for i in range(N):
            for j in range(i+1,N):
                a,b=nearest_points(shapes[i],shapes[j]);pa=np.asarray(a.coords[0]);pb=np.asarray(b.coords[0]);delta=pb-pa
                dist=float(np.linalg.norm(delta));R=robot_radius_m
                if dist<=2*R+margin+1e-8:
                    raise ValueError(f'R{i}-R{j} k{k}: curvas sin separador convexo; d={dist:.6f}')
                n=delta/dist;mid=(pa+pb)/2
                corridors[i][k].append((n,float(n@mid-R-margin/2)))
                corridors[j][k].append((-n,float(-n@mid-R-margin/2)))
    return corridors


def certify(controls, hs, points, paths, tol=1e-6):
    """Certifica los controles completos, no solo 32 muestras."""
    maximum=0.0;c2=0.0;endpoint=0.0;tail=0.0
    for i,c in enumerate(controls):
        c=np.asarray(c)
        for k,h in enumerate(hs[i]):
            for n,b in h:maximum=max(maximum,float(np.max(c[k]@n-b)))
        for k in range(len(c)-1):
            a,b=c[k],c[k+1]
            c2=max(c2,float(np.linalg.norm(a[5]-b[0])),float(np.linalg.norm(a[5]-a[4]-b[1]+b[0])),float(np.linalg.norm(a[5]-2*a[4]+a[3]-b[2]+2*b[1]-b[0])))
        active=len(paths[i])-1
        endpoint=max(endpoint,float(np.linalg.norm(c[0,0]-points[i,0])),float(np.linalg.norm(c[active-1,5]-points[i,active])))
        if active<len(c):tail=max(tail,float(np.max(np.linalg.norm(c[active:]-points[i,active],axis=2))))
    ok=max(maximum,c2,endpoint,tail)<=tol
    return dict(pass_certificate=bool(ok),max_halfspace_violation_m=maximum,max_C2_residual=c2,max_endpoint_residual_m=endpoint,max_wait_residual_m=tail)


def refine(points, paths, baseline, *, rectangles,bounds,static_clearance_m,
           circles_by_robot,wall_clearance_m,robot_radius_m,iterations=5,
           samples=32,margin=0.,maxiter=450,require_improvement=True,
           return_last_halfspaces=False):
    current=[np.array(c,copy=True) for c in baseline]
    history=[dict(iteration=0,accepted=True,cost=cost(current))]
    last_accepted_halfspaces=None
    for iteration in range(1,iterations+1):
        old_cost=cost(current)
        try:
            hs=halfspaces_from_curves(current,rectangles=rectangles,bounds=bounds,
                 static_clearance_m=static_clearance_m,circles_by_robot=circles_by_robot,
                 wall_clearance_m=wall_clearance_m,robot_radius_m=robot_radius_m,
                 samples=samples,margin=margin)
            # Historical null-space SLSQP retained unchanged. Previous controls
            # not currently passed as warm start (original interface has no init).
            new,stats=solve_all(points,paths,hs,require_success=False,maxiter=maxiter)
            cert=certify(new,hs,points,paths)
            solver_ok=all(x['success'] and x['inequality_violation']<=1e-6 and x['equality_violation']<=1e-6 for x in stats)
            newcost=cost(new)
            if not solver_ok or not cert['pass_certificate']:
                history.append(dict(iteration=iteration,accepted=False,reason='solver or certificate',solver=stats,certificate=cert))
                break
            if require_improvement and newcost>old_cost+1e-8:
                history.append(dict(iteration=iteration,accepted=False,reason='cost increased',candidate_cost=newcost,previous_cost=old_cost,certificate=cert))
                break
            current=new
            # These are the exact halfspaces against which the accepted
            # Bernstein control points have been certified. They must be
            # reused by the final continuous certificate; regenerated
            # sample-based planes can differ and need not contain the controls.
            last_accepted_halfspaces=hs
            history.append(dict(iteration=iteration,accepted=True,cost=newcost,certificate=cert,solver=stats))
        except Exception as exc:
            history.append(dict(iteration=iteration,accepted=False,reason=f'{type(exc).__name__}: {exc}'))
            break
    if return_last_halfspaces:
        return current,history,last_accepted_halfspaces
    return current,history
