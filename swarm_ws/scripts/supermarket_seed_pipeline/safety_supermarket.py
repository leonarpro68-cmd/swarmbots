"""Offline conservative certification adapted from original hybrid_v2/safety.py.

Checks convex hulls of all six Bezier controls (not only curve samples).
Does NOT alter original_hybrid/safety.py. Reports dynamic upper bounds with a
common conservative physical duration per CBS tick, independent of V3.
"""
from __future__ import annotations
from dataclasses import dataclass
from itertools import combinations
import math
import numpy as np
from shapely.geometry import MultiPoint,Point
from corridors_supermarket import Rectangle

@dataclass(frozen=True)
class Settings:
    margin_m: float=0.0
    max_speed_m_s: float=1.20
    max_acceleration_m_s2: float=1.0
    time_safety_factor: float=1.08
    tolerance_m: float=1e-6
    preserve_intermediate_waits: bool=False

def certify(controls, points, paths, rectangles, bounds, rectangle_clearance_m,
            circles_by_robot, halfspaces, *, wall_clearance_m, robot_radius_m,
            settings=Settings()):
    c=np.asarray(controls,float); p=np.asarray(points,float)
    n,K=c.shape[:2];tol=settings.tolerance_m; margin=settings.margin_m
    if c.shape!=(n,K,6,2) or p.shape!=(n,K+1,2) or len(paths)!=n or len(circles_by_robot)!=n or len(halfspaces)!=n:
        raise ValueError('Dimensiones incompatibles')
    if not np.isfinite(c).all():raise ValueError('Controles no finitos')
    xmin,xmax,ymin,ymax=map(float,bounds)
    hulls=[[MultiPoint(c[i,k]).convex_hull for k in range(K)] for i in range(n)]
    rects=[(r,r.shape()) for r in rectangles]
    min_rect=min_obj=min_pair=min_wall=math.inf
    max_hs=-math.inf;max_c2=0.;max_end=0.;max_wait=0.; vmax=0.;amax=0.
    errors=[]
    for i in range(n):
        if len(halfspaces[i])!=K:raise ValueError('Semiplanos mal dimensionados')
        active=len(paths[i])-1
        if not 1<=active<=K:raise ValueError('Longitud de ruta inválida')
        max_end=max(max_end,float(np.linalg.norm(c[i,0,0]-p[i,0])),float(np.linalg.norm(c[i,-1,-1]-p[i,-1])))
        for k in range(K):
            pc=c[i,k];h=hulls[i][k]
            x0,y0,x1,y1=h.bounds
            slack=min(x0-xmin-wall_clearance_m,xmax-wall_clearance_m-x1,y0-ymin-wall_clearance_m,ymax-wall_clearance_m-y1)
            min_wall=min(min_wall,slack)
            if slack<margin-tol:errors.append(f'R{i} k{k}: muro {slack:.4f}')
            for normal,b in halfspaces[i][k]:
                v=float(np.max(pc@np.asarray(normal)-float(b)))
                max_hs=max(max_hs,v)
                if v>tol:errors.append(f'R{i} k{k}: semiplano {v:.4g}')
            for r,shape in rects:
                d=h.distance(shape)-rectangle_clearance_m
                min_rect=min(min_rect,d)
                if d<margin-tol:errors.append(f'R{i} k{k}: {r.name} {d:.4f}')
            for center,radius in circles_by_robot[i]:
                d=h.distance(Point(center))-radius
                min_obj=min(min_obj,d)
                if d<margin-tol:errors.append(f'R{i} k{k}: basura/deposito {d:.4f}')
            if k>=active or (settings.preserve_intermediate_waits and k<active and paths[i][k]==paths[i][k+1]):
                target=p[i,-1] if k>=active else p[i,k]
                e=float(np.max(np.linalg.norm(pc-target,axis=1)))
                max_wait=max(max_wait,e)
                if e>tol:errors.append(f'R{i} k{k}: wait {e:.4g}')
            vmax=max(vmax,float(np.linalg.norm(5*np.diff(pc,axis=0),axis=1).max()))
            amax=max(amax,float(np.linalg.norm(20*np.diff(pc,n=2,axis=0),axis=1).max()))
            if k<K-1:
                nx=c[i,k+1]
                jumps=[pc[5]-nx[0],(pc[5]-pc[4])-(nx[1]-nx[0]),(pc[5]-2*pc[4]+pc[3])-(nx[2]-2*nx[1]+nx[0])]
                jump=max(float(np.linalg.norm(z)) for z in jumps)
                max_c2=max(max_c2,jump)
                if jump>tol:errors.append(f'R{i} k{k}: C2 {jump:.4g}')
    for i,j in combinations(range(n),2):
        for k in range(K):
            d=hulls[i][k].distance(hulls[j][k])-2*robot_radius_m
            min_pair=min(min_pair,d)
            if d<margin-tol:errors.append(f'R{i}-R{j} k{k}: separacion {d:.4f}')
    if max_end>tol:errors.append(f'Extremos {max_end:.4g}')
    dt=settings.time_safety_factor*max(vmax/settings.max_speed_m_s,math.sqrt(amax/settings.max_acceleration_m_s2),1e-3)
    return dict(pass_certificate=not errors,errors=errors[:25],error_count=len(errors),robots=n,pieces=K,
                min_rectangle_extra_clearance_m=min_rect,min_dynamic_object_extra_clearance_m=min_obj if math.isfinite(min_obj) else None,
                min_interrobot_extra_clearance_m=min_pair,min_wall_extra_clearance_m=min_wall,
                max_halfspace_violation_m=max(0,max_hs),max_C2_residual=max_c2,max_endpoint_residual_m=max_end,max_wait_residual_m=max_wait,
                max_speed_per_virtual_tick_m=vmax,max_acceleration_per_virtual_tick2_m=amax,
                conservative_uniform_tick_seconds=dt,conservative_stage_duration_seconds=K*dt,
                speed_bound_m_s=vmax/dt,acceleration_bound_m_s2=amax/dt**2,
                limits=dict(max_speed_m_s=settings.max_speed_m_s,max_acceleration_m_s2=settings.max_acceleration_m_s2),
                margin_m=margin,
                note='Certificacion conservadora de envolventes convexas; el dt uniforme es solo una cota offline, no V3 ni una ejecucion Gazebo')
