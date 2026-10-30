"""Hybrid V2 pre-execution certification. Offline, no ROS side effects.

Geometry uses convex hull containment of each Bezier piece, which is
conservative and certifies the *whole* polynomial, not only samples.
"""
from __future__ import annotations
from dataclasses import dataclass, asdict
from itertools import combinations
import math
import numpy as np
from shapely.geometry import MultiPoint, Point
from . import geometry_reference as geom


@dataclass(frozen=True)
class SafetySettings:
    operational_margin_m: float = 0.05
    max_speed_m_s: float = 1.20
    max_acceleration_m_s2: float = 1.0
    time_safety_factor: float = 1.08
    constraint_tolerance_m: float = 1e-6
    stationary_tolerance_m: float = 1e-6

    def __post_init__(self):
        values = (self.operational_margin_m, self.max_speed_m_s,
                  self.max_acceleration_m_s2, self.time_safety_factor,
                  self.constraint_tolerance_m, self.stationary_tolerance_m)
        if any(not math.isfinite(x) for x in values):
            raise ValueError('Parámetros de seguridad no finitos')
        if self.operational_margin_m < 0 or self.max_speed_m_s <= 0 or self.max_acceleration_m_s2 <= 0:
            raise ValueError('Márgenes y límites dinámicos inválidos')
        if self.time_safety_factor < 1 or self.constraint_tolerance_m < 0 or self.stationary_tolerance_m < 0:
            raise ValueError('Factor temporal o tolerancias inválidos')


def certify(controls, points, node_paths, obstacles, *, settings=SafetySettings(), halfspaces=None, preserve_intermediate_waits=False):
    """Return a report only for safe plans; otherwise raise ValueError.

    ``obstacles`` are (center, *existing center-exclusion radius*): the
    physical robot clearance is already accounted for in that radius.
    ``operational_margin_m`` is an additional execution reserve.

    Final post-arrival pieces must be constant. Intermediate CBS WAITs
    are permitted to relax when preserve_intermediate_waits=False, consistent
    with the reference experiment; synchronized hull safety is still checked.
    """
    p = np.asarray(points, dtype=float)
    c = np.asarray(controls, dtype=float)
    if p.ndim != 3 or p.shape[-1] != 2 or c.ndim != 4 or c.shape[2:] != (6, 2):
        raise ValueError('Dimensiones inválidas para puntos/controles')
    N, K, _, _ = c.shape
    if p.shape != (N, K + 1, 2) or len(node_paths) != N or not np.isfinite(c).all():
        raise ValueError('Rutas y controles desalineados o no finitos')
    if halfspaces is not None and (len(halfspaces) != N or any(len(h) != K for h in halfspaces)):
        raise ValueError('Dimensiones de corredores inválidas')
    tol = settings.constraint_tolerance_m
    margin = settings.operational_margin_m
    r = geom.CLEARANCE_WALL_M
    hulls = [[MultiPoint(c[i,k]).convex_hull for k in range(K)] for i in range(N)]
    min_obs, min_pair, min_wall = math.inf, math.inf, math.inf
    max_c2_jump = 0.
    max_speed_per_tick = 0.
    max_acc_per_tick2 = 0.
    for i in range(N):
        if len(node_paths[i]) < 2 or len(node_paths[i]) > K+1:
            raise ValueError('Ruta CBS inválida')
        if np.linalg.norm(c[i,0,0]-p[i,0])>tol or np.linalg.norm(c[i,-1,-1]-p[i,-1])>tol:
            raise ValueError(f'Robot {i}: extremos no coinciden con CBS')
        for k in range(K):
            piece = c[i,k]
            hull = hulls[i][k]
            # Center is confined to the warehouse with inflated wall clearance.
            x0,y0,x1,y1 = hull.bounds
            slack = min(x0-(geom.X_MIN+r), (geom.X_MAX-r)-x1,
                        y0-(geom.Y_MIN+r), (geom.Y_MAX-r)-y1)
            min_wall = min(min_wall, slack)
            if slack < margin-tol:
                raise ValueError(f'Robot {i} tramo {k}: margen de pared insuficiente ({slack:.4f}m)')
            if halfspaces is not None:
                for normal,limit in halfspaces[i][k]:
                    excess = max(np.asarray(piece)@np.asarray(normal,dtype=float)-float(limit))
                    if excess > tol:
                        raise ValueError(f'Robot {i} tramo {k}: fuera de corredor ({excess:.3g}m)')
            for center,radius in obstacles:
                d = hull.distance(Point(np.asarray(center,dtype=float))) - float(radius)
                min_obs = min(min_obs, d)
                if d < margin-tol:
                    raise ValueError(f'Robot {i} tramo {k}: obstáculo sin reserva ({d:.4f}m)')
            # WAIT detection uses the original unpadded CBS route.
            if preserve_intermediate_waits and k < len(node_paths[i])-1 and node_paths[i][k] == node_paths[i][k+1]:
                if np.max(np.linalg.norm(piece-p[i,k],axis=1)) > settings.stationary_tolerance_m:
                    raise ValueError(f'Robot {i} tramo {k}: WAIT de CBS no respetado')
            if k >= len(node_paths[i])-1:
                if np.max(np.linalg.norm(piece-p[i,-1],axis=1)) > settings.stationary_tolerance_m:
                    raise ValueError(f'Robot {i} tramo {k}: permanencia final no respetada')
            v = 5*np.diff(piece,axis=0)
            a = 20*np.diff(piece,n=2,axis=0)
            max_speed_per_tick = max(max_speed_per_tick, float(np.linalg.norm(v,axis=1).max()))
            max_acc_per_tick2 = max(max_acc_per_tick2,float(np.linalg.norm(a,axis=1).max()))
            if k<K-1:
                nxt=c[i,k+1]
                errors=(np.linalg.norm(piece[-1]-nxt[0]),
                        np.linalg.norm(5*(piece[-1]-piece[-2])-5*(nxt[1]-nxt[0])),
                        np.linalg.norm(20*(piece[-1]-2*piece[-2]+piece[-3])-20*(nxt[2]-2*nxt[1]+nxt[0])))
                max_c2_jump=max(max_c2_jump,*errors)
                if max(errors)>1e-5:
                    raise ValueError(f'Robot {i}: discontinuidad C2 en tramo {k}: {errors}')
    for i,j in combinations(range(N),2):
        for k in range(K):
            # Conservative: simultaneous convex hull separation implies continuous safety.
            d=hulls[i][k].distance(hulls[j][k])-2*r
            min_pair=min(min_pair,d)
            if d < margin-tol:
                raise ValueError(f'Robots {i}-{j}, tramo {k}: margen interrobot insuficiente ({d:.4f}m)')
    dt = settings.time_safety_factor*max(max_speed_per_tick/settings.max_speed_m_s,
                 math.sqrt(max_acc_per_tick2/settings.max_acceleration_m_s2),1e-3)
    return {'certified':True,'robots':N,'pieces':K,'dt_s':dt,'duration_s':K*dt,
            'speed_upper_bound_m_s':max_speed_per_tick/dt,
            'acceleration_upper_bound_m_s2':max_acc_per_tick2/(dt*dt),
            'min_obstacle_extra_clearance_m':min_obs if math.isfinite(min_obs) else None,
            'min_pair_extra_clearance_m':min_pair if math.isfinite(min_pair) else None,
            'min_wall_extra_clearance_m':min_wall,'continuity_max_error':max_c2_jump,
            'operational_margin_m':margin,'intermediate_wait_policy':('fixed' if preserve_intermediate_waits else 'relaxed_with_continuous_separation'),
            'note':'Conservative hull certification; assumes synchronized execution and modeled circular exclusion zones'}
