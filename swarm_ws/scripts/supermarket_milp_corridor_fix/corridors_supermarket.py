"""Corredores convexos para supermercado; adaptador *aislado* a Hybrid V2.

Salida idéntica al contrato de geometry_corridors.corridors del proyecto:
    corridors[i][k] == [(normal_2d, limite), ...]
    normal @ P[k,j] <= limite  (para los seis controles Bezier)

No se modifica ni se reimplementa slsqp.py del ZIP de SwarmBots.
Las estanterías son rectángulos físicos axis-aligned; clearance_m es la
exclusión TOTAL medida desde el centro del Summit hasta el rectángulo,
no un radio que deba volver a sumarse a ROBOT_RADIUS.
"""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from shapely.geometry import Point, MultiPoint, box
from shapely.ops import nearest_points
from original_hybrid.geometry_corridors import disk_halfspace, closest_segments
from clearance_config import (RECTANGLE_CENTER_CLEARANCE_M, WALL_CENTER_CLEARANCE_M,
                              INTERROBOT_HALFSEPARATOR_M)

# Radio de cada semiplano interrobot: (2*0.55 + 0.05)/2.
ROBOT_CENTER_CLEARANCE_M = INTERROBOT_HALFSEPARATOR_M
_DEFAULT_EPS = 1e-8


@dataclass(frozen=True)
class Rectangle:
    name: str
    xmin: float
    xmax: float
    ymin: float
    ymax: float

    @classmethod
    def from_json(cls, d):
        return cls(str(d['name']), *(float(d[k]) for k in ('xmin','xmax','ymin','ymax')))

    @property
    def corners(self):
        return np.array([[self.xmin, self.ymin], [self.xmin, self.ymax],
                         [self.xmax, self.ymin], [self.xmax, self.ymax]], dtype=float)

    def shape(self):
        return box(self.xmin,self.ymin,self.xmax,self.ymax)


def rectangle_halfspace(track, rectangle: Rectangle, clearance_m: float, *, eps=_DEFAULT_EPS):
    """Separador de soporte contra Q ⊕ disco(clearance_m).

    El segmento/CONJUNTO de muestras se protege completo: calculamos la
    distancia entre su envolvente CONVEXA y Q, no solo entre dos puntos.
    Para normal n (del track a Q): n·p <= min_{q en Q}(n·q)-clearance.
    Si el convex hull toca el obstáculo inflado, NO existe la certificación
    mediante un único semiplano en la dirección calculada: rechazar.

    Devuelve (normal 2D, limite, holgura_m). La envolvente convexa también
    hace válido este procedimiento para la reconstrucción futura por muestras,
    aunque esa funcionalidad NO se integra aún en refinement.py.
    """
    p=np.asarray(track,dtype=float)
    if p.ndim!=2 or p.shape[1]!=2 or not len(p) or not np.all(np.isfinite(p)):
        raise ValueError('track debe ser array finito (N,2), N>=1')
    if not np.isfinite(clearance_m) or clearance_m < 0:
        raise ValueError('clearance_m debe ser >= 0')
    q=rectangle
    if not (q.xmin<q.xmax and q.ymin<q.ymax and np.all(np.isfinite(q.corners))):
        raise ValueError(f'Rectángulo inválido: {q.name}')
    geom=Point(p[0]) if len(p)==1 else MultiPoint(p).convex_hull
    pa,pb=nearest_points(geom, q.shape())
    a=np.array(pa.coords[0]); b=np.array(pb.coords[0]); d=b-a
    dist=float(np.linalg.norm(d))
    if dist<=clearance_m+eps:
        raise ValueError(f'{q.name}: segmento/envolvente demasiado cerca de estantería: '
                         f'd={dist:.6f} m, exclusion={clearance_m:.6f} m')
    normal=d/dist
    # Soporte mínimo EXACTO sobre el rectángulo completo, no solo un punto.
    limit=float(np.min(q.corners @ normal)-clearance_m)
    slack=limit-float(np.max(p @ normal))
    if slack<=eps:
        raise ValueError(f'{q.name}: semiplano no contiene todo el tramo '
                         f'(holgura={slack:.8f} m)')
    return normal,limit,slack


def _check_shapes(points, circles, rectangles, bounds, circles_by_robot, margin):
    p=np.asarray(points,dtype=float)
    if p.ndim!=3 or p.shape[2]!=2 or p.shape[1]<2 or not np.all(np.isfinite(p)):
        raise ValueError('points debe ser (N,K+1,2) finito, K>=1')
    if not np.isfinite(margin) or margin<0:
        raise ValueError('operational_margin_m debe ser >= 0')
    if len(bounds)!=4 or any(not np.isfinite(v) for v in bounds):
        raise ValueError('bounds inválidos')
    xmin,xmax,ymin,ymax=map(float,bounds)
    if not (xmin<xmax and ymin<ymax):
        raise ValueError('bounds degenerados')
    rects=tuple(rectangles)
    if not rects:
        raise ValueError('Se requieren rectángulos estáticos explícitos')
    if circles_by_robot is not None and len(circles_by_robot)!=p.shape[0]:
        raise ValueError('circles_by_robot requiere un conjunto por robot')
    return p,rects,(xmin,xmax,ymin,ymax)


def corridors(points, circles=(), *, rectangles, bounds=(-15.,15.,-25.,25.),
              static_clearance_m=RECTANGLE_CENTER_CLEARANCE_M, wall_clearance_m=WALL_CENTER_CLEARANCE_M,
              robot_radius_m=INTERROBOT_HALFSEPARATOR_M,
              circles_by_robot=None, operational_margin_m=0.0):
    """Construye semiplanos compatibles con el solver SLSQP sin tocarlo.

    - `rectangles`: estanterías con clearance TOTAL centro-rectángulo 0.60 m
      (=radio Summit 0.55 + distancia libre 0.05), sin usar graph.clearance_m.
    - `circles`: obstáculos circulares comunes (centro, exclusión total).
    - `circles_by_robot[i]`: obstáculos extra privados de robot i; permite
      excepciones de aproximación de basura propia sin retirar las ajenas.
    - Para cada intervalo se aplica la misma separación de segmentos de
      robots que usaba el corridor builder de SwarmBots.
    - Las esperas y las metas tempranas siguen siendo puntos estáticos en
      cada intervalo: se incorporan a los separadores interrobot.
    """
    margin=float(operational_margin_m)
    p, rects, (xmin,xmax,ymin,ymax)=_check_shapes(
        points,circles,rectangles,bounds,circles_by_robot,margin)
    if min(static_clearance_m,wall_clearance_m,robot_radius_m)<0:
        raise ValueError('Las distancias de exclusión deben ser no negativas')
    if not all(np.isfinite(x) for x in (static_clearance_m,wall_clearance_m,robot_radius_m)):
        raise ValueError('Las distancias de exclusión deben ser finitas')
    circles=tuple(circles)  # materializar también si el llamador entregó un generador
    n,steps,_=p.shape
    K=steps-1
    hs=[[[] for _ in range(K)] for _ in range(n)]
    for k in range(K):
        for i in range(n):
            seg=p[i,k:k+2]
            out=hs[i][k]
            c=float(wall_clearance_m)+margin
            out.extend([(np.array([1.,0.]),xmax-c),
                        (np.array([-1.,0.]),-xmin-c),
                        (np.array([0.,1.]),ymax-c),
                        (np.array([0.,-1.]),-ymin-c)])
            for rect in rects:
                normal,limit,_=rectangle_halfspace(seg,rect,float(static_clearance_m)+margin)
                out.append((normal,limit))
            objects=list(circles)
            if circles_by_robot is not None:
                objects.extend(circles_by_robot[i])
            for center,exclusion in objects:
                out.append(disk_halfspace(seg,center,float(exclusion)+margin))
            # Verificar que ambos extremos CBS están dentro del corredor,
            # también para bordes/obstáculos circulares.
            for normal,limit in out:
                if np.max(seg @ normal)-limit>_DEFAULT_EPS:
                    raise ValueError(f'R{i} k={k}: segmento CBS fuera de un semiplano')
        # Par de segmentos simultáneos: fórmula ORIGINAL de SwarmBots.
        for i in range(n):
            for j in range(i+1,n):
                pa,pb=closest_segments(p[i,k:k+2],p[j,k:k+2])
                diff=pb-pa;distance=float(np.linalg.norm(diff))
                R=float(robot_radius_m)
                if distance<=2*R+margin+1e-7:
                    raise ValueError(f'R{i}-R{j} k={k}: CBS no admite separador '
                                     f'para segmentos completos (d={distance:.6f} m; '
                                     f'mínimo={2*R+margin:.6f} m)')
                normal=diff/distance;midpoint=(pa+pb)/2
                hs[i][k].append((normal,float(normal@midpoint-R-margin/2)))
                hs[j][k].append((-normal,float(-normal@midpoint-R-margin/2)))
                for q in (i,j):
                    if any(max(float(nrm @ p[q,k]),float(nrm @ p[q,k+1]))-b > _DEFAULT_EPS
                           for nrm,b in hs[q][k]):
                        raise ValueError(f'R{q} k={k}: segmento no cabe en separador interrobot')
    return hs


def check_segments_in_corridors(points, halfspaces, *, tol=1e-7):
    """Comprobación independiente de que ambos extremos CBS caben en cada poliedro."""
    pts=np.asarray(points,dtype=float)
    violations=[]
    for i in range(len(pts)):
        for k in range(len(pts[i])-1):
            for n,b in halfspaces[i][k]:
                excess=float(np.max(pts[i,k:k+2]@n)-b)
                if excess>tol:
                    violations.append((i,k,excess))
    return violations
