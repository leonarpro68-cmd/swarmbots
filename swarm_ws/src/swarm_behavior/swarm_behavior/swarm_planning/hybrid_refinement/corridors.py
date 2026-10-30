"""Corredores convexos híbridos de SwarmBots (Fase 2, solamente offline).

Se parte de DiscreteSchedule de la Fase 1. No se modifica CBS, no se
interactúa con ROS y NO se promete que exista todavía una trayectoria Bézier.

Por cada subintervalo sincrónico:
- robot/robot: bisectriz de máxima separación de los dos SEGMENTOS completos;
- robot/obstáculo circular: tangente a círculo ya inflado por la huella y
  el margen que usa geometry / DynamicTrashGraph;
- paredes: rectángulo de centros accesibles del mapa.

OBSERVACIÓN: CBS valida posiciones para el MISMO instante (su interpolación
sincronizada). Dos segmentos pueden acercarse o cruzarse espacialmente aunque
ese movimiento sincronizado sea válido. Por eso se subdividen TODOS los ticks
un número de veces común (1, 2, 4, ...) hasta lograr corredores separados.
Si no es posible, la construcción falla explícitamente (NO se relaja seguridad).

La prueba de seguridad de un futuro Bézier se basa en TODOS sus puntos de
control dentro del corredor convexo, no en muestras puntuales de la curva.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from .. import cbs, geometry as geom
from ..dynamic_map import DynamicTrashGraph, DEPOSIT_RADIUS_M, TRASH_EXCLUSION_M
from .adapter import DiscreteSchedule

Point = geom.Point
EPS = 1e-9


class CorridorError(ValueError):
    """No hay una partición convexa válida para las restricciones requeridas."""


@dataclass(frozen=True)
class CircleObstacle:
    name: str
    center: Point
    exclusion_radius_m: float


@dataclass(frozen=True)
class HalfSpace:
    """n·p <= offset_m, con normal unitario (metros)."""
    normal: Point
    offset_m: float
    source: str
    kind: str

    def slack(self, p: Point) -> float:
        return self.offset_m - _dot(self.normal, p)

    def to_dict(self) -> dict:
        return {
            'normal': list(self.normal), 'offset_m': self.offset_m,
            'source': self.source, 'kind': self.kind,
        }


@dataclass(frozen=True)
class SafeCorridor:
    robot_name: str
    substep: int
    original_tick: int
    t0_s: float
    t1_s: float
    start: Point
    end: Point
    wait: bool
    halfspaces: tuple[HalfSpace, ...]
    polygon: tuple[Point, ...]
    area_m2: float

    def contains(self, p: Point, tolerance_m: float = 1e-7) -> bool:
        return all(h.slack(p) >= -tolerance_m for h in self.halfspaces)

    def to_dict(self) -> dict:
        return {
            'robot': self.robot_name,
            'substep': self.substep,
            'original_tick': self.original_tick,
            't0_s': self.t0_s, 't1_s': self.t1_s,
            'start_m': list(self.start), 'end_m': list(self.end),
            'wait': self.wait,
            'area_m2': self.area_m2,
            'vertices_m': [list(p) for p in self.polygon],
            'halfspaces': [h.to_dict() for h in self.halfspaces],
        }


@dataclass(frozen=True)
class HybridCorridorPlan:
    robot_names: tuple[str, ...]
    original_ticks: int
    subdivisions_per_tick: int
    tick_seconds: float
    min_separation_m: float
    obstacles: tuple[CircleObstacle, ...]
    # Indexación: corridors[substep][robot]
    corridors: tuple[tuple[SafeCorridor, ...], ...]

    @property
    def substeps(self) -> int:
        return len(self.corridors)

    @property
    def duration_s(self) -> float:
        return self.original_ticks * self.tick_seconds

    def to_dict(self) -> dict:
        return {
            'type': 'swarmbots_hybrid_safe_corridors_v1',
            'frame_id': 'map',
            'status': 'geometry_only_not_executable',
            'robot_names': list(self.robot_names),
            'original_ticks': self.original_ticks,
            'subdivisions_per_tick': self.subdivisions_per_tick,
            'substeps': self.substeps,
            'tick_seconds': self.tick_seconds,
            'duration_s': self.duration_s,
            'min_separation_m': self.min_separation_m,
            'obstacles': [dict(name=o.name, center_m=list(o.center),
                               exclusion_radius_m=o.exclusion_radius_m)
                          for o in self.obstacles],
            'corridors': [[c.to_dict() for c in row] for row in self.corridors],
        }


def _dot(a: Point, b: Point) -> float:
    return a[0]*b[0] + a[1]*b[1]


def _sub(a: Point, b: Point) -> Point:
    return (a[0]-b[0], a[1]-b[1])


def _add(a: Point, b: Point) -> Point:
    return (a[0]+b[0], a[1]+b[1])


def _scale(a: Point, s: float) -> Point:
    return (a[0]*s, a[1]*s)


def _lerp(a: Point, b: Point, t: float) -> Point:
    return (a[0] + (b[0]-a[0])*t, a[1] + (b[1]-a[1])*t)


def _nearest_on_segment(p: Point, a: Point, b: Point) -> Point:
    d = _sub(b, a)
    den = _dot(d, d)
    t = max(0.0, min(1.0, _dot(_sub(p,a), d)/den)) if den > EPS*EPS else 0.0
    return _lerp(a, b, t)


def _closest_segments(a: Point, b: Point, c: Point, d: Point) -> tuple[Point, Point, float]:
    """Puntos más próximos entre dos segmentos cerrados en R², sin Shapely."""
    v, w = _sub(b,a), _sub(d,c)
    den = v[0]*w[1] - v[1]*w[0]
    if abs(den) > 1e-13:
        delta = _sub(c,a)
        t = (delta[0]*w[1] - delta[1]*w[0])/den
        u = (delta[0]*v[1] - delta[1]*v[0])/den
        if -EPS <= t <= 1+EPS and -EPS <= u <= 1+EPS:
            p = _lerp(a,b,max(0.0,min(1.0,t)))
            return p, p, 0.0
    candidate = []
    for p in (a,b):
        q = _nearest_on_segment(p,c,d)
        candidate.append((math.dist(p,q),p,q))
    for q in (c,d):
        p = _nearest_on_segment(q,a,b)
        candidate.append((math.dist(p,q),p,q))
    dist,p,q = min(candidate, key=lambda x:x[0])
    return p,q,dist


def _clip(poly: tuple[Point, ...], half: HalfSpace) -> tuple[Point, ...]:
    if not poly:
        return ()
    result = []
    for a,b in zip(poly, poly[1:]+poly[:1]):
        da, db = half.slack(a), half.slack(b)
        if da >= -EPS:
            result.append(a)
        if (da >= -EPS) != (db >= -EPS) and abs(da-db)>1e-14:
            alpha = max(0.0,min(1.0,da/(da-db)))
            result.append(_lerp(a,b,alpha))
    # Eliminar vértices adyacentes iguales por razones numéricas.
    clean=[]
    for p in result:
        if not clean or math.dist(clean[-1],p)>1e-8:
            clean.append(p)
    if len(clean)>1 and math.dist(clean[0],clean[-1])<=1e-8:
        clean.pop()
    return tuple(clean)


def _area(poly: tuple[Point, ...]) -> float:
    return abs(sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(poly,poly[1:]+poly[:1])))*0.5


def obstacles_for_graph(graph) -> tuple[CircleObstacle, ...]:
    """Obstáculos modelados por el planificador ACTUAL, no inventados del SDF.

    Radio ya inflado para cada tipo (las basuras pendientes desaparecen
    solo al llamar collect() en la fase adecuada).
    """
    if not isinstance(graph, (cbs.Graph, DynamicTrashGraph)):
        raise CorridorError(
            'Grafo desconocido: no se puede certificar su geometría; '
            'se requiere cbs.Graph o DynamicTrashGraph')
    obstacles=[CircleObstacle(f'columna_{i}', (float(x),float(y)),
                              geom.CLEARANCE_COLUMN_M)
               for i,(x,y) in enumerate(geom.COLUMNS_XY)]
    if isinstance(graph, DynamicTrashGraph):
        obstacles += [CircleObstacle(f'basura_{name}', xy, TRASH_EXCLUSION_M)
                      for name,xy in zip(sorted(graph.pending),graph.active_positions())]
        obstacles += [CircleObstacle(f'deposito_{name}', geom.node_position(dep.node),
                                     DEPOSIT_RADIUS_M)
                      for name,dep in sorted(graph.deposits.items())]
    return tuple(obstacles)


def _boundary_halfspaces() -> tuple[HalfSpace, ...]:
    r = geom.CLEARANCE_WALL_M
    return (
        HalfSpace((1.0, 0.0), geom.X_MAX-r, 'pared_este', 'wall'),
        HalfSpace((-1.0, 0.0), -geom.X_MIN-r, 'pared_oeste', 'wall'),
        HalfSpace((0.0, 1.0), geom.Y_MAX-r, 'pared_norte', 'wall'),
        HalfSpace((0.0, -1.0), -geom.Y_MIN-r, 'pared_sur', 'wall'),
    )


def _one_partition(schedule: DiscreteSchedule, obstacles: tuple[CircleObstacle,...],
                   factor: int, area_epsilon_m2: float) -> HybridCorridorPlan:
    names = schedule.robot_names
    robot_count = len(names)
    steps=[]
    walls=_boundary_halfspaces()
    initial=((geom.X_MIN, geom.Y_MIN), (geom.X_MAX,geom.Y_MIN),
             (geom.X_MAX, geom.Y_MAX), (geom.X_MIN,geom.Y_MAX))
    for k in range(schedule.ticks):
        for part in range(factor):
            s0=part/factor; s1=(part+1)/factor
            segments=[(_lerp(p[k],p[k+1],s0), _lerp(p[k],p[k+1],s1))
                      for p in schedule.metric_paths]
            hs=[list(walls) for _ in range(robot_count)]
            for i in range(robot_count):
                for j in range(i+1, robot_count):
                    p,q,d=_closest_segments(*segments[i],*segments[j])
                    if d < schedule.min_separation_m + 1e-6:
                        raise CorridorError(f'tick={k}, parte={part}/{factor}: '
                                            f'{names[i]} vs {names[j]}: segmentos a '
                                            f'{d:.5f} m, margen exigido '
                                            f'{schedule.min_separation_m:.5f} m')
                    normal=_scale(_sub(q,p),1/d)
                    midpoint=_scale(_add(p,q), 0.5)
                    hs[i].append(HalfSpace(normal, _dot(normal,midpoint)-
                                           schedule.min_separation_m/2,
                                           names[j], 'robot_max_margin'))
                    inv=_scale(normal,-1)
                    hs[j].append(HalfSpace(inv, _dot(inv,midpoint)-
                                           schedule.min_separation_m/2,
                                           names[i], 'robot_max_margin'))
            for i in range(robot_count):
                for obstacle in obstacles:
                    a,b=segments[i]
                    nearest=_nearest_on_segment(obstacle.center,a,b)
                    dist=math.dist(nearest,obstacle.center)
                    if dist < obstacle.exclusion_radius_m+1e-7:
                        raise CorridorError(f'tick={k}, parte={part}/{factor}: '
                                            f'{names[i]} toca el obstáculo {obstacle.name}')
                    # Hiperplano tangente al círculo inflado, mirando hacia el obstáculo.
                    n=_scale(_sub(obstacle.center,nearest),1/dist)
                    hs[i].append(HalfSpace(n, _dot(n,obstacle.center)-
                                           obstacle.exclusion_radius_m,
                                           obstacle.name,'obstacle_tangent'))
            row=[]
            for i,(a,b) in enumerate(segments):
                poly=initial
                for h in hs[i]:
                    poly=_clip(poly,h)
                    if not poly:
                        raise CorridorError(f'tick={k}, parte={part}/{factor}: '
                                            f'corredor vacío de {names[i]}')
                area=_area(poly)
                if len(poly)<3 or area<area_epsilon_m2:
                    raise CorridorError(f'tick={k}, parte={part}/{factor}: '
                                        f'corredor degenerado {names[i]} (área={area:.9f})')
                if any(h.slack(point) < -2e-6 for point in (*poly,a,b) for h in hs[i]):
                    raise CorridorError(f'tick={k}, parte={part}/{factor}: '
                                        f'segmento original no contenido en corredor de {names[i]}')
                row.append(SafeCorridor(
                    robot_name=names[i], substep=k*factor+part, original_tick=k,
                    t0_s=(k+s0)*schedule.tick_seconds,
                    t1_s=(k+s1)*schedule.tick_seconds,
                    start=a,end=b,wait=(schedule.grid_paths[i][k] == schedule.grid_paths[i][k+1]),
                    halfspaces=tuple(hs[i]), polygon=poly,area_m2=area,
                ))
            steps.append(tuple(row))
    return HybridCorridorPlan(names,schedule.ticks,factor,schedule.tick_seconds,
                              schedule.min_separation_m,obstacles,tuple(steps))


def build_hybrid_corridors(
    schedule: DiscreteSchedule, *, graph,
    max_subdivisions: int = 64, min_area_m2: float = 1e-7,
) -> HybridCorridorPlan:
    """Construye todos los corredores a la vez con subdivisión común.

    Si ninguna subdivisión produce un conjunto completo de corredores de
    área positiva, falla explícitamente; nunca reduce radios de seguridad.
    ``graph`` debe ser el MISMO grafo (y estado de basuras) que el del CBS.
    """
    if not isinstance(schedule, DiscreteSchedule):
        raise CorridorError('Se requiere DiscreteSchedule de la Fase 1')
    if max_subdivisions < 1 or max_subdivisions & (max_subdivisions-1):
        raise CorridorError('max_subdivisions debe ser potencia de dos >= 1')
    if not (math.isfinite(min_area_m2) and min_area_m2 > 0):
        raise CorridorError('min_area_m2 debe ser positivo y finito')
    obstacles=obstacles_for_graph(graph)
    if schedule.ticks == 0:
        return HybridCorridorPlan(schedule.robot_names, 0, 1, schedule.tick_seconds,
                                  schedule.min_separation_m, obstacles, ())
    # Una entrada alterada después de adaptar no se considera confiable.
    for i,p in enumerate(schedule.metric_paths):
        for a,b in zip(p,p[1:]):
            is_clear = (graph.segment_clear(a,b) if hasattr(graph, 'segment_clear')
                        else geom.segment_clear(a,b))
            if not is_clear:
                raise CorridorError(f'La geometría actual bloquea la ruta del robot {i}; '
                                    'vuelve a ejecutar CBS y el adaptador')
    if cbs.first_conflict(schedule.grid_paths,clearance=schedule.min_separation_m):
        raise CorridorError('Las rutas CBS tienen un conflicto')
    last=None
    factor=1
    while factor<=max_subdivisions:
        try:
            return _one_partition(schedule,obstacles,factor,min_area_m2)
        except CorridorError as exc:
            last=str(exc)
            factor*=2
    raise CorridorError(f'No existe partición certificable con hasta '
                        f'{max_subdivisions} subintervalos/tick; último fallo: {last}')
