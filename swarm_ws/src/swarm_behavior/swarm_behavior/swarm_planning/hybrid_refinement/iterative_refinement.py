"""Fase 4 (offline): partición híbrida iterativa a partir de curvas Bézier.

Basado en VII-D de Hönig et al.: muestras temporales por pieza, envolventes
convexas, nuevos semiplanos robot-robot / robot-obstáculo, QP repetido.

Diferencias deliberadas respecto al experimento original:
* WAIT del CBS obligatoriamente estacionario y anclas CBS por defecto.
* Cada propuesta tiene que superar comprobaciones analíticas de contención
  por TODOS los puntos de control; las muestras solo construyen planos.
* No se acepta un empeoramiento del objetivo ni un resultado no certificado.
* Ante fallo se conserva la última solución y corredor válidos; no se mueve ROS.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .adapter import DiscreteSchedule
from .bezier_optimizer import (
    BezierOptimizationError, BezierTrajectoryPlan, evaluate_bezier,
    optimize_bezier, scale_trajectory_time,
)
from .corridors import (
    CircleObstacle, CorridorError, HalfSpace, HybridCorridorPlan, SafeCorridor,
    _add, _area, _boundary_halfspaces, _clip, _closest_segments, _dot,
    _nearest_on_segment, _scale, _sub,
)

Point = tuple[float, float]


class RefinementError(RuntimeError):
    """Problema al validar o inicializar un refinamiento."""


@dataclass(frozen=True)
class RefinementIteration:
    index: int
    objective: float
    objectives_per_robot: tuple[float, ...]
    scaled_duration_s: float
    time_scale: float
    solver_success: tuple[bool, ...]
    min_corridor_area_m2: float

    def to_dict(self) -> dict:
        return dict(iteration=self.index, objective_acceleration_sq=self.objective,
                    objective_per_robot=list(self.objectives_per_robot),
                    scaled_duration_s=self.scaled_duration_s,
                    temporal_factor=self.time_scale,
                    solver_success=list(self.solver_success),
                    min_corridor_area_m2=self.min_corridor_area_m2)


@dataclass(frozen=True)
class IterativeRefinementResult:
    schedule: DiscreteSchedule
    initial_trajectory: BezierTrajectoryPlan
    final_trajectory: BezierTrajectoryPlan
    final_corridors: HybridCorridorPlan
    history: tuple[RefinementIteration, ...]
    reason: str
    samples_per_piece: int

    @property
    def iterations_completed(self) -> int:
        return len(self.history) - 1

    def to_dict(self) -> dict:
        return dict(type='swarmbots_hybrid_iterative_refinement_phase4_v1',
                    status='offline_validated_not_ros_executable',
                    method='sampled_curve_convex_hull_hybrid_separators',
                    samples_per_piece=self.samples_per_piece,
                    iterations_completed=self.iterations_completed,
                    stop_reason=self.reason,
                    history=[h.to_dict() for h in self.history],
                    trajectory=self.final_trajectory.to_dict(),
                    # Los tiempos escalados están en history[-1].
                    warning='Certificación geométrica ideal sin errores de seguimiento/ROS')


def _cross(a: Point, b: Point, c: Point) -> float:
    return ((b[0]-a[0])*(c[1]-a[1]) -
            (b[1]-a[1])*(c[0]-a[0]))


def _hull(points: Sequence[Point]) -> tuple[Point, ...]:
    """Convex hull 2D, admite colinealidad/WAIT sin depender de Shapely."""
    pts=sorted(set((float(x),float(y)) for x,y in points))
    if not pts:
        raise RefinementError('No hay puntos para construir el hull')
    if len(pts)<=2:
        return tuple(pts)
    lower=[]
    for p in pts:
        while len(lower)>=2 and _cross(lower[-2],lower[-1],p)<=1e-12:
            lower.pop()
        lower.append(p)
    upper=[]
    for p in reversed(pts):
        while len(upper)>=2 and _cross(upper[-2],upper[-1],p)<=1e-12:
            upper.pop()
        upper.append(p)
    return tuple(lower[:-1]+upper[:-1])


def _edges(poly: Sequence[Point]):
    if len(poly)==1:
        return ((poly[0],poly[0]),)
    if len(poly)==2:
        return ((poly[0],poly[1]),)
    return tuple(zip(poly,(*poly[1:],poly[0])))


def _closest_hulls(a: Sequence[Point],b: Sequence[Point]) -> tuple[Point, Point, float]:
    # Al provenir de curvas validadas dentro de corredores independientes, los
    # conjuntos convexos están separados; detectar aún si dejan de estarlo.
    candidates=(_closest_segments(u,v,x,y) for u,v in _edges(a) for x,y in _edges(b))
    return min(candidates,key=lambda p:p[2])


def _closest_to_center(poly: Sequence[Point], center: Point) -> tuple[Point,float]:
    candidates=(_nearest_on_segment(center,u,v) for u,v in _edges(poly))
    point=min(candidates,key=lambda p:math.dist(p,center))
    return point,math.dist(point,center)


def _curve_hull(control_points: Sequence[Point], samples: int) -> tuple[Point,...]:
    # Los extremos u=0 y u=1 están incluidos. NO incorporamos los puntos de
    # control, como en el experimento de refinamiento de referencia.
    return _hull(tuple(evaluate_bezier(control_points, j/(samples-1))
                       for j in range(samples)))


def _validate_trajectory(schedule: DiscreteSchedule, corridors: HybridCorridorPlan,
                         trajectory: BezierTrajectoryPlan, *,
                         require_anchors: bool = True, tolerance: float = 1e-6) -> None:
    """Certifica la seguridad continua usando la envolvente de controles.

    Los semiplanos de Fase 2 contienen regiones que son mutuamente seguras.
    Validar LOS SEIS controles de TODAS las piezas basta para garantizar
    contención para cualquier t; muestrear posiciones no sería suficiente.
    """
    if (trajectory.robot_names != schedule.robot_names or
        trajectory.robot_names != corridors.robot_names or
        trajectory.original_ticks != schedule.ticks or
        trajectory.subdivision_factor != corridors.subdivisions_per_tick or
        len(trajectory.robots) != len(schedule.robot_names) or
        abs(trajectory.dt_s - corridors.tick_seconds/corridors.subdivisions_per_tick)>1e-9):
        raise RefinementError('Trayectoria, calendario y corredores incompatibles')
    if any(len(r.control_points)!=corridors.substeps for r in trajectory.robots):
        raise RefinementError('Número de piezas Bézier no coincide con corredores')
    for i,robot in enumerate(trajectory.robots):
        for k,control in enumerate(robot.control_points):
            c=corridors.corridors[k][i]
            if len(control)!=6:
                raise RefinementError('Seis puntos de control requeridos')
            for xy in control:
                if not all(math.isfinite(v) for v in xy) or not c.contains(xy,tolerance_m=tolerance):
                    raise RefinementError(f'Control inseguro: robot={i}, pieza={k}')
                if c.wait and math.dist(xy,schedule.metric_paths[i][c.original_tick])>tolerance:
                    raise RefinementError(f'WAIT de CBS alterado: robot={i}, pieza={k}')
            if k:
                previous=robot.control_points[k-1]
                for derivative in (0,1,2):
                    u=evaluate_bezier(previous,1,derivative,trajectory.dt_s)
                    v=evaluate_bezier(control,0,derivative,trajectory.dt_s)
                    if math.dist(u,v)>max(1e-5,tolerance*10):
                        raise RefinementError(f'Discontinuidad C2: robot={i}, pieza={k}')
        if math.dist(robot.control_points[0][0],schedule.metric_paths[i][0])>tolerance:
            raise RefinementError(f'Inicio incorrecto robot={i}')
        if math.dist(robot.control_points[-1][-1],schedule.metric_paths[i][-1])>tolerance:
            raise RefinementError(f'Meta incorrecta robot={i}')
        if require_anchors:
            f=corridors.subdivisions_per_tick
            for tick in range(1,schedule.ticks):
                xy=robot.control_points[tick*f-1][-1]
                if math.dist(xy,schedule.metric_paths[i][tick])>tolerance:
                    raise RefinementError(f'Waypoint CBS no preservado robot={i}, tick={tick}')


def rebuild_corridors_from_bezier(
    schedule: DiscreteSchedule, old: HybridCorridorPlan,
    trajectory: BezierTrajectoryPlan, *, samples_per_piece: int=32,
    area_epsilon_m2: float=1e-7,
) -> HybridCorridorPlan:
    """Reconstruye todos los semiplanos desde hulls de muestras de curva.

    Si los hulls se tocan o el espacio convexo degenera, rechaza esa partición:
    la versión anterior sigue disponible para el llamador.
    """
    if samples_per_piece < 3 or samples_per_piece > 10000:
        raise RefinementError('samples_per_piece debe estar entre 3 y 10000')
    if not math.isfinite(area_epsilon_m2) or area_epsilon_m2 <= 0:
        raise RefinementError('area_epsilon_m2 inválido')
    _validate_trajectory(schedule,old,trajectory,require_anchors=False)
    walls=_boundary_halfspaces()
    from .. import geometry as geom
    world=((geom.X_MIN,geom.Y_MIN),(geom.X_MAX,geom.Y_MIN),
           (geom.X_MAX,geom.Y_MAX),(geom.X_MIN,geom.Y_MAX))
    rows=[]
    sep=old.min_separation_m
    for k,old_row in enumerate(old.corridors):
        hulls=[_curve_hull(robot.control_points[k],samples_per_piece)
               for robot in trajectory.robots]
        hs=[list(walls) for _ in old.robot_names]
        for i in range(len(hulls)):
            for j in range(i+1,len(hulls)):
                p,q,dist=_closest_hulls(hulls[i],hulls[j])
                if dist < sep-2e-6:
                    raise RefinementError(f'Hulls de robots sin separación suficiente: '
                                          f'pieza={k}, par=({i},{j}), d={dist:.6f}')
                if dist<=1e-10:
                    raise RefinementError('Hulls en contacto; no existe bisectriz')
                n=_scale(_sub(q,p),1/dist)
                mid=_scale(_add(p,q),.5)
                hs[i].append(HalfSpace(n,_dot(n,mid)-sep/2,old.robot_names[j],
                                       'robot_max_margin_refined'))
                neg=_scale(n,-1.)
                hs[j].append(HalfSpace(neg,_dot(neg,mid)-sep/2,old.robot_names[i],
                                       'robot_max_margin_refined'))
        for i,hull in enumerate(hulls):
            for obstacle in old.obstacles:
                p,dist=_closest_to_center(hull,obstacle.center)
                if dist < obstacle.exclusion_radius_m-2e-6:
                    raise RefinementError(f'Hull cerca de obstáculo: pieza={k}, '
                                          f'robot={i}, obstáculo={obstacle.name}')
                if dist<=1e-10:
                    raise RefinementError('Centro de obstáculo coincidente con curva')
                n=_scale(_sub(obstacle.center,p),1/dist)
                hs[i].append(HalfSpace(n,_dot(n,obstacle.center)-obstacle.exclusion_radius_m,
                                       obstacle.name,'obstacle_tangent_refined'))
        new_row=[]
        for i,old_corridor in enumerate(old_row):
            poly=world
            for h in hs[i]:
                poly=_clip(poly,h)
                if not poly:
                    raise RefinementError(f'Corredor vacío, robot={i}, pieza={k}')
            area=_area(poly)
            if len(poly)<3 or area<area_epsilon_m2:
                raise RefinementError(f'Corredor degenerado, robot={i}, pieza={k}')
            # Esquinas de la curva previa son los nuevos endpoints de
            # referencia, no se exige pasar por nodos intermedios subdivididos.
            a=evaluate_bezier(trajectory.robots[i].control_points[k],0)
            b=evaluate_bezier(trajectory.robots[i].control_points[k],1)
            if not all(h.slack(p)>=-3e-6 for p in (a,b,*hulls[i]) for h in hs[i]):
                raise RefinementError(f'Hull no contenido, robot={i}, pieza={k}')
            new_row.append(SafeCorridor(
                robot_name=old_corridor.robot_name,substep=k,
                original_tick=old_corridor.original_tick,
                t0_s=old_corridor.t0_s,t1_s=old_corridor.t1_s,
                start=a,end=b,wait=old_corridor.wait,
                halfspaces=tuple(hs[i]),polygon=poly,area_m2=area,
            ))
        rows.append(tuple(new_row))
    return HybridCorridorPlan(old.robot_names,old.original_ticks,
                              old.subdivisions_per_tick,old.tick_seconds,
                              sep,old.obstacles,tuple(rows))


def _cost(trajectory: BezierTrajectoryPlan) -> float:
    return math.fsum(robot.objective_acceleration_sq for robot in trajectory.robots)


def _snapshot(index: int, traj: BezierTrajectoryPlan, corridors: HybridCorridorPlan,
              max_speed_m_s: float, max_acceleration_m_s2: float) -> RefinementIteration:
    scaled=scale_trajectory_time(traj,max_speed_m_s=max_speed_m_s,
                                 max_acceleration_m_s2=max_acceleration_m_s2)
    return RefinementIteration(index,_cost(traj),
                               tuple(r.objective_acceleration_sq for r in traj.robots),
                               scaled.duration_s,scaled.factor,
                               tuple(r.solver_success for r in traj.robots),
                               min(c.area_m2 for row in corridors.corridors for c in row))


def refine_iteratively(
    schedule: DiscreteSchedule, initial_corridors: HybridCorridorPlan, *,
    initial_trajectory: BezierTrajectoryPlan | None=None,
    iterations: int=5, samples_per_piece: int=32,
    improvement_tolerance: float=1e-6,
    maxiter: int=180, anchor_ticks: bool=True,
    max_speed_m_s: float=.60, max_acceleration_m_s2: float=.50,
) -> IterativeRefinementResult:
    """Refina en bucle y conserva siempre la última solución validada.

    A diferencia del experimento con dos robots, la monotonía del costo es
    una regla de aceptación: soluciones peores/no certificadas no se aceptan.
    El bucle es offline y NO proporciona garantías sobre seguimiento físico.
    """
    if iterations<0 or iterations>100:
        raise RefinementError('iterations debe estar entre 0 y 100')
    if not math.isfinite(improvement_tolerance) or improvement_tolerance<0:
        raise RefinementError('improvement_tolerance inválida')
    if initial_trajectory is None:
        initial_trajectory=optimize_bezier(schedule,initial_corridors,
                                            anchor_ticks=anchor_ticks,maxiter=maxiter)
    _validate_trajectory(schedule,initial_corridors,initial_trajectory,
                         require_anchors=anchor_ticks)
    current=initial_trajectory
    corridors=initial_corridors
    history=[_snapshot(0,current,corridors,max_speed_m_s,max_acceleration_m_s2)]
    reason='iteration_limit'
    for number in range(1,iterations+1):
        try:
            proposed=rebuild_corridors_from_bezier(schedule,corridors,current,
                                                    samples_per_piece=samples_per_piece)
            candidate=optimize_bezier(schedule,proposed,anchor_ticks=anchor_ticks,
                                      maxiter=maxiter,initial_trajectory=current)
            _validate_trajectory(schedule,proposed,candidate,require_anchors=anchor_ticks)
            new_cost=_cost(candidate)
            old_cost=_cost(current)
            if not math.isfinite(new_cost) or new_cost>old_cost+1e-6:
                reason='candidate_not_improving'
                break
            # Si el cambio es demasiado pequeño se conserva el plan ANTERIOR
            # (mejor diagnosticable; no se cambia versión de corredores).
            if old_cost-new_cost <= improvement_tolerance:
                reason='converged'
                break
            # No sustituir solución por una iteración numéricamente fallida:
            # un solver puede devolver una solución geométrica factible que
            # todavía no cumpla el criterio de éxito de convergencia.
            if not all(r.solver_success for r in candidate.robots):
                reason='solver_not_converged'
                break
            corridors=proposed
            current=candidate
            history.append(_snapshot(number,current,corridors,
                                     max_speed_m_s,max_acceleration_m_s2))
        except (CorridorError,BezierOptimizationError,RefinementError,
                ArithmeticError,ValueError) as exc:
            reason=f'rejected: {type(exc).__name__}: {exc}'
            break
    return IterativeRefinementResult(schedule,initial_trajectory,current,corridors,
                                     tuple(history),reason,samples_per_piece)
