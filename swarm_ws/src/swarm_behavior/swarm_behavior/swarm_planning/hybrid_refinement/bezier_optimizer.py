"""Fase 3 (offline): optimización de curvas Bézier quínticas C² por robot.

Toma ``HybridCorridorPlan`` de Fase 2, SIN modificar CBS ni publicar ROS.
Cada punto de control queda dentro del corredor convexo de su intervalo;
por la propiedad de envolvente convexa la curva completa queda dentro.
Se conservan sin movimiento TODOS los ticks CBS etiquetados WAIT.

El objetivo cuadrático es la integral de la aceleración al cuadrado (en
segundos físicos). Cada robot se optimiza independientemente porque los
corredores de Fase 2 ya están separados para cada intervalo compartido.

Si el solucionador falla, se conserva como *candidato offline* una semilla
Bézier C² comprobada; nunca se publica ni ejecuta una solución no validada.
Esta fase NO garantiza seguimiento temporal ni cinodinámica de base mecanum.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np
from scipy import sparse
from scipy.linalg import qr
from scipy.optimize import LinearConstraint, minimize

from .adapter import DiscreteSchedule
from .corridors import HybridCorridorPlan

DEGREE = 5
CONTROL_COUNT = DEGREE + 1


class BezierOptimizationError(RuntimeError):
    """Entrada inconsistente o trayectoria que no supera la validación."""


@dataclass(frozen=True)
class RobotBezier:
    name: str
    # control_points[k][j] = (x,y) ; tiempo inicial = k*dt
    control_points: tuple[tuple[tuple[float, float], ...], ...]
    objective_acceleration_sq: float
    seed_objective_acceleration_sq: float
    solver_success: bool
    solver_message: str
    max_equality_error: float
    max_corridor_violation: float

    def to_dict(self) -> dict:
        return dict(robot=self.name,
                    control_points_m=[[list(p) for p in seg] for seg in self.control_points],
                    objective_acceleration_sq=self.objective_acceleration_sq,
                    seed_objective_acceleration_sq=self.seed_objective_acceleration_sq,
                    solver_success=self.solver_success,
                    solver_message=self.solver_message,
                    max_equality_error=self.max_equality_error,
                    max_corridor_violation=self.max_corridor_violation)


@dataclass(frozen=True)
class BezierTrajectoryPlan:
    robot_names: tuple[str, ...]
    dt_s: float
    initial_duration_s: float
    subdivision_factor: int
    original_ticks: int
    robots: tuple[RobotBezier, ...]
    # NON-executable: temporal constraints still not certified for ROS.

    def to_dict(self) -> dict:
        return dict(type='swarmbots_hybrid_bezier_phase3_v1',frame_id='map',
                    status='offline_bezier_not_ros_executable',degree=DEGREE,
                    continuity='C2',robot_names=list(self.robot_names),
                    dt_s=self.dt_s,original_ticks=self.original_ticks,
                    subdivisions_per_tick=self.subdivision_factor,
                    initial_duration_s=self.initial_duration_s,
                    safety_model='Fase 2: corredores convexos en tiempo común, espera fija',
                    robots=[r.to_dict() for r in self.robots])


def _bernstein_gram_3() -> np.ndarray:
    """Matriz de Gram de B_{i,3} en [0,1], sin dependencia scipy.special."""
    return np.array([
        [math.comb(3, i)*math.comb(3, j)*
         math.factorial(i+j)*math.factorial(6-i-j)/math.factorial(7)
         for j in range(4)] for i in range(4)], dtype=float)


def _single_axis_gram(dt: float) -> np.ndarray:
    # d²/dt² = 20/dt² * suma Delta²(P_j) B_{j,3}(u), dt = dt*du
    d2=np.zeros((4,6),dtype=float)
    for j in range(4):
        d2[j,j:j+3]=(1.,-2.,1.)
    return (400./dt**3)*(d2.T @ _bernstein_gram_3() @ d2)


def _hessian(n: int, dt: float) -> sparse.csc_matrix:
    gram=_single_axis_gram(dt)
    per_segment=np.kron(gram, np.eye(2))
    # objective = x^T Q x. scipy minimize uses 0.5 x^T P x
    return sparse.kron(sparse.eye(n, format='csc'),
                       sparse.csc_matrix(2*per_segment),format='csc')


def _seed(corridor_plan: HybridCorridorPlan, index: int) -> np.ndarray:
    pieces=[]
    for row in corridor_plan.corridors:
        a,b=row[index].start,row[index].end
        # [a,a,a,b,b,b] es la smoothstep quíntica; C² entre piezas con
        # velocidad/aceleración nulas y contenida en segmento original.
        pieces.append([a,a,a,b,b,b])
    return np.asarray(pieces,dtype=float).reshape(-1)


def _idx(k: int, cp: int, coord: int) -> int:
    return 12*k + 2*cp + coord


def _linear_constraints(schedule: DiscreteSchedule, plan: HybridCorridorPlan,
                        index: int, anchor_ticks: bool):
    n=plan.substeps
    dim=12*n
    eqrows=[]; eqvals=[]
    def equal(terms, target):
        row={}
        for k,j,d,c in terms:
            ii=_idx(k,j,d)
            row[ii]=row.get(ii,0.)+float(c)
        eqrows.append(row);eqvals.append(float(target))

    for d in range(2):
        equal(((0,0,d,1),),plan.corridors[0][index].start[d])
        equal(((n-1,5,d,1),),plan.corridors[-1][index].end[d])
        # reposo en ambos extremos para casar con las operaciones de la misión
        equal(((0,1,d,1),(0,0,d,-1)),0)
        equal(((0,2,d,1),(0,1,d,-2),(0,0,d,1)),0)
        equal(((n-1,5,d,1),(n-1,4,d,-1)),0)
        equal(((n-1,5,d,1),(n-1,4,d,-2),(n-1,3,d,1)),0)
        for k in range(n-1):
            # Uniform dt; continuidad geométrica, velocidad y aceleración físicas
            equal(((k,5,d,1),(k+1,0,d,-1)),0)
            equal(((k,5,d,1),(k,4,d,-1),
                   (k+1,1,d,-1),(k+1,0,d,1)),0)
            equal(((k,5,d,1),(k,4,d,-2),(k,3,d,1),
                   (k+1,2,d,-1),(k+1,1,d,2),(k+1,0,d,-1)),0)
        if anchor_ticks:
            for tick in range(1,schedule.ticks):
                end_of_tick=tick*plan.subdivisions_per_tick-1
                equal(((end_of_tick,5,d,1),),schedule.metric_paths[index][tick][d])

    # WAIT del CBS: todas las posiciones de control son exactamente el waypoint.
    for k,row in enumerate(plan.corridors):
        if row[index].wait:
            waypoint=schedule.metric_paths[index][row[index].original_tick]
            for d in range(2):
                for j in range(6):
                    equal(((k,j,d,1),),waypoint[d])

    # Solo caras del polígono: ya representan intersección de TODOS los
    # semiplanos originales (paredes, obstáculos y robot/robot).
    ineqrows=[]; upper=[]
    for k,row in enumerate(plan.corridors):
        polygon=np.asarray(row[index].polygon,dtype=float)
        center=polygon.mean(axis=0)
        for a,b in zip(polygon,np.roll(polygon,-1,axis=0)):
            tangent=b-a
            length=float(np.linalg.norm(tangent))
            if length<1e-10:
                continue
            normal=np.array([tangent[1],-tangent[0]])/length
            bound=float(normal@a)
            if float(normal@center)>bound:
                normal=-normal;bound=-bound
            for j in range(6):
                ineqrows.append({_idx(k,j,0):float(normal[0]),
                                 _idx(k,j,1):float(normal[1])})
                upper.append(bound)

    def make_matrix(rows):
        rr=[];cc=[];vv=[]
        for ii,row in enumerate(rows):
            for col,val in row.items():
                if val:
                    rr.append(ii);cc.append(col);vv.append(val)
        return sparse.csr_matrix((vv,(rr,cc)),shape=(len(rows),dim),dtype=float)
    return make_matrix(eqrows),np.asarray(eqvals),make_matrix(ineqrows),np.asarray(upper)


def _feasibility(x,A,b,C,u):
    eq=float(np.max(np.abs(A@x-b))) if len(b) else 0.
    ineq=float(max(0.,np.max(C@x-u))) if len(u) else 0.
    return eq,ineq


def _validate_inputs(schedule,plan):
    if not isinstance(schedule,DiscreteSchedule) or not isinstance(plan,HybridCorridorPlan):
        raise BezierOptimizationError('Se requiere el adaptador Fase 1 y corredores Fase 2')
    if plan.robot_names!=schedule.robot_names or plan.original_ticks!=schedule.ticks:
        raise BezierOptimizationError('Rutas o nombres incompatibles entre Fase 1 y Fase 2')
    if abs(plan.tick_seconds-schedule.tick_seconds)>1e-8 or len(plan.corridors)!=schedule.ticks*plan.subdivisions_per_tick:
        raise BezierOptimizationError('Calendario de corredores incompatible con el CBS')
    if schedule.ticks==0:
        raise BezierOptimizationError('Plan sin movimiento: no hay segmentos para optimizar')
    if not math.isfinite(plan.tick_seconds) or plan.tick_seconds<=0 or plan.subdivisions_per_tick<=0:
        raise BezierOptimizationError('Tiempos inválidos')
    for k,row in enumerate(plan.corridors):
        if len(row)!=len(schedule.robot_names):
            raise BezierOptimizationError('Número incorrecto de corredores')
        for i,c in enumerate(row):
            if c.robot_name!=schedule.robot_names[i] or c.substep!=k:
                raise BezierOptimizationError('Corredores desordenados')
            if not c.contains(c.start) or not c.contains(c.end):
                raise BezierOptimizationError('Segmento fuera del corredor')
            if abs(c.t1_s-c.t0_s-plan.tick_seconds/plan.subdivisions_per_tick)>1e-7:
                raise BezierOptimizationError('Intervalos de distinta duración')


def optimize_bezier(
    schedule: DiscreteSchedule, plan: HybridCorridorPlan, *,
    anchor_ticks: bool = True, maxiter: int = 300, ftol: float = 1e-7,
    feasibility_tolerance_m: float = 1e-7,
    initial_trajectory: BezierTrajectoryPlan | None = None,
) -> BezierTrajectoryPlan:
    """Resuelve min integral ||a||² con QP linealmente restringido por robot.

    SLSQP minimiza el objetivo convexo; se comprueba la factibilidad
    por separado y se usa la semilla factible si el resultado del solver
    incumple restricciones o es peor. No se reclama optimalidad global si
    ``solver_success=False``. Los puntos de control siguen siendo offline.
    """
    _validate_inputs(schedule,plan)
    if maxiter<1 or not math.isfinite(ftol) or ftol<=0:
        raise BezierOptimizationError('Parámetros del solucionador inválidos')
    dt=plan.tick_seconds/plan.subdivisions_per_tick
    H=_hessian(plan.substeps,dt)
    if initial_trajectory is not None:
        if (initial_trajectory.robot_names!=schedule.robot_names or
            initial_trajectory.original_ticks!=schedule.ticks or
            initial_trajectory.subdivision_factor!=plan.subdivisions_per_tick or
            abs(initial_trajectory.dt_s-dt)>1e-9):
            raise BezierOptimizationError('Warm start de trayectoria incompatible')
    records=[]
    for i,name in enumerate(schedule.robot_names):
        A,b,C,u=_linear_constraints(schedule,plan,i,anchor_ticks)
        seed=_seed(plan,i)
        # En refinamiento la nueva partición puede no incluir el segmento
        # recto original; el anterior Bézier validado es un mejor warm start.
        # Si sus controles caen fuera del nuevo corredor, la smoothstep entre
        # los extremos NUEVOS sirve como semilla C2 alternativa.
        if initial_trajectory is not None:
            previous=np.asarray(initial_trajectory.robots[i].control_points,dtype=float)
            if previous.shape!=(plan.substeps,6,2) or not np.isfinite(previous).all():
                raise BezierOptimizationError('Warm start inválido')
            peq,pineq=_feasibility(previous.reshape(-1),A,b,C,u)
            if peq<=feasibility_tolerance_m and pineq<=feasibility_tolerance_m:
                seed=previous.reshape(-1)
        eq,ineq=_feasibility(seed,A,b,C,u)
        if eq>1e-7 or ineq>1e-7:
            raise BezierOptimizationError(f'Semilla geométrica no factible de {name}: '
                                          f'eq={eq:.3g} ineq={ineq:.3g}')
        initial_cost=float(seed@(H@seed)/2)
        def fun(x):return float(x@(H@x)/2)
        def jac(x):return np.asarray(H@x)
        # SLSQP usa matrices densas para restricciones; permitir la prueba
        # offline, con políticas de maxiter declaradas y sin tiempo real.
        # SLSQP necesita igualdad de rango fila completo. WAIT consecutivos
        # y anclas pueden producir ecuaciones linealmente redundantes.
        full_A=A.toarray()
        _, R, pivot=qr(full_A.T, mode='economic', pivoting=True, check_finite=False)
        diagonal=np.abs(np.diag(R))
        rank=int(np.sum(diagonal>1e-9*max(1.,diagonal[0] if len(diagonal) else 1.)))
        kept=np.sort(pivot[:rank])
        result=minimize(fun,seed,jac=jac,method='SLSQP',
                        constraints=[LinearConstraint(full_A[kept],b[kept],b[kept]),
                                     LinearConstraint(C.toarray(),-np.inf,u)],
                        options={'maxiter':maxiter,'ftol':ftol,'disp':False})
        sol=np.asarray(result.x)
        err,viol=_feasibility(sol,A,b,C,u)
        cost=fun(sol)
        safe=(np.all(np.isfinite(sol)) and err<=feasibility_tolerance_m
              and viol<=feasibility_tolerance_m and cost<=initial_cost+1e-6)
        if not safe:
            sol=seed;err=eq;viol=ineq;cost=initial_cost
        # Tolerancia estricta: los corredores ya incluyen márgenes geométricos;
        # no permitimos violaciones suficientemente grandes para perderlos.
        if err>feasibility_tolerance_m or viol>feasibility_tolerance_m:
            raise BezierOptimizationError(f'No existe solución certificada para {name}')
        result_ctrl=sol.reshape(plan.substeps,6,2)
        # Verificación adicional sobre las desigualdades ORIGINALES.
        for k,row in enumerate(plan.corridors):
            for xy in result_ctrl[k]:
                if not row[i].contains(tuple(xy),tolerance_m=feasibility_tolerance_m):
                    raise BezierOptimizationError(f'Punto de control fuera de corredor: '
                                                  f'{name}, intervalo {k}')
        records.append(RobotBezier(name,tuple(tuple(tuple(map(float,p)) for p in piece)
                                              for piece in result_ctrl),cost,initial_cost,
                                   bool(result.success and safe),
                                   str(result.message) if safe else 'fallback: semilla C2 factible',
                                   err,viol))
    return BezierTrajectoryPlan(schedule.robot_names,dt,schedule.duration_s,
                                plan.subdivisions_per_tick,plan.original_ticks,tuple(records))


def evaluate_bezier(control_points: Sequence[Sequence[float]], u: float, derivative: int=0,
                    dt_s: float=1.) -> tuple[float,float]:
    """Evalúa posición/velocidad/aceleración en tiempo físico local."""
    if derivative not in (0,1,2) or not 0<=u<=1 or dt_s<=0:
        raise ValueError('derivative=0/1/2, 0<=u<=1, dt_s>0')
    points=np.asarray(control_points,dtype=float)
    if points.shape!=(6,2):raise ValueError('Se requieren seis puntos 2D')
    degree=5
    for order in range(derivative):
        points=(degree-order)*np.diff(points,axis=0)/dt_s
    n=degree-derivative
    coeff=np.array([math.comb(n,j)*(1-u)**(n-j)*u**j for j in range(n+1)])
    return tuple(map(float,coeff@points))

@dataclass(frozen=True)
class TimeScaledPlan:
    """Límites cartesianos conservadores; no incluye límites de curvatura/yaw."""
    trajectory: BezierTrajectoryPlan
    factor: float
    dt_s: float
    duration_s: float
    speed_limit_m_s: float
    acceleration_limit_m_s2: float
    # cotas superiores seguras por propiedad convexa de las derivadas
    speed_bound_m_s: tuple[float,...]
    acceleration_bound_m_s2: tuple[float,...]

    def to_dict(self):
        result=self.trajectory.to_dict()
        result['temporal_scaling']={
            'method':'common_uniform_scaling_using_bezier_derivative_control_polygon',
            'factor':self.factor,'dt_s':self.dt_s,'duration_s':self.duration_s,
            'speed_limit_m_s':self.speed_limit_m_s,
            'acceleration_limit_m_s2':self.acceleration_limit_m_s2,
            'speed_upper_bounds_m_s':list(self.speed_bound_m_s),
            'acceleration_upper_bounds_m_s2':list(self.acceleration_bound_m_s2),
            'warning':'Solo límites cartesianos; no verifica control mecanum ni errores ROS',
        }
        return result


def scale_trajectory_time(trajectory: BezierTrajectoryPlan, *,
                          max_speed_m_s: float=.60,
                          max_acceleration_m_s2: float=.50,
                          safety_factor: float=1.001) -> TimeScaledPlan:
    """Escala todos los robots por igual usando cotas analíticas CONSERVADORAS.

    Las derivadas Bézier también son curvas Bézier. Su velocidad máxima es
    <= max norma de los cinco puntos de control de la primera derivada; el
    mismo argumento vale para la segunda derivada. Esto evita depender de
    un muestreo que podría pasar por alto picos.
    """
    for name,val in (('max_speed_m_s',max_speed_m_s),
                     ('max_acceleration_m_s2',max_acceleration_m_s2),
                     ('safety_factor',safety_factor)):
        if not math.isfinite(val) or val<=0:
            raise BezierOptimizationError(f'{name} debe ser finito y positivo')
    if safety_factor < 1:
        raise BezierOptimizationError('safety_factor debe ser >= 1')
    if not trajectory.robots:
        raise BezierOptimizationError('No hay robots que escalar')
    old_dt=trajectory.dt_s
    bounds=[]
    for robot in trajectory.robots:
        cp=np.asarray(robot.control_points)
        first=5*np.diff(cp,axis=1)/old_dt
        second=4*np.diff(first,axis=1)/old_dt
        speed=float(np.max(np.linalg.norm(first,axis=2)))
        accel=float(np.max(np.linalg.norm(second,axis=2)))
        bounds.append((speed,accel))
    factor=max(1.,max(s/max_speed_m_s for s,a in bounds),
               max(math.sqrt(a/max_acceleration_m_s2) for s,a in bounds))
    factor*=safety_factor
    return TimeScaledPlan(trajectory,factor,old_dt*factor,
                          trajectory.initial_duration_s*factor,
                          max_speed_m_s,max_acceleration_m_s2,
                          tuple(s/factor for s,a in bounds),
                          tuple(a/factor**2 for s,a in bounds))
