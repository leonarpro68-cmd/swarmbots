"""Adaptador de rutas CBS a una programación métrica y temporal común.

Entrada: caminos discretos *reales* obtenidos de ``cbs.solve`` o
``PlanningBridge.plan_pickup/plan_delivery``.

Salida: series métricas en el marco ``map`` (geometría del warehouse) y un
calendario NOMINAL compartido. NO se asume que cada tick del CBS dure 1 s;
no se generan curvas ni se publican mensajes ROS.

El tiempo nominal obtenido de ``max_speed_m_s`` solo acota la velocidad
media en aristas rectas. Aceleraciones y errores de seguimiento aún deben
validarse y pueden requerir aumentar globalmente todos los tiempos.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .. import cbs, geometry

Node = geometry.Node
Point = geometry.Point


class CBSAdapterError(ValueError):
    """La entrada no preserva las garantías del plan discreto original."""


@dataclass(frozen=True)
class TimedStep:
    """Movimiento simultáneo de TODOS los robots en [t0, t1]."""
    index: int
    t0: float
    t1: float
    starts: tuple[Point, ...]
    ends: tuple[Point, ...]
    waits: tuple[bool, ...]


@dataclass(frozen=True)
class DiscreteSchedule:
    """Salida del CBS alineada por ticks, en metros y en segundos nominales."""
    robot_names: tuple[str, ...]
    grid_paths: tuple[tuple[Node, ...], ...]
    metric_paths: tuple[tuple[Point, ...], ...]
    tick_seconds: float
    min_separation_m: float
    max_step_m: float
    max_speed_m_s: float
    was_padded: tuple[int, ...]

    @property
    def ticks(self) -> int:
        return len(self.grid_paths[0]) - 1

    @property
    def duration_s(self) -> float:
        return self.ticks * self.tick_seconds

    @property
    def times_s(self) -> tuple[float, ...]:
        return tuple(k * self.tick_seconds for k in range(self.ticks + 1))

    @property
    def steps(self) -> tuple[TimedStep, ...]:
        return tuple(
            TimedStep(
                index=k, t0=k * self.tick_seconds, t1=(k + 1) * self.tick_seconds,
                starts=tuple(p[k] for p in self.metric_paths),
                ends=tuple(p[k + 1] for p in self.metric_paths),
                waits=tuple(p[k] == p[k + 1] for p in self.grid_paths),
            ) for k in range(self.ticks)
        )

    def to_dict(self) -> dict:
        """JSON serializable para pruebas offline; no es un mensaje ROS."""
        return {
            'type': 'cbs_discrete_nominal_schedule_v1',
            'frame_id': 'map',
            'robots': list(self.robot_names),
            'ticks': self.ticks,
            'tick_seconds': self.tick_seconds,
            'duration_s': self.duration_s,
            'time_semantics': 'nominal_global_time_not_executable_trajectory',
            'grid_step_m': geometry.STEP_M,
            'min_separation_m': self.min_separation_m,
            'max_step_m': self.max_step_m,
            'max_speed_m_s': self.max_speed_m_s,
            'end_waits_appended': list(self.was_padded),
            'times_s': list(self.times_s),
            'grid_paths': [[list(v) for v in p] for p in self.grid_paths],
            'metric_paths_m': [[list(xy) for xy in p] for p in self.metric_paths],
            'wait_ticks': [[k for k in range(self.ticks) if p[k] == p[k + 1]]
                           for p in self.grid_paths],
        }


def from_cbs_paths(
    paths: Sequence[Sequence[Node]],
    *,
    graph=None,
    robot_names: Sequence[str] | None = None,
    max_speed_m_s: float = 0.60,
    tick_seconds: float | None = None,
    min_separation_m: float = cbs.MIN_SEPARATION,
) -> DiscreteSchedule:
    """Valida y convierte una solución CBS con ``WAIT`` y metas persistentes.

    ``graph`` debería ser el MISMO grafo que se entregó al CBS: en recogida
    y entrega es ``PlanningBridge.graph`` (dinámico). Con ``graph=None`` se
    utiliza un grafo estático nuevo; esto es útil solo para ensayos sintéticos.

    Se rellenan los caminos cortos manteniendo el último vértice, tal como
    hace el ejecutor ``padded_paths``. No se añaden esperas intermedias.

    ``tick_seconds`` puede ser un tiempo propuesto por un planificador externo,
    pero se rechaza si implica sobrepasar ``max_speed_m_s`` en algún tramo.
    No se afirma factibilidad dinámica para los Summit.
    """
    if not paths:
        raise CBSAdapterError('CBS devolvió cero rutas')
    if not (math.isfinite(max_speed_m_s) and max_speed_m_s > 0):
        raise CBSAdapterError('max_speed_m_s debe ser positivo y finito')
    if not (math.isfinite(min_separation_m) and min_separation_m > 0):
        raise CBSAdapterError('min_separation_m debe ser positivo y finito')

    try:
        clean = tuple(tuple(tuple(v) for v in path) for path in paths)
    except (TypeError, ValueError) as exc:
        raise CBSAdapterError('Formato inválido: esperado [robot][tick]=(i,j)') from exc
    if any(not p for p in clean):
        raise CBSAdapterError('No se permiten rutas vacías')
    if any(len(v) != 2 or not all(isinstance(k, int) and not isinstance(k, bool)
                                  for k in v) for p in clean for v in p):
        raise CBSAdapterError('Se esperan índices enteros (i,j) del grid')

    if robot_names is None:
        robot_names = tuple(f'summit{i}' for i in range(len(clean)))
    names = tuple(robot_names)
    if len(names) != len(clean) or len(set(names)) != len(names):
        raise CBSAdapterError('Nombres de robots duplicados o cantidad incorrecta')

    if graph is None:
        graph = cbs.Graph()
    if not hasattr(graph, 'vertices') or not hasattr(graph, 'neighbors'):
        raise CBSAdapterError('graph no cumple la interfaz del CBS')
    for i, p in enumerate(clean):
        for v in p:
            if v not in graph.vertices:
                raise CBSAdapterError(f'{names[i]}: vértice bloqueado o desconocido {v}')
        for a, b in zip(p, p[1:]):
            if a != b and b not in graph.neighbors[a]:
                raise CBSAdapterError(f'{names[i]}: transición no válida {a} -> {b}')
            if a != b and hasattr(graph, 'segment_clear') and not graph.segment_clear(
                    geometry.node_position(a), geometry.node_position(b)):
                raise CBSAdapterError(f'{names[i]}: segmento bloqueado {a} -> {b}')

    longest = max(map(len, clean))
    pads = tuple(longest - len(p) for p in clean)
    aligned = tuple(p + (p[-1],) * n for p, n in zip(clean, pads))
    # first_conflict valida separación geométrica durante cada tick y al
    # mantener las metas ocupadas después de alcanzarlas.
    conflict = cbs.first_conflict(aligned, clearance=min_separation_m)
    if conflict is not None:
        raise CBSAdapterError(
            f'Conflicto CBS: robot {conflict.a} y {conflict.b}, '
            f'tick={conflict.time}, {conflict.kind}, '
            f'distancia={conflict.distance:.4f} m')

    xy = tuple(tuple(geometry.node_position(v) for v in p) for p in aligned)
    biggest = max((math.dist(a, b) for p in xy for a, b in zip(p, p[1:])), default=0.0)
    dt_min = biggest / max_speed_m_s
    if tick_seconds is None:
        # Incluso si todos esperan, se requiere un intervalo temporal positivo.
        dt = max(1.0, dt_min)
    else:
        if not (math.isfinite(tick_seconds) and tick_seconds > 0):
            raise CBSAdapterError('tick_seconds debe ser positivo y finito')
        dt = float(tick_seconds)
        if dt < dt_min - 1e-9:
            raise CBSAdapterError(
                f'Tick de {dt:.4f} s demasiado corto: el mínimo geométrico '
                f'a {max_speed_m_s:.3f} m/s es {dt_min:.4f} s')
    return DiscreteSchedule(
        names, aligned, xy, dt, min_separation_m, biggest, max_speed_m_s, pads
    )
