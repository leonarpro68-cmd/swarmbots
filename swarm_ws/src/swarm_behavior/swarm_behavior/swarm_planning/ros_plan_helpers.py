"""Utilidades puras para tomar una instantánea de odometría y alinear inicios."""
from __future__ import annotations
import math
import time
from dataclasses import dataclass
from . import geometry

AGENTS = ('summit0', 'summit1', 'summit2')

@dataclass(frozen=True)
class RobotSample:
    x: float
    y: float
    received_at: float
    frame_id: str = ''


def snapshot(samples: dict[str, RobotSample], max_age: float, now: float | None = None):
    """Devuelve una copia consistente o None. Usa reloj monotónico, no /clock."""
    now = time.monotonic() if now is None else now
    if max_age <= 0:
        raise ValueError('max_age debe ser positivo')
    if any(a not in samples or not (0 <= now - samples[a].received_at <= max_age)
           for a in AGENTS):
        return None
    return tuple(samples[a] for a in AGENTS)


def select_alignment(graph, xy, *, radius=2.5):
    """Selecciona un vértice libre alcanzable mediante segmento libre.

    Respeta tanto columnas estáticas como basuras pendientes.
    """
    if radius <= 0 or not graph.free_position(xy):
        raise ValueError(f'Inicio fuera de zona libre: {xy}')
    for v in sorted(graph.vertices, key=lambda v: (math.dist(xy, geometry.node_position(v)), v)):
        to = geometry.node_position(v)
        if math.dist(xy, to) > radius:
            break
        if graph.segment_clear(xy, to):
            return v
    raise ValueError(f'No se encontró vértice seguro para {xy}')


def planning_starts(graph, samples):
    starts = tuple(select_alignment(graph, (s.x, s.y)) for s in samples)
    if len(set(starts)) != len(starts):
        raise ValueError('Dos robots quedaron alineados al mismo vértice')
    # Verifica también separación física conservadora al tomar el estado inicial.
    min_separation = 2 * geometry.CLEARANCE_WALL_M
    for i in range(len(samples)):
        for j in range(i + 1, len(samples)):
            if math.dist((samples[i].x, samples[i].y), (samples[j].x, samples[j].y)) < min_separation:
                raise ValueError('Robots iniciales demasiado próximos')
    return starts
