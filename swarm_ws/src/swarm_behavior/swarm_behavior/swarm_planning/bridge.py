"""Fachada pura de planificación para la futura integración ROS 2.

No suscribe odometría, no controla robots y no modifica Gazebo.
Se crea una instancia por misión: evita variables globales de escenario.
"""
from __future__ import annotations
from dataclasses import dataclass
from . import assignment, cbs
from .dynamic_map import DynamicTrashGraph
from .scenario import Scenario, load


@dataclass(frozen=True)
class PickupPlan:
    starts: tuple
    assigned: tuple  # (nombre basura, distancia euclidiana), por robot
    goals: tuple  # opciones de aproximación, por robot
    paths: tuple
    cost: int
    expanded: int
    generated: int


@dataclass(frozen=True)
class DeliveryPlan:
    starts: tuple
    trash_names: tuple
    colors: tuple
    goals: tuple  # hasta ocho vértices accesibles por depósito, por robot
    paths: tuple
    cost: int
    expanded: int
    generated: int


class PlanningBridge:
    def __init__(self, scenario: Scenario | None = None):
        self.scenario = scenario
        self.graph = DynamicTrashGraph(elements=scenario.elements if scenario is not None else None)

    @classmethod
    def from_json(cls, filename):
        return cls(load(filename))

    def plan_pickup(self, starts, *, max_ct=20000, max_time=65):
        starts = tuple(tuple(v) for v in starts)
        if not starts or any(v not in self.graph.vertices for v in starts):
            raise ValueError('Inicios fuera del mapa libre')
        selected = assignment.assign_nearest(starts, self.graph)
        goals = tuple(assignment.approach_vertices(self.graph, name) for name, _ in selected)
        result = assignment.solve_multi(self.graph, starts, goals, max_ct=max_ct, max_time=max_time)
        for path, allowed in zip(result['paths'], goals):
            if path[-1] not in allowed or any(v not in self.graph.vertices for v in path):
                raise RuntimeError('Ruta inconsistente con grafo dinámico')
        if cbs.first_conflict(result['paths']) is not None:
            raise RuntimeError('Conflicto persistente en solución CBS')
        return PickupPlan(starts, selected, goals, result['paths'], result['cost'], result['expanded'], result['generated'])

    def confirm_collected(self, name):
        """Llamar SOLO después de confirmar externamente eliminación en Gazebo."""
        self.graph.collect(name)

    @property
    def pending(self):
        return frozenset(self.graph.pending)

    def deposit_goals(self, color):
        """Ocho vecinos potenciales del depósito; su centro no es una meta.

        Preserva la definición original de cbs_entrega_depositos_ros.py.
        """
        deposits = [d for d in self.graph.deposits.values() if d.color == color]
        if len(deposits) != 1:
            raise ValueError(f'Se esperaba un depósito único de color {color}')
        i, j = deposits[0].node
        return tuple(sorted((i + di, j + dj)
                            for di in (-1, 0, 1) for dj in (-1, 0, 1)
                            if (di or dj) and (i + di, j + dj) in self.graph.vertices))

    def plan_delivery(self, starts, picked_names, *, max_ct=20000, max_time=65):
        """Planifica entregas con CBS usando solo basuras ya confirmadas.

        No actualiza mapa, odometría ni Gazebo. Robots del mismo color pueden
        terminar simultáneamente en distintos vecinos del mismo depósito.
        """
        starts = tuple(tuple(v) for v in starts)
        picked_names = tuple(picked_names)
        if len(starts) != 3 or len(picked_names) != 3:
            raise ValueError('Se requieren tres robots y tres basuras para la entrega')
        if len(set(picked_names)) != len(picked_names):
            raise ValueError('Basura repetida entre robots')
        if any(name not in self.graph.trash or name in self.graph.pending
               for name in picked_names):
            raise ValueError('La entrega exige basuras existentes y recogidas')
        if any(v not in self.graph.vertices for v in starts):
            raise ValueError('Algún inicio de entrega está bloqueado')
        colors = tuple(self.graph.trash[name].color for name in picked_names)
        goals = tuple(self.deposit_goals(color) for color in colors)
        if any(not options for options in goals):
            raise ValueError('Un depósito carece de vecinos accesibles')
        result = assignment.solve_multi(self.graph, starts, goals,
                                        max_ct=max_ct, max_time=max_time)
        paths = result['paths']
        if cbs.first_conflict(paths) is not None:
            raise RuntimeError('CBS devolvió conflictos durante la entrega')
        for i, path in enumerate(paths):
            if path[0] != starts[i] or path[-1] not in goals[i]:
                raise RuntimeError('Inicio o destino de entrega incorrecto')
            if any(v not in self.graph.vertices for v in path):
                raise RuntimeError('Una ruta de entrega atraviesa un obstáculo')
            if any(a != b and b not in self.graph.neighbors[a]
                   for a, b in zip(path, path[1:])):
                raise RuntimeError('Una ruta de entrega contiene una arista prohibida')
        return DeliveryPlan(starts, picked_names, colors, goals, paths,
                            result['cost'], result['expanded'], result['generated'])
