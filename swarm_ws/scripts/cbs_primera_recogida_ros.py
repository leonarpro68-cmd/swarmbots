#!/usr/bin/env python3
"""Primera ronda REAL: odometría -> asignación euclidiana -> CBS -> barreras.

No elimina todavía objetos de Gazebo: se detiene junto a tres basuras.
No modifica path_follower.py, ni otros componentes ya probados.
Requiere las seis basuras fijas ya colocadas en el mundo.
"""
from __future__ import annotations
import argparse
import math
import time

import grafo_cbs_125 as geom
import cbs_grafo_125 as cbs
from cbs_step_executor_demo import SyncDemo
from cbs_ros_executor import pad_paths, extract_step
from cbs_asignacion_basuras import assign_nearest, approach_vertices, solve_multi
from grafo_basuras_dinamico import DynamicTrashGraph


def choose_safe_alignment(graph, xy, max_distance=2.5):
    """Elegir vértice libre con segmento de alineación libre de basuras."""
    if not graph.free_position(xy):
        raise ValueError(f'Robot ubicado dentro de zona prohibida: {xy}')
    for v in sorted(graph.vertices, key=lambda p: (math.dist(xy, geom.node_position(p)), p)):
        dest = geom.node_position(v)
        if math.dist(xy, dest) > max_distance:
            break
        if graph.segment_clear(xy, dest):
            return v
    raise ValueError(f'Sin vértice de alineación seguro alrededor de {xy}')


def plan(starts, graph=None, max_ct=20000, max_time=65):
    graph = graph or DynamicTrashGraph()
    starts = tuple(starts)
    if any(s not in graph.vertices for s in starts):
        raise ValueError('Algún inicio está bloqueado por basura o columna')
    if cbs.first_conflict(tuple((s,) for s in starts)) is not None:
        raise ValueError('Inicios demasiado próximos para CBS')
    assignments = assign_nearest(starts, graph)
    goal_sets = tuple(approach_vertices(graph, name) for name, _ in assignments)
    result = solve_multi(graph, starts, goal_sets, max_ct=max_ct, max_time=max_time)
    if cbs.first_conflict(result['paths']) is not None:
        raise ValueError('CBS devolvió trayectorias incompatibles')
    for i, path in enumerate(result['paths']):
        if path[0] != starts[i] or path[-1] not in goal_sets[i]:
            raise ValueError(f'Ruta {i}: inicio o destino inválido')
        if any(v not in graph.vertices for v in path):
            raise ValueError(f'Ruta {i} atraviesa basura pendiente')
        if any(a != b and b not in graph.neighbors[a] for a, b in zip(path, path[1:])):
            raise ValueError(f'Ruta {i} usa una conexión prohibida')
    return assignments, goal_sets, result


class FirstPickupExecutor(SyncDemo):
    def __init__(self, max_ct=20000, max_time=65):
        self.dynamic = DynamicTrashGraph()
        self.max_ct = max_ct
        self.max_time = max_time
        self.planned = ()
        self.assignments = None
        super().__init__()
        self.node.get_logger().info('Primera ronda: esperando odometría para asignar basuras')

    def tick(self):
        if self.phase in ('DONE', 'ABORT'):
            return
        if not self.fresh():
            if self.phase != 'WAIT_ODOM':
                self.fail('Odometría ausente: se detienen los Summit')
            return
        if self.phase == 'WAIT_ODOM':
            try:
                starts = tuple(choose_safe_alignment(self.dynamic, self.odom[a][:2])
                               for a in self.agents)
                self.assignments, goal_sets, result = plan(
                    starts, self.dynamic, self.max_ct, self.max_time)
                self.planned = pad_paths(result['paths'])
                self.node.get_logger().info(f'Inicios: {starts}')
                for i, (name, dist) in enumerate(self.assignments):
                    self.node.get_logger().info(
                        f'{self.agents[i]}: basura={name} distancia={dist:.2f}m '
                        f'meta={result["paths"][i][-1]} ruta={result["paths"][i]}')
                self.node.get_logger().info(
                    f'CBS OK: costo={result["cost"]}, CT={result["expanded"]}, '
                    f'pasos={len(self.planned[0])-1}')
                self.start_phase('ALIGN', {
                    a: geom.node_position(starts[i]) for i, a in enumerate(self.agents)
                })
            except (ValueError, RuntimeError) as exc:
                self.fail(f'Planificación fallida: {exc}')
            return

        now = time.monotonic()
        if now - self.started > self.timeout:
            self.fail(f'Tiempo agotado en {self.phase}')
            return
        for agent in self.agents:
            x, y, speed, _, _ = self.odom[agent]
            gx, gy = self.target[agent]
            reached = math.hypot(x-gx, y-gy) <= self.tol and speed <= self.speed_limit
            self.holding_since[agent] = (
                now if reached and self.holding_since[agent] is None
                else self.holding_since[agent] if reached else None
            )
        if not all(self.holding_since[a] is not None and
                   now-self.holding_since[a] >= self.hold for a in self.agents):
            return
        self.node.get_logger().info(f'BARRERA superada: {self.phase}')
        next_step = 1 if self.phase == 'ALIGN' else self.step_idx + 1
        if next_step >= len(self.planned[0]):
            self.stop_all()
            self.phase = 'DONE'
            self.node.get_logger().info('PRIMERA RONDA TERMINADA: los tres Summit están junto a su basura')
            for i, (name, _) in enumerate(self.assignments):
                self.node.get_logger().info(f'{self.agents[i]} junto a {name}: '
                                            'todavía NO recogida ni eliminada de Gazebo')
            return
        self.step_idx = next_step
        nodes = extract_step(self.planned, next_step)
        waits = [a for i, a in enumerate(self.agents)
                 if self.planned[i][next_step] == self.planned[i][next_step-1]]
        self.node.get_logger().info(f'STEP_{next_step}: {nodes} | WAIT={waits}')
        self.start_phase(f'STEP_{next_step}', {
            a: geom.node_position(nodes[a]) for a in self.agents
        })


def main():
    import rclpy
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--max-ct', type=int, default=20000)
    ap.add_argument('--max-time', type=int, default=65)
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    executor = FirstPickupExecutor(args.max_ct, args.max_time)
    try:
        rclpy.spin(executor.node)
    except KeyboardInterrupt:
        pass
    finally:
        executor.stop_all()
        executor.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
