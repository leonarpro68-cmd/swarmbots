#!/usr/bin/env python3
"""CBS + barrera de pasos para tres Summit XLS en ROS 2 Jazzy.

Reutiliza SIN modificar:
  grafo_cbs_125.py, cbs_grafo_125.py, cbs_step_executor_demo.py,
  y tres instancias de path_follower.py.

No introduce chequeos geométricos adicionales. Emplea las rutas de CBS
existentes (incluida su detección de conflictos) y espera una barrera de
llegada antes de cada transición. La barrera NO garantiza seguridad física
entre robots durante transiciones con velocidades distintas.

Uso:
  python3 cbs_ros_executor.py
  python3 cbs_ros_executor.py --goals 14,18 8,18 11,22
"""
from __future__ import annotations

import argparse
import math
import time

import grafo_cbs_125 as geom
import cbs_grafo_125 as cbs
from cbs_step_executor_demo import SyncDemo, select_alignment


def pad_paths(paths):
    """Prolonga cada ruta con WAIT finales para disponer de pasos comunes."""
    horizon = max(map(len, paths))
    return tuple(tuple(p) + (p[-1],) * (horizon - len(p)) for p in paths)


def extract_step(paths, step):
    """Devuelve un vértice destino por robot en el paso discreto solicitado."""
    if step < 0 or step >= len(paths[0]):
        raise IndexError(step)
    return {f'summit{i}': path[step] for i, path in enumerate(paths)}


class CBSRosExecutor(SyncDemo):
    def __init__(self, goals, max_ct=3000, max_time=65):
        # SyncDemo crea los publishers, subscriptions, temporizador y barrera.
        # El temporizador llama a self.tick(), implementado aquí.
        self.goals = tuple(goals)
        self.max_ct = max_ct
        self.max_time = max_time
        self.planned = ()
        self.result = None
        super().__init__()
        self.node.get_logger().info('CBS ROS: esperando odometría para alineación inicial')

    def tick(self):
        if self.phase in ('DONE', 'ABORT'):
            return
        if not self.fresh():
            if self.phase != 'WAIT_ODOM':
                self.fail('Odometría ausente o vencida: cancelando ejecución CBS')
            return
        if self.phase == 'WAIT_ODOM':
            try:
                starts = tuple(select_alignment(self.vertices, self.odom[a][:2])
                               for a in self.agents)
                if any(goal not in self.vertices for goal in self.goals):
                    raise ValueError(f'Meta fuera del grafo libre: {self.goals}')
                # No se permite iniciar dos Summit en vértices demasiado próximos.
                if cbs.first_conflict(tuple((v,) for v in starts)) is not None:
                    raise ValueError('Los vértices iniciales no permiten separación CBS')
                self.node.get_logger().info(f'Inicios: {starts} | Metas: {self.goals}')
                self.result = cbs.solve(starts, self.goals,
                                        max_ct=self.max_ct, max_time=self.max_time)
                self.planned = pad_paths(self.result['paths'])
                if cbs.first_conflict(self.planned) is not None:
                    raise ValueError('La solución CBS todavía tiene conflictos')
                for i, route in enumerate(self.planned):
                    self.node.get_logger().info(f'R{i}: {route}')
                self.node.get_logger().info(
                    f'CBS OK: costo={self.result["cost"]}, '
                    f'CT expandidos={self.result["expanded"]}, '
                    f'pasos sincronizados={len(self.planned[0])-1}')
                # Primero ir hasta los nodos iniciales del plan.
                self.start_phase('ALIGN', {
                    a: geom.node_position(starts[i])
                    for i, a in enumerate(self.agents)
                })
            except (ValueError, RuntimeError) as ex:
                self.fail(f'No se pudo planificar CBS: {ex}')
            return

        now = time.monotonic()
        if now - self.started > self.timeout:
            self.fail(f'Tiempo agotado en {self.phase}')
            return
        for agent in self.agents:
            x, y, speed, _, _ = self.odom[agent]
            gx, gy = self.target[agent]
            reached = math.hypot(x - gx, y - gy) <= self.tol and speed <= self.speed_limit
            if reached:
                if self.holding_since[agent] is None:
                    self.holding_since[agent] = now
            else:
                self.holding_since[agent] = None
        if not all(self.holding_since[a] is not None and
                   now - self.holding_since[a] >= self.hold for a in self.agents):
            return
        self.node.get_logger().info(f'BARRERA superada: {self.phase}')
        if self.phase == 'ALIGN':
            next_step = 1
        else:
            next_step = self.step_idx + 1
        if next_step >= len(self.planned[0]):
            self.stop_all()
            self.phase = 'DONE'
            self.node.get_logger().info('CBS COMPLETADO: todos llegaron a sus metas')
            return
        self.step_idx = next_step
        vertices = extract_step(self.planned, next_step)
        waits = [a for i, a in enumerate(self.agents)
                 if self.planned[i][next_step] == self.planned[i][next_step-1]]
        self.node.get_logger().info(
            f'STEP_{next_step}: ' + ', '.join(f'{a}->{vertices[a]}' for a in self.agents)
            + (f' | WAIT: {waits}' if waits else ''))
        self.start_phase(f'STEP_{next_step}', {
            a: geom.node_position(vertices[a]) for a in self.agents
        })


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--goals', nargs=3, default=['14,18', '8,18', '11,22'],
                    metavar='I,J', help='Tres metas como índices de vértice')
    ap.add_argument('--max-ct', type=int, default=3000)
    ap.add_argument('--max-time', type=int, default=65)
    args, ros_args = ap.parse_known_args()
    try:
        goals = tuple(tuple(map(int, s.split(','))) for s in args.goals)
        if any(len(v) != 2 for v in goals):
            raise ValueError
    except ValueError:
        ap.error('Usa --goals I,J I,J I,J, por ejemplo 14,18 8,18 11,22')
    return goals, args.max_ct, args.max_time, ros_args


def main():
    import rclpy
    goals, max_ct, max_time, ros_args = parse_args()
    rclpy.init(args=ros_args)
    demo = CBSRosExecutor(goals, max_ct=max_ct, max_time=max_time)
    try:
        rclpy.spin(demo.node)
    except KeyboardInterrupt:
        pass
    finally:
        demo.stop_all()
        demo.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
