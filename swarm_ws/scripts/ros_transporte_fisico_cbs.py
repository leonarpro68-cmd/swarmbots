#!/usr/bin/env python3
"""Prueba experimental: transportar tres basuras ya sujetas mediante CBS.

No solicita agarres, no elimina modelos, no suelta las basuras y NO confirma
la entrega lógica. Los tres nombres --carried corresponden a summit0/1/2.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from swarm_behavior.swarm_planning.geometry import node_position
from swarm_behavior.swarm_planning.ros_plan_helpers import AGENTS, planning_starts
from swarm_behavior.swarm_planning.ros_pickup_executor import PickupExecutor, padded_paths, barrier_complete


class PhysicalDeliveryNavigation(PickupExecutor):
    def __init__(self, scenario, carried, *, output, **kwargs):
        self.carried_names = tuple(carried)
        self.output = Path(output)
        super().__init__(scenario, **kwargs)
        # Es un ejecutor nuevo: no realizar el pickup de 6B.
        if len(self.carried_names) != len(AGENTS) or len(set(self.carried_names)) != len(AGENTS):
            raise ValueError('Se requieren tres basuras distintas en orden summit0, summit1, summit2')
        data = json.loads(Path(scenario).read_text(encoding='utf-8'))
        valid_trash = {e['name'] for e in data['elements'] if e['type'] == 'trash'}
        if any(n not in valid_trash for n in self.carried_names):
            raise ValueError('Algún nombre --carried no corresponde a basura del escenario')
        self.coordinator.confirm_pickups(self.carried_names)
        self.state = 'DELIVERY_WAIT_ODOM'
        self.node.get_logger().info('Transporte físico: objetos sujetos externamente; esperando odometría')
        self.node.get_logger().info(f'Cargas declaradas (summit0..2): {self.carried_names}')

    def tick(self):
        if self.state in ('DONE', 'FAILED'):
            return
        if self.state == 'DELIVERY_WAIT_ODOM':
            fresh = self.fresh()
            if fresh is None:
                return
            try:
                starts = planning_starts(self.coordinator.bridge.graph, fresh)
                self.coordinator.submit_delivery(starts)
            except (ValueError, RuntimeError) as exc:
                self.fail(f'No se pudo solicitar CBS entrega: {exc}')
                return
            self.state = 'DELIVERY_PLANNING'
            self.node.get_logger().info(f'CBS entrega solicitado; inicios={starts}')
            return
        if self.state == 'DELIVERY_PLANNING':
            outcome = self.coordinator.poll()
            if outcome is None:
                return
            if outcome.error is not None or outcome.kind != 'delivery':
                self.fail(f'CBS entrega falló: {outcome.error or outcome.kind}')
                return
            self.delivery_plan = outcome.value
            self.routes = padded_paths(self.delivery_plan.paths)
            self.step = 0
            self.node.get_logger().info(f'CBS ENTREGA OK: costo={self.delivery_plan.cost}; pasos={len(self.routes[0])-1}')
            for i, agent in enumerate(AGENTS):
                self.node.get_logger().info(f'{agent}: lleva={self.carried_names[i]}, depósito color={self.delivery_plan.colors[i]}, meta={self.routes[i][-1]}')
            self.state = 'DELIVERY_REFRESH_ODOM'
            return
        if self.state == 'DELIVERY_REFRESH_ODOM':
            if self.fresh() is None:
                return
            for i, agent in enumerate(AGENTS):
                sample = self.samples[agent]
                target = node_position(self.routes[i][0])
                if math.dist((sample.x, sample.y), target) > 2.5:
                    self.fail(f'{agent} está demasiado lejos del inicio planificado')
                    return
            self.start_barrier('DELIVERY_ALIGN', {
                agent: node_position(self.routes[i][0]) for i, agent in enumerate(AGENTS)
            })
            return
        if self.state == 'MOVING':
            if self.fresh() is None:
                self.fail('Odometría vencida durante transporte')
                return
            now = time.monotonic()
            if now - self.barrier.since > self.step_timeout:
                self.fail(f'Tiempo agotado en {self.barrier.name}')
                return
            if not barrier_complete(self.barrier, self.samples, self.velocities, now,
                                    self.arrival_tolerance, self.arrival_speed, self.hold_seconds):
                return
            self.node.get_logger().info(f'BARRERA superada: {self.barrier.name}')
            self.step += 1
            if self.step >= len(self.routes[0]):
                self.stop()
                payload = {
                    'status': 'arrived_not_delivered',
                    'carried': dict(zip(AGENTS, self.carried_names)),
                    'colors': list(self.delivery_plan.colors),
                    'delivery_goals': {a: list(self.routes[i][-1]) for i, a in enumerate(AGENTS)},
                    'reference_xy': {a: [self.samples[a].x, self.samples[a].y] for a in AGENTS},
                    'cost': self.delivery_plan.cost,
                    'note': 'No se solicitó detach ni se confirmó entrega física',
                }
                self.output.parent.mkdir(parents=True, exist_ok=True)
                self.output.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
                self.state = 'DONE'
                self.node.get_logger().info('TRANSPORTE COMPLETADO: robots en depósitos, basuras aún sujetas; NO ENTREGADAS')
                self.node.get_logger().info(f'Referencias para maniobras locales: {self.output}')
                return
            waits = [a for i, a in enumerate(AGENTS) if self.routes[i][self.step] == self.routes[i][self.step-1]]
            self.node.get_logger().info(f'DELIVERY_STEP_{self.step}; WAIT={waits}')
            self.start_barrier(f'DELIVERY_STEP_{self.step}', {
                agent: node_position(self.routes[i][self.step]) for i, agent in enumerate(AGENTS)
            })
            return
        self.fail(f'Estado inesperado: {self.state}')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--carried', required=True, nargs=3, metavar=('SUMMIT0', 'SUMMIT1', 'SUMMIT2'))
    ap.add_argument('--output', default='/tmp/swarm_transporte_fisico.json')
    ap.add_argument('--step-timeout', type=float, default=90.0)
    ap.add_argument('--max-ct', type=int, default=20000)
    ap.add_argument('--max-time', type=int, default=65)
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    executor = None
    try:
        executor = PhysicalDeliveryNavigation(args.scenario, args.carried, output=args.output,
                                              step_timeout=args.step_timeout,
                                              max_ct=args.max_ct, max_time=args.max_time)
        while rclpy.ok() and executor.state not in ('DONE', 'FAILED'):
            rclpy.spin_once(executor.node, timeout_sec=0.2)
        return 0 if executor.state == 'DONE' else 1
    except KeyboardInterrupt:
        return 130
    finally:
        if executor is not None:
            if rclpy.ok():
                executor.close()
            else:
                executor.coordinator.close()
                executor.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
