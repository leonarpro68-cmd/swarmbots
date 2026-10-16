#!/usr/bin/env python3
"""Etapa 6D: una ronda de recogida confirmada y entrega sincronizada.

No inicia la segunda ronda. Mantiene la versión 6C intacta.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

from .geometry import node_position
from .ros_plan_helpers import AGENTS, planning_starts
from .ros_pickup_executor import padded_paths, barrier_complete
from .ros_pickup_removal import PickupRemovalExecutor


class PickupDeliveryExecutor(PickupRemovalExecutor):
    """Extensión temporal de 6C, limitada a una ronda para validación real."""

    def __init__(self, scenario, *, delivery_state_file='/tmp/swarm_refactor_6d.json', **kwargs):
        self.delivery_state_file = Path(delivery_state_file)
        self.delivery_active = False
        super().__init__(scenario, **kwargs)
        self.node.get_logger().info('6D: recoger tres basuras y entregarlas; SIN segunda ronda')

    def start_barrier(self, name, targets):
        if self.delivery_active and name.startswith('STEP_'):
            name = 'DELIVERY_' + name
        super().start_barrier(name, targets)

    def _poll_removal(self):
        # Actualización del mapa y confirmaciones: exactamente el código validado en 6C.
        super()._poll_removal()
        if self.state == 'PICKUP_DONE':
            if not self.coordinator._carrying or len(self.coordinator._carrying) != 3:
                self.fail('6D: recogida sin tres cargas confirmadas')
                return
            self.state = 'DELIVERY_WAIT_ODOM'
            self.node.get_logger().info('6D: recogida confirmada; esperando odometría para CBS de entrega')

    def tick(self):
        if self.state == 'DELIVERY_WAIT_ODOM':
            current = self.fresh()
            if current is None:
                return
            try:
                starts = planning_starts(self.coordinator.bridge.graph, current)
                self.coordinator.submit_delivery(starts)
            except (ValueError, RuntimeError) as exc:
                self.fail(f'6D: no se pudo iniciar CBS de entrega: {exc}')
                return
            self.state = 'DELIVERY_PLANNING'
            self.node.get_logger().info(f'6D: CBS entrega asíncrono: inicios={starts}')
            return
        if self.state == 'DELIVERY_PLANNING':
            result = self.coordinator.poll()
            if result is None:
                return
            if result.error is not None or result.kind != 'delivery':
                self.fail(f'6D: CBS entrega falló: {result.error or result.kind}')
                return
            self.delivery_plan = result.value
            self.routes = padded_paths(result.value.paths)
            self.step = 0
            self.delivery_active = True
            self.node.get_logger().info(
                f'CBS ENTREGA OK: costo={result.value.cost}, '
                f'CT={result.value.expanded}, pasos={len(self.routes[0])-1}')
            for i, a in enumerate(AGENTS):
                self.node.get_logger().info(
                    f'{a}: depósito color={result.value.colors[i]}, meta={self.routes[i][-1]}')
            # CBS puede consumir segundos. Esperar de nuevo odometría fresca.
            self.state = 'DELIVERY_REFRESH_ODOM'
            return
        if self.state == 'DELIVERY_REFRESH_ODOM':
            if self.fresh() is None:
                return
            for i, a in enumerate(AGENTS):
                current = self.samples[a]
                target = node_position(self.routes[i][0])
                if math.dist((current.x, current.y), target) > 2.5:
                    self.fail(f'6D: {a} cambió demasiado durante el cálculo de entrega')
                    return
            self.start_barrier('DELIVERY_ALIGN',
                               {a: node_position(self.routes[i][0]) for i, a in enumerate(AGENTS)})
            self.state = 'DELIVERY_MOVING'
            return
        if self.state == 'DELIVERY_MOVING':
            if self.fresh() is None:
                self.fail('6D: odometría vencida durante entrega')
                return
            now = time.monotonic()
            if now - self.barrier.since > self.step_timeout:
                self.fail(f'6D: tiempo agotado en {self.barrier.name}')
                return
            if not barrier_complete(self.barrier, self.samples, self.velocities, now,
                                    self.arrival_tolerance, self.arrival_speed, self.hold_seconds):
                return
            self.node.get_logger().info(f'BARRERA superada: {self.barrier.name}')
            self.step += 1
            if self.step >= len(self.routes[0]):
                self.stop()
                try:
                    self.coordinator.confirm_delivery()
                except Exception as exc:
                    self.fail(f'6D: no se pudo confirmar entrega: {exc}')
                    return
                self.state = 'DELIVERY_DONE'
                self._save_delivery()
                self.node.get_logger().info(
                    f'6D COMPLETADA: {len(self.coordinator.delivered)} basuras entregadas; '
                    f'{len(self.coordinator.pending)} pendientes; SIN segunda ronda')
                return
            waits = [a for i, a in enumerate(AGENTS)
                     if self.routes[i][self.step] == self.routes[i][self.step - 1]]
            self.node.get_logger().info(f'DELIVERY_STEP_{self.step}; WAIT={waits}')
            self.start_barrier(f'DELIVERY_STEP_{self.step}',
                               {a: node_position(self.routes[i][self.step]) for i, a in enumerate(AGENTS)})
            self.state = 'DELIVERY_MOVING'
            return
        if self.state in ('DELIVERY_DONE', 'REMOVAL_FAILED', 'FAILED'):
            return
        super().tick()

    def _save_delivery(self):
        payload = {
            'status': 'delivered',
            'delivered': list(self.coordinator.delivered),
            'pending': sorted(self.coordinator.pending),
            'colors': list(self.delivery_plan.colors),
            'delivery_goals': [list(path[-1]) for path in self.delivery_plan.paths],
            'delivery_cost': self.delivery_plan.cost,
            'delivery_ct_expanded': self.delivery_plan.expanded,
            'vertices_free': len(self.coordinator.bridge.graph.vertices),
            'note': '6D: una ronda completa, segunda ronda aún no ejecutada',
        }
        try:
            self.delivery_state_file.parent.mkdir(parents=True, exist_ok=True)
            self.delivery_state_file.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
        except OSError as exc:
            self.node.get_logger().error(f'No se pudo guardar resultado 6D: {exc}')


def main():
    import rclpy
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--world', default=None)
    ap.add_argument('--state-file', default='/tmp/swarm_refactor_6c.json')
    ap.add_argument('--delivery-state-file', default='/tmp/swarm_refactor_6d.json')
    ap.add_argument('--odom-timeout', type=float, default=2.)
    ap.add_argument('--step-timeout', type=float, default=60.)
    ap.add_argument('--arrival-tolerance', type=float, default=.12)
    ap.add_argument('--arrival-speed', type=float, default=.10)
    ap.add_argument('--hold-seconds', type=float, default=.6)
    ap.add_argument('--max-ct', type=int, default=20000)
    ap.add_argument('--max-time', type=int, default=65)
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    executor = None
    try:
        executor = PickupDeliveryExecutor(
            args.scenario, world=args.world, state_file=args.state_file,
            delivery_state_file=args.delivery_state_file,
            odom_timeout=args.odom_timeout, step_timeout=args.step_timeout,
            arrival_tolerance=args.arrival_tolerance, arrival_speed=args.arrival_speed,
            hold_seconds=args.hold_seconds, max_ct=args.max_ct, max_time=args.max_time)
        while rclpy.ok() and executor.state not in ('DELIVERY_DONE', 'REMOVAL_FAILED', 'FAILED'):
            rclpy.spin_once(executor.node, timeout_sec=.2)
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
