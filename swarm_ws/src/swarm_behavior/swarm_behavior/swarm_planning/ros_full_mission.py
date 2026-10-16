#!/usr/bin/env python3
"""6E: dos rondas automáticas de recogida y entrega con CBS y mapa persistente.

Extensión acotada del ejecutor validado 6D. No modifica los ejecutores estables.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .ros_pickup_delivery import PickupDeliveryExecutor


class FullMissionExecutor(PickupDeliveryExecutor):
    """Reutiliza 6D por ronda y conserva PlanningBridge/Coordinator entre rondas."""

    def __init__(self, scenario, *, mission_state_file='/tmp/swarm_refactor_6e.json', **kwargs):
        self.mission_state_file = Path(mission_state_file)
        self.round_index = 1
        self.rounds = []
        super().__init__(scenario, **kwargs)
        self.node.get_logger().info('6E: misión completa, dos rondas automáticas')

    def tick(self):
        if self.state in ('MISSION_DONE', 'FAILED', 'REMOVAL_FAILED'):
            return
        super().tick()
        if self.state == 'DELIVERY_DONE':
            self._finish_round()

    def _finish_round(self):
        """El estado DELIVERY_DONE solo ocurre tras barrera y confirm_delivery de 6D."""
        if self.coordinator._carrying:
            self.fail('6E: quedan basuras transportadas sin entregar')
            return
        delivered = list(self.coordinator.delivered)
        if len(delivered) != self.round_index * 3 or len(set(delivered)) != len(delivered):
            self.fail('6E: número de entregas o identificadores incoherentes')
            return
        self.rounds.append({
            'round': self.round_index,
            'delivered_total': len(delivered),
            'pending': sorted(self.coordinator.pending),
            'pickup_assignment': [name for name, _ in self.result.assigned],
            'pickup_cost': self.result.cost,
            'delivery_cost': self.delivery_plan.cost,
            'delivery_goals': [list(route[-1]) for route in self.delivery_plan.paths],
        })
        self.node.get_logger().info(
            f'RONDA {self.round_index} ENTREGADA: {len(delivered)} entregadas; '
            f'{len(self.coordinator.pending)} pendientes')
        self._save_mission('in_progress')
        if self.coordinator.mission_complete:
            self.state = 'MISSION_DONE'
            self.stop()
            self._save_mission('completed')
            self.node.get_logger().info(
                f'MISIÓN COMPLETADA: {len(delivered)}/{len(self.coordinator.bridge.graph.trash)} '
                'basuras recogidas y entregadas; 0 pendientes')
            return
        if not self.coordinator.pending:
            self.fail('6E: no quedan basuras, pero el coordinador no confirma misión completa')
            return
        if len(self.coordinator.pending) < 3 or len(self.coordinator.pending) % 3:
            self.fail('6E: número de basuras restantes no compatible con grupos de tres')
            return
        # El núcleo original ya está libre: ningún CBS activo y ninguna carga pendiente.
        if self.coordinator.busy or self.removal_future is None or not self.removal_future.done():
            self.fail('6E: hay operaciones pendientes al cambiar de ronda')
            return
        self.round_index += 1
        self.delivery_active = False
        self.removal_future = None
        self.delivery_plan = None
        self.result = None
        self.routes = ()
        self.step = 0
        self.barrier = None
        self.state = 'WAIT_ODOM'
        self.node.get_logger().info(
            f'RONDA {self.round_index}: esperando odometría actual para asignar '
            f'{len(self.coordinator.pending)} basuras pendientes')

    def _save_mission(self, status):
        payload = {
            'status': status,
            'rounds_completed': len(self.rounds),
            'rounds': self.rounds,
            'delivered': list(self.coordinator.delivered),
            'pending': sorted(self.coordinator.pending),
            'carrying': list(self.coordinator._carrying),
            'vertices_free': len(self.coordinator.bridge.graph.vertices),
            'edges_free': len(self.coordinator.bridge.graph.edges),
        }
        try:
            self.mission_state_file.parent.mkdir(parents=True, exist_ok=True)
            self.mission_state_file.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        except OSError as exc:
            self.node.get_logger().error(f'No se pudo guardar estado 6E: {exc}')


def main():
    import rclpy
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--world', default=None)
    ap.add_argument('--state-file', default='/tmp/swarm_refactor_6c.json')
    ap.add_argument('--delivery-state-file', default='/tmp/swarm_refactor_6d.json')
    ap.add_argument('--mission-state-file', default='/tmp/swarm_refactor_6e.json')
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
        executor = FullMissionExecutor(
            args.scenario, world=args.world,
            state_file=args.state_file,
            delivery_state_file=args.delivery_state_file,
            mission_state_file=args.mission_state_file,
            odom_timeout=args.odom_timeout,
            step_timeout=args.step_timeout,
            arrival_tolerance=args.arrival_tolerance,
            arrival_speed=args.arrival_speed,
            hold_seconds=args.hold_seconds,
            max_ct=args.max_ct, max_time=args.max_time)
        while rclpy.ok() and executor.state not in ('MISSION_DONE', 'FAILED', 'REMOVAL_FAILED'):
            rclpy.spin_once(executor.node, timeout_sec=.2)
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
