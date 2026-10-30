#!/usr/bin/env python3
"""Transporte físico CBS V2: espera DDS y diagnostica barreras sin alterar CBS.

USAR SOLO con tres cargas físicamente sujetas y robots en vértices de recogida.
No agarra, suelta, elimina ni confirma entrega. No permite reinicios automáticos.
"""
from __future__ import annotations

import argparse
import math
import time

import rclpy
from swarm_behavior.swarm_planning.ros_plan_helpers import AGENTS
from ros_transporte_fisico_cbs import PhysicalDeliveryNavigation


class PhysicalDeliveryNavigationV2(PhysicalDeliveryNavigation):
    def __init__(self, scenario, carried, *, discovery_timeout=20.0,
                 diagnostic_interval=2.0, retry_interval=5.0, max_retries=3, **kwargs):
        self.discovery_timeout = discovery_timeout
        self.diagnostic_interval = diagnostic_interval
        self.retry_interval = retry_interval
        self.max_retries = max_retries
        self.discovery_started = time.monotonic()
        self._last_discovery_log = 0.0
        self._last_diag = 0.0
        self._last_retry = 0.0
        self._retry_count = 0
        super().__init__(scenario, carried, **kwargs)
        # La clase base ya puso DELIVERY_WAIT_ODOM. Retenemos ese estado
        # hasta que los tres subscriptores DDS sean visibles.
        self.node.get_logger().info('V2: esperando DDS para /summit[0..2]/planned_path')

    def _dds_ready(self):
        counts = {a: self.pubs[a].get_subscription_count() for a in AGENTS}
        if all(n >= 1 for n in counts.values()):
            return True
        now = time.monotonic()
        if now - self._last_discovery_log >= 2.0:
            self._last_discovery_log = now
            self.node.get_logger().warn(f'V2 DDS: suscriptores planned_path={counts}; esperando')
        if now - self.discovery_started > self.discovery_timeout:
            self.fail(f'V2: timeout esperando suscriptores DDS: {counts}')
        return False

    def start_barrier(self, name, targets):
        counts = {a: self.pubs[a].get_subscription_count() for a in AGENTS}
        if any(n == 0 for n in counts.values()):
            self.fail(f'V2: enlace DDS perdido antes de {name}: {counts}')
            return
        super().start_barrier(name, targets)
        self._last_diag = 0.0
        self._last_retry = time.monotonic()
        self._retry_count = 0
        self.node.get_logger().info(f'V2 PUBLICADO {name}: suscriptores DDS={counts}')

    def tick(self):
        if self.state == 'DELIVERY_WAIT_ODOM' and not self._dds_ready():
            return
        if self.state == 'MOVING' and self.barrier is not None:
            now = time.monotonic()
            if now - self._last_diag >= self.diagnostic_interval:
                self._last_diag = now
                parts = []
                for a in AGENTS:
                    sample = self.samples.get(a)
                    if sample is None:
                        parts.append(f'{a}: sin odometría')
                        continue
                    d = math.dist((sample.x, sample.y), self.barrier.targets[a])
                    v = self.velocities.get(a, float('nan'))
                    n = self.pubs[a].get_subscription_count()
                    parts.append(f'{a}: d={d:.3f}m v={v:.3f}m/s DDS={n}')
                self.node.get_logger().info(
                    f'V2 DIAGNOSTICO {self.barrier.name} | ' + ' | '.join(parts))
            # Si una publicación se perdió justo en el cambio de ejecutor,
            # reenviar SOLO el objetivo actual de robots fuera de tolerancia.
            # Los robots en WAIT o que ya llegaron NO reciben nueva orden.
            if now - self._last_retry >= self.retry_interval and self._retry_count < self.max_retries:
                self._last_retry = now
                self._retry_count += 1
                for a in AGENTS:
                    sample = self.samples.get(a)
                    if sample is None:
                        continue
                    distance = math.dist((sample.x, sample.y), self.barrier.targets[a])
                    count = self.pubs[a].get_subscription_count()
                    if count == 0:
                        self.fail(f'V2: enlace DDS perdido para {a} durante {self.barrier.name}')
                        return
                    if distance > self.arrival_tolerance:
                        self.publish(a, self.barrier.targets[a])
                        self.node.get_logger().warn(
                            f'V2 REENVIO {self._retry_count}/{self.max_retries}: '
                            f'{self.barrier.name} {a}, d={distance:.3f} m')
        super().tick()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--carried', nargs=3, required=True)
    ap.add_argument('--output', default='/tmp/swarm_transporte_fisico_v2.json')
    ap.add_argument('--step-timeout', type=float, default=90.0)
    ap.add_argument('--discovery-timeout', type=float, default=20.0)
    ap.add_argument('--diagnostic-interval', type=float, default=2.0)
    ap.add_argument('--retry-interval', type=float, default=5.0)
    ap.add_argument('--max-retries', type=int, default=3)
    ap.add_argument('--max-ct', type=int, default=20000)
    ap.add_argument('--max-time', type=int, default=65)
    args, ros_args = ap.parse_known_args()
    if (args.step_timeout < 20 or args.discovery_timeout < 3
            or args.diagnostic_interval < 0.5 or args.retry_interval < 3
            or not 0 <= args.max_retries <= 5):
        ap.error('Parámetros de recuperación fuera de los límites de prueba')
    rclpy.init(args=ros_args)
    exe = None
    try:
        exe = PhysicalDeliveryNavigationV2(
            args.scenario, args.carried, output=args.output,
            step_timeout=args.step_timeout, discovery_timeout=args.discovery_timeout,
            diagnostic_interval=args.diagnostic_interval, retry_interval=args.retry_interval,
            max_retries=args.max_retries, max_ct=args.max_ct, max_time=args.max_time)
        while rclpy.ok() and exe.state not in ('DONE', 'FAILED'):
            rclpy.spin_once(exe.node, timeout_sec=0.2)
        return 0 if exe.state == 'DONE' else 1
    except KeyboardInterrupt:
        return 130
    finally:
        if exe is not None:
            if rclpy.ok():
                exe.close()
            else:
                exe.coordinator.close()
                exe.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
