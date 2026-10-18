#!/usr/bin/env python3
"""Segunda ronda, prueba supervisada: diagnostico y reenvio acotado SOLO de ALIGN.

No modifica CBS ni path_follower; no hace agarres ni elimina objetos.
Partir exclusivamente del estado fisico al final de ronda 1.
"""
import argparse
import math
import time
import rclpy

from ros_recogida_ronda2_cbs import Round2Pickup, previous_names
from swarm_behavior.swarm_planning.ros_pickup_executor import AGENTS


class Round2PickupDiagnostic(Round2Pickup):
    def __init__(self, scenario, previous, *, retry_interval=3.0, max_retries=5,
                 status_interval=2.0, **kwargs):
        self.retry_interval = retry_interval
        self.max_retries = max_retries
        self.status_interval = status_interval
        self.align_retries = 0
        self.align_last_send = None
        self.last_status = 0.0
        super().__init__(scenario, previous, **kwargs)
        self.node.get_logger().info('R2 V2: diagnóstico ALIGN; reenvío acotado y SOLO durante ALIGN')

    def start_barrier(self, name, targets):
        super().start_barrier(name, targets)
        if name == 'ALIGN':
            self.align_retries = 0
            self.align_last_send = time.monotonic()
            self.last_status = 0.0

    def tick(self):
        # Solo dentro del estado MOVING / ALIGN. El resto del ejecutor es idéntico al estable.
        if self.state == 'MOVING' and self.barrier is not None and self.barrier.name == 'ALIGN':
            now = time.monotonic()
            if now - self.last_status >= self.status_interval:
                self.last_status = now
                details = []
                for a in AGENTS:
                    sample = self.samples.get(a)
                    if sample is None:
                        details.append(f'{a}: sin odometría')
                        continue
                    goal = self.barrier.targets[a]
                    d = math.dist((sample.x, sample.y), goal)
                    speed = self.velocities.get(a, float('nan'))
                    age = now - sample.received_at if hasattr(sample, 'received_at') else float('nan')
                    details.append(f'{a}: d={d:.3f}m, v={speed:.3f}m/s')
                self.node.get_logger().info('ALIGN DIAGNOSTICO | ' + ' | '.join(details))
            if self.align_last_send is not None and now - self.align_last_send >= self.retry_interval:
                self.align_last_send = now
                if self.align_retries < self.max_retries:
                    self.align_retries += 1
                    for a in AGENTS:
                        sample = self.samples.get(a)
                        if sample is None:
                            continue
                        goal = self.barrier.targets[a]
                        d = math.dist((sample.x, sample.y), goal)
                        # Solo reenviar objetivos de robots realmente fuera de la tolerancia.
                        # No tocamos la ruta ni la sincronización de pasos CBS.
                        if d > self.arrival_tolerance:
                            self.publish(a, goal)
                            self.node.get_logger().warn(
                                f'ALIGN REENVIO {self.align_retries}/{self.max_retries}: '
                                f'{a} d={d:.3f}m -> {goal}')
                else:
                    self.node.get_logger().warn('ALIGN: límite de reenvíos alcanzado; esperando barrera o timeout')
        super().tick()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--previous-state', default='/tmp/mision_fisica_ronda1_estado.json')
    ap.add_argument('--step-timeout', type=float, default=90.0)
    ap.add_argument('--retry-interval', type=float, default=3.0)
    ap.add_argument('--max-retries', type=int, default=5)
    ap.add_argument('--status-interval', type=float, default=2.0)
    ap.add_argument('--max-ct', type=int, default=20000)
    ap.add_argument('--max-time', type=int, default=65)
    args, ros_args = ap.parse_known_args()
    if args.retry_interval < 1.0 or not 0 <= args.max_retries <= 10 or args.status_interval < 0.5:
        ap.error('Intervalos y número de reintentos fuera de rangos seguros')
    prev = previous_names(args.previous_state, args.scenario)
    rclpy.init(args=ros_args)
    exe = None
    try:
        exe = Round2PickupDiagnostic(args.scenario, prev, step_timeout=args.step_timeout,
                                    retry_interval=args.retry_interval,
                                    max_retries=args.max_retries,
                                    status_interval=args.status_interval,
                                    max_ct=args.max_ct, max_time=args.max_time)
        while rclpy.ok() and exe.state not in ('DONE', 'FAILED'):
            rclpy.spin_once(exe.node, timeout_sec=0.2)
        return 0 if exe.state == 'DONE' else 1
    except KeyboardInterrupt:
        return 130
    finally:
        if exe is not None:
            exe.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
