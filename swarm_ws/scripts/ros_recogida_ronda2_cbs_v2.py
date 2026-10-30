#!/usr/bin/env python3
"""Recogida CBS ronda 2: reenvío limitado para ALIGN y STEP_n.

Conserva la ruta temporal CBS y la barrera original: solo reenvía el
mismo objetivo del paso actual al robot estancado. NO ejecuta agarres.
"""
import argparse
import math
import time

import rclpy
from ros_recogida_ronda2_cbs import Round2Pickup, previous_names
from swarm_behavior.swarm_planning.ros_pickup_executor import AGENTS


class Round2PickupDiagnosticV3(Round2Pickup):
    def __init__(self, scenario, previous, *, retry_interval=3.0,
                 max_retries=5, status_interval=2.0, stall_seconds=2.0,
                 progress_distance=0.03, **kwargs):
        self.retry_interval = float(retry_interval)
        self.max_retries = int(max_retries)
        self.status_interval = float(status_interval)
        self.stall_seconds = float(stall_seconds)
        self.progress_distance = float(progress_distance)
        self.retry_counts = {}
        self.last_retry = {}
        self.last_progress = {}
        self.last_dist = {}
        self.last_status = 0.0
        super().__init__(scenario, previous, **kwargs)
        self.node.get_logger().info(
            'R2 V3: diagnóstico/reenvío limitado por robot en ALIGN y STEP_n; '
            'misma meta y barrera CBS, sin saltar pasos')

    def start_barrier(self, name, targets):
        # Se reinicia al iniciar CADA barrera, no durante los reintentos.
        super().start_barrier(name, targets)
        now = time.monotonic()
        self.retry_counts = {a: 0 for a in AGENTS}
        self.last_retry = {a: now for a in AGENTS}
        self.last_progress = {a: now for a in AGENTS}
        self.last_dist = {}
        self.last_status = 0.0
        for a in AGENTS:
            sample = self.samples.get(a)
            if sample is not None:
                self.last_dist[a] = math.dist(
                    (sample.x, sample.y), self.barrier.targets[a])

    def tick(self):
        if self.state == 'MOVING' and self.barrier is not None:
            now = time.monotonic()
            barrier = self.barrier
            data = []
            for a in AGENTS:
                sample = self.samples.get(a)
                if sample is None:
                    data.append(f'{a}: sin odometría')
                    continue
                d = math.dist((sample.x, sample.y), barrier.targets[a])
                speed = self.velocities.get(a, float('inf'))
                prev = self.last_dist.get(a, d)
                # Al acercarse, reanudar temporizador de estancamiento.
                # Si se aleja sustancialmente, también se evita un reenvío inmediato.
                if abs(prev - d) >= self.progress_distance:
                    self.last_dist[a] = d
                    self.last_progress[a] = now
                age = now - sample.received_at
                data.append(
                    f'{a}: d={d:.3f}m, v={speed:.3f}m/s, '
                    f'odom_age={age:.2f}s, retries={self.retry_counts[a]}')
                if age > self.odom_timeout:
                    continue  # el ejecutor base tiene autoridad para fallar
                if d <= self.arrival_tolerance or speed > self.arrival_speed:
                    continue
                if now - self.last_progress[a] < self.stall_seconds:
                    continue
                if now - self.last_retry[a] < self.retry_interval:
                    continue
                if self.retry_counts[a] >= self.max_retries:
                    continue
                self.retry_counts[a] += 1
                self.last_retry[a] = now
                # Nunca se transmite el siguiente paso ni se altera CBS.
                self.publish(a, barrier.targets[a])
                self.node.get_logger().warn(
                    f'R2_V3 REENVIO {barrier.name} {a} '
                    f'{self.retry_counts[a]}/{self.max_retries}: '
                    f'd={d:.3f}m, v={speed:.3f}m/s -> {barrier.targets[a]}')

            if now - self.last_status >= self.status_interval:
                self.last_status = now
                self.node.get_logger().info(
                    f'R2_V3 DIAGNOSTICO {barrier.name} | ' + ' | '.join(data))
        super().tick()  # Único responsable de validar barrera / timeout / avanzar CBS


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--previous-state', default='/tmp/mision_fisica_ronda1_estado.json')
    ap.add_argument('--step-timeout', type=float, default=90.0)
    ap.add_argument('--retry-interval', type=float, default=3.0)
    ap.add_argument('--max-retries', type=int, default=5)
    ap.add_argument('--status-interval', type=float, default=2.0)
    ap.add_argument('--stall-seconds', type=float, default=2.0)
    ap.add_argument('--progress-distance', type=float, default=0.03)
    ap.add_argument('--max-ct', type=int, default=20000)
    ap.add_argument('--max-time', type=int, default=65)
    args, ros_args = ap.parse_known_args()
    if (args.retry_interval < 1.0 or not 0 <= args.max_retries <= 10
            or args.status_interval < 0.5 or args.stall_seconds < 1.0
            or not 0.01 <= args.progress_distance <= 0.20):
        ap.error('Valores de reintento/diagnóstico fuera de rangos seguros')
    previous = previous_names(args.previous_state, args.scenario)
    rclpy.init(args=ros_args)
    exe = None
    try:
        exe = Round2PickupDiagnosticV3(
            args.scenario, previous, step_timeout=args.step_timeout,
            retry_interval=args.retry_interval, max_retries=args.max_retries,
            status_interval=args.status_interval,
            stall_seconds=args.stall_seconds,
            progress_distance=args.progress_distance,
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
