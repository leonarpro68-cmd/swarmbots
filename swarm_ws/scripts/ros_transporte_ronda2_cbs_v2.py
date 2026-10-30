#!/usr/bin/env python3
"""Transporte CBS ronda 2 con DDS/reintentos del transporte V2 ya probado.

No modifica CBS ni Gazebo; requiere tres cargas físicas confirmadas.
"""
import argparse
import rclpy
from ros_recogida_ronda2_cbs import previous_names
from ros_transporte_fisico_cbs_v2 import PhysicalDeliveryNavigationV2


class Round2DeliveryV2(PhysicalDeliveryNavigationV2):
    def __init__(self, scenario, carried, previous, **kwargs):
        if set(carried) & set(previous):
            raise ValueError('No volver a transportar basura de la ronda 1')
        super().__init__(scenario, carried, **kwargs)
        for name in previous:
            self.coordinator.bridge.confirm_collected(name)
        if self.coordinator.pending:
            raise ValueError(f'Siguen pendientes basuras: {self.coordinator.pending}')
        self.node.get_logger().info(f'RONDA 2 V2: mapa actualizado con previas={previous}')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--previous-state', required=True)
    ap.add_argument('--carried', nargs=3, required=True)
    ap.add_argument('--output', default='/tmp/swarm_transporte_ronda2_v2.json')
    ap.add_argument('--step-timeout', type=float, default=90.0)
    ap.add_argument('--discovery-timeout', type=float, default=20.0)
    ap.add_argument('--diagnostic-interval', type=float, default=2.0)
    ap.add_argument('--retry-interval', type=float, default=5.0)
    ap.add_argument('--max-retries', type=int, default=3)
    ap.add_argument('--max-ct', type=int, default=20000)
    ap.add_argument('--max-time', type=int, default=65)
    args, ros_args = ap.parse_known_args()
    if (args.step_timeout < 20 or args.discovery_timeout < 3
            or args.diagnostic_interval < .5 or args.retry_interval < 3
            or not 0 <= args.max_retries <= 5):
        ap.error('Parámetros de recuperación fuera de los límites de prueba')
    prev = previous_names(args.previous_state, args.scenario)
    rclpy.init(args=ros_args)
    exe = None
    try:
        exe = Round2DeliveryV2(
            args.scenario, args.carried, prev, output=args.output,
            step_timeout=args.step_timeout, discovery_timeout=args.discovery_timeout,
            diagnostic_interval=args.diagnostic_interval, retry_interval=args.retry_interval,
            max_retries=args.max_retries, max_ct=args.max_ct, max_time=args.max_time,
        )
        while rclpy.ok() and exe.state not in ('DONE', 'FAILED'):
            rclpy.spin_once(exe.node, timeout_sec=.2)
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
