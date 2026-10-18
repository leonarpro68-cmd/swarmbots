#!/usr/bin/env python3
"""CBS entrega ronda 2: grafo sin las basuras recogidas en ambas rondas.

No modifica Gazebo, ni manda detach. Los tres objetos de ronda 2 ya deben estar sujetos.
"""
import argparse
import rclpy
from ros_recogida_ronda2_cbs import previous_names
from ros_transporte_fisico_cbs import PhysicalDeliveryNavigation


class Round2Delivery(PhysicalDeliveryNavigation):
    def __init__(self, scenario, carried, previous, **kwargs):
        if set(carried) & set(previous):
            raise ValueError('No transportar otra vez basura de ronda 1')
        super().__init__(scenario, carried, **kwargs)
        for name in previous:
            self.coordinator.bridge.confirm_collected(name)
        if self.coordinator.pending:
            raise ValueError(f'Deben estar recogidas las seis basuras; pendientes={self.coordinator.pending}')
        self.node.get_logger().info(f'RONDA 2 TRANSPORTE: mapa actualizado con basuras previas {previous}')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--previous-state', required=True)
    ap.add_argument('--carried', required=True, nargs=3)
    ap.add_argument('--output', default='/tmp/swarm_transporte_ronda2.json')
    ap.add_argument('--step-timeout', type=float, default=90.)
    ap.add_argument('--max-ct', type=int, default=20000)
    ap.add_argument('--max-time', type=int, default=65)
    args, ros_args = ap.parse_known_args()
    prev = previous_names(args.previous_state, args.scenario)
    rclpy.init(args=ros_args)
    exe = None
    try:
        exe = Round2Delivery(args.scenario, args.carried, prev, output=args.output,
                             step_timeout=args.step_timeout, max_ct=args.max_ct, max_time=args.max_time)
        while rclpy.ok() and exe.state not in ('DONE','FAILED'):
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
