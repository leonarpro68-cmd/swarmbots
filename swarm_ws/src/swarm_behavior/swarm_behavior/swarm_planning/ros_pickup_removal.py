#!/usr/bin/env python3
"""6C: aproximación CBS (6B) + retirada asíncrona de tres basuras.

Sin entrega ni segunda ronda. Requiere Gazebo, path_followers y escenario compartido.
"""
from __future__ import annotations

import argparse
import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from .gazebo_removal import remove_batch
from .ros_pickup_executor import PickupExecutor


class PickupRemovalExecutor(PickupExecutor):
    """Extensión temporal y acotada de 6B; sin bloquear callbacks de odometría."""
    def __init__(self, scenario, *, world=None, state_file='/tmp/swarm_refactor_6c.json', **kwargs):
        self.world = world
        self.state_file = Path(state_file)
        self.removal_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='gazebo_remove')
        self.removal_future = None
        super().__init__(scenario, **kwargs)
        self.node.get_logger().info('6C: aproximación y retirada confirmada; SIN entrega')

    def tick(self):
        if self.state == 'REMOVING':
            self._poll_removal()
            return
        if self.state in ('FAILED', 'REMOVAL_FAILED', 'PICKUP_DONE'):
            return
        super().tick()
        if self.state == 'DONE':
            names = tuple(name for name, _ in self.result.assigned)
            if len(names) != 3 or len(set(names)) != len(names):
                self.fail('Asignación de basuras inconsistente')
                return
            self.stop()
            self.removal_future = self.removal_pool.submit(remove_batch, names, self.world)
            self.state = 'REMOVING'
            self.node.get_logger().info(f'6C: retirada solicitada en segundo plano: {names}')

    def _poll_removal(self):
        if self.removal_future is None or not self.removal_future.done():
            return
        try:
            result = self.removal_future.result()
        except Exception as exc:
            self.fail(f'No se pudo consultar/eliminar en Gazebo: {exc}')
            return
        self.node.get_logger().info(f'Servicio Gazebo: {result.service}')
        # Actualizar el grafo exclusivamente en el hilo del ejecutor ROS,
        # tras obtener confirmación de cada eliminación.
        if result.confirmed:
            try:
                self.coordinator.confirm_pickups(result.confirmed)
            except Exception as exc:
                self.fail(f'Gazebo confirmó eliminaciones, pero falló la actualización del grafo: {exc}')
                return
        for name in result.confirmed:
            self.node.get_logger().info(f'RECOGIDA CONFIRMADA: {name}')
        for name, reason in result.failures.items():
            self.node.get_logger().error(f'RECOGIDA NO CONFIRMADA: {name}: {reason}')
        payload = {
            'confirmed': list(result.confirmed),
            'failures': result.failures,
            'pending': sorted(self.coordinator.pending),
            'vertices_free': len(self.coordinator.bridge.graph.vertices),
            'edges_free': len(self.coordinator.bridge.graph.edges),
            'note': '6C: entrega todavía no implementada',
        }
        try:
            self.state_file.parent.mkdir(parents=True, exist_ok=True)
            self.state_file.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
            self.node.get_logger().info(f'Estado guardado: {self.state_file}')
        except OSError as exc:
            self.node.get_logger().error(f'Error escribiendo estado: {exc}')
        if result.failures or len(result.confirmed) != 3:
            self.state = 'REMOVAL_FAILED'
            self.node.get_logger().error('6C INCOMPLETA: no iniciar entrega; comprobar estado de Gazebo')
        else:
            self.state = 'PICKUP_DONE'
            self.node.get_logger().info('6C COMPLETADA: 3 basuras eliminadas, grafo actualizado; SIN entrega')

    def close(self):
        super().close()
        self.removal_pool.shutdown(wait=False, cancel_futures=True)


def main():
    import rclpy
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--world', default=None)
    ap.add_argument('--state-file', default='/tmp/swarm_refactor_6c.json')
    ap.add_argument('--odom-timeout', type=float, default=2.)
    ap.add_argument('--step-timeout', type=float, default=60.)
    ap.add_argument('--arrival-tolerance', type=float, default=.12)
    ap.add_argument('--arrival-speed', type=float, default=.10)
    ap.add_argument('--hold-seconds', type=float, default=.6)
    ap.add_argument('--max-ct', type=int, default=20000)
    ap.add_argument('--max-time', type=int, default=65)
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    exe = None
    try:
        exe = PickupRemovalExecutor(
            args.scenario, world=args.world, state_file=args.state_file,
            odom_timeout=args.odom_timeout, step_timeout=args.step_timeout,
            arrival_tolerance=args.arrival_tolerance, arrival_speed=args.arrival_speed,
            hold_seconds=args.hold_seconds, max_ct=args.max_ct, max_time=args.max_time)
        while rclpy.ok() and exe.state not in ('PICKUP_DONE', 'REMOVAL_FAILED', 'FAILED'):
            rclpy.spin_once(exe.node, timeout_sec=.2)
    except KeyboardInterrupt:
        pass
    finally:
        if exe is not None:
            exe.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
