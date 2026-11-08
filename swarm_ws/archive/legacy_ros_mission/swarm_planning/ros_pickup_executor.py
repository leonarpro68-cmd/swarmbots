#!/usr/bin/env python3
"""6B: recogida SOLO aproximación, sin eliminar modelos y sin realizar entrega.

Uso: python3 -m swarm_behavior.swarm_planning.ros_pickup_executor --scenario /path/escenario_actual.json
Necesita tres path_follower activos y los objetos ya colocados.
"""
from __future__ import annotations
import argparse
import math
import time
from dataclasses import dataclass
from .bridge import PlanningBridge
from .mission_core import MissionPlanningCoordinator
from .ros_plan_helpers import AGENTS, RobotSample, snapshot, planning_starts
from .geometry import node_position


@dataclass
class Barrier:
    name: str
    targets: dict
    since: float
    holds: dict


def padded_paths(paths):
    paths = tuple(tuple(p) for p in paths)
    if not paths or any(not p for p in paths):
        raise ValueError('Rutas vacías')
    n = max(map(len, paths))
    return tuple(p + (p[-1],) * (n - len(p)) for p in paths)


def barrier_complete(barrier, samples, velocities, now, tol=0.12, speed_limit=0.10, hold=0.6):
    """Comprueba llegadas estables; opera sin ROS para facilitar pruebas."""
    for a in AGENTS:
        x, y = samples[a].x, samples[a].y
        gx, gy = barrier.targets[a]
        ok = math.dist((x, y), (gx, gy)) <= tol and velocities[a] <= speed_limit
        barrier.holds[a] = barrier.holds[a] if ok and barrier.holds[a] is not None else now if ok else None
    return all(barrier.holds[a] is not None and now - barrier.holds[a] >= hold for a in AGENTS)


class PickupExecutor:
    def __init__(self, scenario, odom_timeout=2., step_timeout=60., arrival_tolerance=.12,
                 arrival_speed=.10, hold_seconds=.6, max_ct=20000, max_time=65):
        from rclpy.node import Node
        from nav_msgs.msg import Odometry, Path
        from geometry_msgs.msg import PoseStamped
        self.node = Node('swarm_refactor_pickup')
        self.Path, self.PoseStamped = Path, PoseStamped
        self.coordinator = MissionPlanningCoordinator(PlanningBridge.from_json(scenario), max_ct=max_ct, max_time=max_time)
        self.samples, self.velocities = {}, {}
        self.subs = [self.node.create_subscription(Odometry, f'/{a}/odom', lambda m, name=a: self.on_odom(name, m), 10) for a in AGENTS]
        self.pubs = {a: self.node.create_publisher(Path, f'/{a}/planned_path', 10) for a in AGENTS}
        self.odom_timeout, self.step_timeout = odom_timeout, step_timeout
        self.arrival_tolerance, self.arrival_speed, self.hold_seconds = arrival_tolerance, arrival_speed, hold_seconds
        self.state = 'WAIT_ODOM'
        self.barrier = None
        self.routes = ()
        self.step = 0
        self.result = None
        self.node.create_timer(.10, self.tick)
        self.node.get_logger().info('6B: esperando odometría. Solo aproximación, SIN eliminar basuras')

    def on_odom(self, agent, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.samples[agent] = RobotSample(float(p.x), float(p.y), time.monotonic(), msg.header.frame_id)
        self.velocities[agent] = math.hypot(float(v.x), float(v.y))

    def fresh(self):
        return snapshot(self.samples, self.odom_timeout)

    def publish(self, agent, target):
        p = self.Path()
        p.header.frame_id = self.samples[agent].frame_id or 'odom'
        p.header.stamp = self.node.get_clock().now().to_msg()
        v = self.PoseStamped()
        v.header = p.header
        v.pose.position.x, v.pose.position.y = map(float, target)
        v.pose.orientation.w = 1.
        p.poses = [v]
        self.pubs[agent].publish(p)

    def stop(self):
        for a in AGENTS:
            p = self.Path()
            if a in self.samples:
                p.header.frame_id = self.samples[a].frame_id
            self.pubs[a].publish(p)

    def fail(self, message):
        self.node.get_logger().error(message)
        self.stop()
        self.state = 'FAILED'

    def start_barrier(self, name, targets):
        self.barrier = Barrier(name, dict(targets), time.monotonic(), {a: None for a in AGENTS})
        for a in AGENTS:
            self.publish(a, targets[a])
            self.node.get_logger().info(f'{name}: {a} -> {targets[a]}')
        self.state = 'MOVING'

    def tick(self):
        if self.state in ('DONE', 'FAILED'):
            return
        if self.state == 'WAIT_ODOM':
            valid = self.fresh()
            if valid is None:
                return
            try:
                starts = planning_starts(self.coordinator.bridge.graph, valid)
                self.coordinator.submit_pickup(starts)
                self.state = 'PLANNING'
                self.node.get_logger().info(f'CBS asíncrono: inicios={starts}')
            except (RuntimeError, ValueError) as exc:
                self.fail(f'No se pudo iniciar CBS: {exc}')
        elif self.state == 'PLANNING':
            outcome = self.coordinator.poll()
            if outcome is None:
                return
            if outcome.error is not None or outcome.kind != 'pickup':
                self.fail(f'CBS falló: {outcome.error or outcome.kind}')
                return
            self.result = outcome.value
            self.routes = padded_paths(self.result.paths)
            self.node.get_logger().info(f'CBS OK: costo={self.result.cost}, CT={self.result.expanded}, pasos={len(self.routes[0])-1}')
            for i, a in enumerate(AGENTS):
                self.node.get_logger().info(f'{a}: basura={self.result.assigned[i][0]}, meta={self.routes[i][-1]}')
            # CBS puede tardar; no comprobar frescura hasta recibir nuevos mensajes.
            self.state = 'REFRESH_ODOM'
        elif self.state == 'REFRESH_ODOM':
            if self.fresh() is None:
                return
            # Si los robots se movieron mientras CBS trabajaba, cancelar el plan obsoleto.
            for i, a in enumerate(AGENTS):
                current = self.samples[a]
                aligned = node_position(self.routes[i][0])
                if math.dist((current.x, current.y), aligned) > 2.5:
                    self.fail(f'{a} se desplazó demasiado durante la planificación')
                    return
            self.start_barrier('ALIGN', {a: node_position(self.routes[i][0]) for i, a in enumerate(AGENTS)})
        elif self.state == 'MOVING':
            valid = self.fresh()
            if valid is None:
                self.fail('Odometría vencida durante navegación')
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
                self.state = 'DONE'
                self.node.get_logger().info('6B COMPLETADA: tres Summit junto a sus basuras; NO eliminadas')
                return
            waits = [a for i, a in enumerate(AGENTS) if self.routes[i][self.step] == self.routes[i][self.step-1]]
            self.node.get_logger().info(f'STEP_{self.step}; WAIT={waits}')
            self.start_barrier(f'STEP_{self.step}',
                               {a: node_position(self.routes[i][self.step]) for i, a in enumerate(AGENTS)})

    def close(self):
        self.stop()
        self.coordinator.close()
        self.node.destroy_node()


def main():
    import rclpy
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scenario', required=True)
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
        executor = PickupExecutor(args.scenario, args.odom_timeout, args.step_timeout,
                                  args.arrival_tolerance, args.arrival_speed, args.hold_seconds,
                                  args.max_ct, args.max_time)
        while rclpy.ok() and executor.state not in ('DONE', 'FAILED'):
            rclpy.spin_once(executor.node, timeout_sec=.2)
    except KeyboardInterrupt:
        pass
    finally:
        if executor:
            executor.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
