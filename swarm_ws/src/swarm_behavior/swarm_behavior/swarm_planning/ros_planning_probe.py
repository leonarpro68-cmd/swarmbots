#!/usr/bin/env python3
"""Etapa 6A: prueba ROS 2 sin actuación; odometría -> CBS asíncrono -> JSON.

NO publica Path ni cmd_vel, NO elimina modelos de Gazebo, NO ejecuta la misión.
Uso: python3 ros_planning_probe.py --scenario /ruta/escenario_actual.json
"""
from __future__ import annotations
import argparse
import json
import time
from pathlib import Path
from .bridge import PlanningBridge
from .mission_core import MissionPlanningCoordinator
from .ros_plan_helpers import AGENTS, RobotSample, snapshot, planning_starts


def plan_report(result):
    return {
        'status': 'planned_only',
        'starts': [list(p) for p in result.starts],
        'assignments': [{'robot': AGENTS[i], 'trash': name, 'euclidean_m': dist}
                        for i, (name, dist) in enumerate(result.assigned)],
        'paths': [[list(v) for v in route] for route in result.paths],
        'cost': result.cost, 'ct_expanded': result.expanded,
        'ct_generated': result.generated,
    }


class PlanningProbe:
    def __init__(self, scenario: str, output: str, odom_timeout: float = 2., max_ct: int = 20000, max_time: int = 65):
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        self.node = Node('swarm_planning_probe')
        self.coordinator = MissionPlanningCoordinator(
            PlanningBridge.from_json(scenario), max_ct=max_ct, max_time=max_time)
        self.samples = {}
        self.subs = []
        self.output = Path(output)
        self.timeout = odom_timeout
        self.state = 'WAIT_ODOM'
        self.submitted_at = None
        for a in AGENTS:
            self.subs.append(self.node.create_subscription(
                Odometry, f'/{a}/odom', lambda msg, name=a: self.on_odom(name, msg), 10))
        self.timer = self.node.create_timer(.1, self.tick)
        self.node.get_logger().info('6A: esperando odometría; esta prueba NO moverá los robots')

    def on_odom(self, agent, msg):
        p = msg.pose.pose.position
        self.samples[agent] = RobotSample(float(p.x), float(p.y), time.monotonic(), msg.header.frame_id)

    def tick(self):
        if self.state in ('DONE', 'FAILED'):
            return
        if self.state == 'WAIT_ODOM':
            valid = snapshot(self.samples, self.timeout)
            if valid is None:
                return
            try:
                starts = planning_starts(self.coordinator.bridge.graph, valid)
                self.coordinator.submit_pickup(starts)
            except (RuntimeError, ValueError) as exc:
                self.fail(str(exc))
                return
            self.submitted_at = time.monotonic()
            self.state = 'PLANNING'
            self.node.get_logger().info(f'CBS asíncrono iniciado; vértices de inicio={starts}')
        elif self.state == 'PLANNING':
            result = self.coordinator.poll()
            if result is None:
                return  # odom sigue procesándose en los callbacks ROS
            if result.error is not None:
                self.fail(f'CBS falló: {result.error}')
                return
            if result.kind != 'pickup':
                self.fail(f'Resultado inesperado: {result.kind}')
                return
            try:
                payload = plan_report(result.value)
                self.output.parent.mkdir(parents=True, exist_ok=True)
                self.output.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
            except (OSError, ValueError, TypeError) as exc:
                self.fail(f'No se pudo guardar resultado: {exc}')
                return
            self.state = 'DONE'
            fresh = snapshot(self.samples, self.timeout) is not None
            self.node.get_logger().info(
                f'CBS completado: costo={payload["cost"]}, CT={payload["ct_expanded"]}, '
                f'odom_actual={fresh}; reporte={self.output}; SIN MOVIMIENTO')

    def fail(self, message):
        self.state = 'FAILED'
        self.node.get_logger().error(message)

    def close(self):
        self.coordinator.close()
        self.node.destroy_node()


def main():
    import rclpy
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--output', default='/tmp/swarm_plan_probe.json')
    ap.add_argument('--odom-timeout', type=float, default=2.)
    ap.add_argument('--max-ct', type=int, default=20000)
    ap.add_argument('--max-time', type=int, default=65)
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    probe = None
    try:
        probe = PlanningProbe(args.scenario, args.output, args.odom_timeout, args.max_ct, args.max_time)
        while rclpy.ok() and probe.state not in ('DONE', 'FAILED'):
            rclpy.spin_once(probe.node, timeout_sec=0.2)
    finally:
        if probe is not None:
            probe.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
