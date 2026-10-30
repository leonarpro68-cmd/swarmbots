#!/usr/bin/env python3
"""D8.7: verify the ACTUAL Gazebo world odometry before starting CBS.

No motion commands are ever published. Requires ROS 2 Jazzy + nav_msgs.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import time

ROBOTS = ('summit0', 'summit1', 'summit2')


def expected_from_json(data):
    if data.get('version') != 1:
        raise ValueError('Version de escenario inesperada')
    robots = data.get('robots', [])
    positions = {entry['name']: tuple(float(x) for x in entry['xy_m']) for entry in robots}
    if len(robots) != 3 or set(positions) != set(ROBOTS):
        raise ValueError('El escenario debe incluir tres poses iniciales de Summit')
    if not all(len(p) == 2 and all(math.isfinite(x) for x in p) for p in positions.values()):
        raise ValueError('Posiciones no finitas o mal formadas')
    return positions


def validate_positions(expected, observed, max_error=0.25, max_speed=0.10):
    """Pure logic; observed[name] contains xy, frame, speed, received."""
    details, problems = {}, []
    for name in ROBOTS:
        item = observed.get(name)
        if item is None:
            problems.append(f'{name}: odometria ausente')
            continue
        pose = tuple(item['xy'])
        distance = math.dist(expected[name], pose)
        details[name] = {
            'expected_xy': list(expected[name]), 'observed_xy': list(pose),
            'distance_m': round(distance, 6), 'speed_m_s': round(item['speed'], 6),
            'frame': str(item['frame']),
        }
        if not math.isfinite(distance) or distance > max_error:
            problems.append(f'{name}: desplazamiento inicial {distance:.3f} m > {max_error:.3f} m')
        if item['frame'] != f'{name}/odom':
            problems.append(f'{name}: frame {item["frame"]!r} no es {name}/odom')
        if not math.isfinite(item['speed']) or item['speed'] > max_speed:
            problems.append(f'{name}: velocidad inicial {item["speed"]:.3f} m/s > {max_speed:.3f} m/s')
    return details, problems


def verify_live(expected, timeout=22.0, max_error=0.25):
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry

    class Snap(Node):
        def __init__(self):
            super().__init__('d8_verify_scenario_spawns')
            self.observed = {}
            for robot in ROBOTS:
                self.create_subscription(Odometry, f'/{robot}/odom',
                                         lambda m, robot=robot: self.cb(robot, m), 10)

        def cb(self, robot, m):
            self.observed[robot] = {
                'xy': (float(m.pose.pose.position.x), float(m.pose.pose.position.y)),
                'frame': str(m.header.frame_id),
                'speed': math.hypot(float(m.twist.twist.linear.x), float(m.twist.twist.linear.y)),
                'received': time.monotonic(),
            }

    rclpy.init()
    node = Snap()
    started = time.monotonic()
    stable_since = None
    details, issues = {}, ['Esperando odometrias']
    try:
        while time.monotonic() - started < timeout:
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()
            observed = {n: v for n, v in node.observed.items() if now - v['received'] < 1.5}
            details, issues = validate_positions(expected, observed, max_error)
            if not issues:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= 0.5:
                    return details, []
            else:
                stable_since = None
        return details, issues
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scenario', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--max-error', type=float, default=0.25)
    p.add_argument('--timeout', type=float, default=22)
    a = p.parse_args()
    if not 0 < a.max_error <= 0.75 or a.timeout <= 0:
        p.error('Parametro de validacion fuera de rango')
    data = json.loads(a.scenario.read_text(encoding='utf-8'))
    expected = expected_from_json(data)
    details, issues = verify_live(expected, a.timeout, a.max_error)
    result = {'seed': data['seed'], 'status': 'PASS' if not issues else 'FAIL',
              'max_error_m': a.max_error, 'robots': details, 'problems': issues}
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    if issues:
        print('[D8] SPAWN INVALIDO:', '; '.join(issues), 'Ver', a.output, flush=True)
        return 2
    print('[D8] SPAWN OK: las tres odometrias coinciden con scenario.json (<=%.2f m)' % a.max_error,
          flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
