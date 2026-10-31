#!/usr/bin/env python3
"""Inspección de solo lectura de los 3 seguidores, TF y gestores de agarre."""
import argparse
import json
import re
import subprocess
import sys


def ros2(*args):
    cp = subprocess.run(['ros2', *args], capture_output=True, text=True,
                        timeout=12, check=False)
    if cp.returncode != 0:
        raise RuntimeError('ros2 ' + ' '.join(args) + ': ' + cp.stderr.strip())
    return cp.stdout


def publishers_and_subscribers(topic):
    out = ros2('topic', 'info', topic, '--verbose')
    def count(s):
        m = re.search(rf'^{re.escape(s)} count:\s*(\d+)', out, re.M)
        return int(m.group(1)) if m else None
    return {'publishers': count('Publisher'),
            'subscribers': count('Subscription'), 'detail': out}


def main():
    p = argparse.ArgumentParser(description='Preflight del soporte; no mueve los robots')
    p.add_argument('--n-robots', type=int, default=3, choices=[1, 2, 3])
    p.add_argument('--output', default='/tmp/supermarket_support_graph.json')
    a = p.parse_args()
    node_lines = ros2('node', 'list').splitlines()
    nodes = set(node_lines)
    checks = {}
    data = {}
    for i in range(a.n_robots):
        ns = f'summit{i}'
        for name in (f'/{ns}/path_follower', f'/{ns}/visual_grasp',
                     f'/grasp_manager_{ns}', f'/relay_tf_{ns}'):
            checks[f'node:{name}'] = node_lines.count(name) == 1
        command = publishers_and_subscribers(f'/{ns}/cmd_vel')
        grasp = publishers_and_subscribers(f'/{ns}/gripper/grasp')
        release = publishers_and_subscribers(f'/{ns}/gripper/release')
        tf_in = publishers_and_subscribers(f'/{ns}/tf')
        checks[f'{ns}:single_cmd_vel_owner'] = command['publishers'] == 1
        checks[f'{ns}:grasp_subscriber'] = grasp['subscribers'] == 1
        checks[f'{ns}:release_subscriber'] = release['subscribers'] == 1
        checks[f'{ns}:tf_source'] = (tf_in['publishers'] or 0) >= 1
        data[ns] = {
            'cmd_vel_publishers': command['publishers'],
            'grasp_subscribers': grasp['subscribers'],
            'release_subscribers': release['subscribers'],
            'tf_source_publishers': tf_in['publishers'],
        }
    out = {'status': 'PASS_SUPPORT_GRAPH_NOT_EXECUTION_AUTHORIZATION' if all(checks.values())
           else 'BLOCKED', 'checks': checks, 'robots': data,
           'note': 'Solo verifica el grafo ROS; para TF real, odometría y poses use ros_observer_readonly.py'}
    with open(a.output, 'w', encoding='utf-8') as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(json.dumps(out, ensure_ascii=False, indent=2))
    return 0 if all(checks.values()) else 2


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (RuntimeError, subprocess.TimeoutExpired) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        sys.exit(2)
