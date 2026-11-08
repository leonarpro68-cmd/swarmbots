#!/usr/bin/env python3
"""Etapa 7B.2: genera escenario, espera odometria e inserta objetos; NO inicia CBS."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time


def validate_scenario(path):
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    objects = data.get('elements', [])
    if data.get('version') != 1 or len(objects) != 9:
        raise ValueError('Formato de escenario inesperado; se requieren 9 objetos')
    if sum(e.get('type') == 'trash' for e in objects) != 6:
        raise ValueError('El escenario debe tener exactamente 6 basuras')
    if sum(e.get('type') == 'deposit' for e in objects) != 3:
        raise ValueError('El escenario debe tener exactamente 3 depósitos')
    if len({e.get('name') for e in objects}) != 9:
        raise ValueError('Los objetos deben tener nombres únicos')
    if {r.get('name') for r in data.get('robots', [])} != {'summit0', 'summit1', 'summit2'}:
        raise ValueError('Se requieren tres Summit')
    return data


def wait_recent_odometry(timeout_s):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    rclpy.init()
    node = rclpy.create_node('swarm_scenario_readiness')
    recent = {}
    subs = []
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    try:
        for i in range(3):
            name = f'summit{i}'
            subs.append(node.create_subscription(
                Odometry, f'/{name}/odom',
                lambda _msg, robot=name: recent.__setitem__(robot, time.monotonic()), qos))
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()
            if len(recent) == 3 and all(now - t < 2.0 for t in recent.values()):
                print('[7B.2] Odometría reciente de los tres Summit: OK', flush=True)
                return
        raise TimeoutError(f'Odometría no disponible para todos los Summit: {sorted(recent)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


def run_checked(argv, timeout):
    print('[7B.2] Ejecutando:', ' '.join(map(str, argv)), flush=True)
    subprocess.run([str(s) for s in argv], check=True, timeout=timeout)


def prepare(args):
    scripts = Path(args.scripts_dir).expanduser().resolve()
    generator = scripts / 'escenario_aleatorio_swarm.py'
    placer = scripts / 'usar_escenario_swarm.py'
    if not generator.is_file() or not placer.is_file():
        raise FileNotFoundError(f'Faltan scripts de escenario en {scripts}')
    scenario = Path(args.scenario).expanduser().resolve()
    scenario.parent.mkdir(parents=True, exist_ok=True)
    if scenario.exists():
        scenario.unlink()  # impedir confundir archivo previo con una generación nueva
    run_checked([sys.executable, generator, '--seed', args.seed, '--output', scenario], timeout=180)
    data = validate_scenario(scenario)
    print(f'[7B.2] Escenario validado: semilla={data["seed"]}, 6 basuras, 3 depósitos', flush=True)
    wait_recent_odometry(args.ready_timeout)
    run_checked([sys.executable, placer, '--scenario', scenario, 'place'], timeout=480)
    print('[7B.2] COLOCACIÓN SOLICITADA: comprueba visualmente los 9 objetos en Gazebo.', flush=True)
    print('[7B.2] SIN CBS: los tres robots deben permanecer quietos.', flush=True)
    print(f'[7B.2] Escenario JSON: {scenario}', flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--scenario', default='/tmp/swarm_cbs_escenario.json')
    ap.add_argument('--scripts-dir', default='~/swarm_ws/scripts')
    ap.add_argument('--ready-timeout', type=float, default=180)
    args = ap.parse_args()
    try:
        prepare(args)
        return 0
    except Exception as exc:
        print(f'[7B.2] ERROR: {exc}', file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    sys.exit(main())
