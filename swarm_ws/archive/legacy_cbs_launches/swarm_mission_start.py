#!/usr/bin/env python3
"""7B.3: reuse successful 7B.2 preparation; then launch previously validated 6E."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import re


def gazebo_world_name(sdf_file):
    """Obtiene el nombre interno del mundo desde el archivo SDF."""
    content = Path(sdf_file).read_text(encoding='utf-8')

    match = re.search(
        r'<world\b[^>]*\bname\s*=\s*["\']([^"\']+)["\']',
        content
    )

    if not match:
        raise ValueError(
            f'No se encontró el nombre del mundo en {sdf_file}'
        )

    return match.group(1)


def check_mission_state(path):
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    delivered = data.get('delivered', [])
    if data.get('status') != 'completed' or len(delivered) != 6 or len(set(delivered)) != 6:
        raise RuntimeError(f'Misión incompleta: status={data.get("status")}, entregadas={len(delivered)}')
    return data


def orchestrate(args, *, share_dir, run=subprocess.run):
    scenario = Path(args.scenario).expanduser().resolve()
    state_file = Path(args.mission_state_file).expanduser().resolve()
    worlds_dir = Path(share_dir) / 'worlds'
    world_file = worlds_dir / (args.world_file + '.sdf')
    if not world_file.is_file():
        raise FileNotFoundError(f'No existe el SDF solicitado: {world_file}')
    actual_world = gazebo_world_name(world_file)
    print(f'[7B.3] Archivo SDF: {args.world_file}.sdf; mundo Gazebo: {actual_world}', flush=True)

    prepare = Path(share_dir) / 'launch' / 'swarm_scenario_prepare.py'
    if not prepare.is_file():
        raise FileNotFoundError(f'Falta preparador validado de 7B.2: {prepare}')
    if state_file.exists():
        state_file.unlink()  # no confundir estado antiguo con un éxito nuevo
    print('[7B.3] Preparando escenario con el código de 7B.2...', flush=True)
    run([
        sys.executable, '-u', str(prepare),
        '--seed', str(args.seed), '--scenario', str(scenario),
        '--scripts-dir', args.scripts_dir, '--ready-timeout', str(args.ready_timeout),
    ], check=True)
    if not scenario.is_file():
        raise RuntimeError('El preparador terminó sin generar JSON de escenario')

    # Unlike prior failed 7B launch, pass internal <world name> to Gazebo remove service.
    print(f'[7B.3] Iniciando misión 6E; servicio esperado /world/{actual_world}/remove', flush=True)
    result = run([
        'ros2', 'run', 'swarm_behavior', 'swarm_mission',
        '--scenario', str(scenario), '--world', actual_world,
        '--mission-state-file', str(state_file),
    ], check=False)
    if result.returncode:
        raise RuntimeError(f'El ejecutor CBS terminó con código {result.returncode}; revisar el log')
    check_mission_state(state_file)
    print('[7B.3] MISIÓN CONFIRMADA: 6 basuras entregadas, ninguna pendiente', flush=True)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--world-file', default='tugbot_warehouse')
    p.add_argument('--seed', type=int, default=42)
    p.add_argument('--scenario', default='/tmp/swarm_cbs_escenario.json')
    p.add_argument('--scripts-dir', default='~/swarm_ws/scripts')
    p.add_argument('--ready-timeout', type=float, default=180)
    p.add_argument('--mission-state-file', default='/tmp/swarm_refactor_7b3.json')
    args = p.parse_args(argv)
    try:
        from ament_index_python.packages import get_package_share_directory
        orchestrate(args, share_dir=get_package_share_directory('swarm_worlds'))
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f'[7B.3] ERROR: {exc}', file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
