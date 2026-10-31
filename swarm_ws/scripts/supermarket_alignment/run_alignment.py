#!/usr/bin/env python3
"""Adaptador seguro de D9.9.7 original para supermarket/pickup1/seed100.

NO reimplementa el control de alineación. Invoca scripts/d997_initial_alignment_path.py.
Dry-run por defecto. --execute habilita rutas cortas via planned_path.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

EXPECTED_SHA = '4a3015c4ac3d64da0555b485a3bfac77b7243340d251c8f6388c19ac35f19bd4'
EXPECTED_STARTS = {'summit0': (-5.0, 22.5), 'summit1': (1.25, -17.5), 'summit2': (-7.5, -8.75)}


def static_check(workspace: Path) -> dict:
    import numpy as np
    root = workspace.resolve()
    scenario = root / 'config/supermarket/seed_100.json'
    npz = root / 'scripts/supermarket_v3/data/pickup1_supermarket_seed100_v3.npz'
    clock = root / 'scripts/supermarket_v3/data/pickup1_supermarket_seed100_clock.json'
    alignment = root / 'scripts/d997_initial_alignment_path.py'
    for path in (scenario, npz, clock, alignment):
        if not path.is_file():
            raise RuntimeError(f'Archivo requerido ausente: {path}')
    js = json.loads(scenario.read_text(encoding='utf8'))
    if js.get('world') != 'supermarket' or js.get('seed') != 100:
        raise RuntimeError('El escenario no es supermarket / seed 100')
    robots = js.get('robots', [])
    starts_scenario = {r.get('name'): tuple(r.get('xy_m', ())) for r in robots}
    if len(robots) != 3 or set(starts_scenario) != set(EXPECTED_STARTS):
        raise RuntimeError('Robots del escenario diferentes de summit0/1/2')
    digest = hashlib.sha256(npz.read_bytes()).hexdigest()
    if digest != EXPECTED_SHA:
        raise RuntimeError('NPZ no coincide con SHA256 autorizado de supermarket pickup1 seed100')
    cj = json.loads(clock.read_text(encoding='utf8'))
    if (cj.get('input_sha256') != digest or cj.get('status') != 'OFFLINE_SCHEDULE_CERTIFIED_NOT_EXECUTABLE'):
        raise RuntimeError('Reloj V3 incompatible con el NPZ')
    with np.load(npz, allow_pickle=False) as archive:
        for i, (name, xy) in enumerate(EXPECTED_STARTS.items()):
            cp = archive[f'R{i}_controls']
            if cp.ndim != 3 or cp.shape[1:] != (6, 2) or len(cp) < 1 or not np.isfinite(cp).all():
                raise RuntimeError(f'{name}: controles Bézier no válidos')
            start = tuple(map(float, cp[0, 0]))
            if math.dist(start, xy) > 1e-6 or math.dist(xy, starts_scenario[name]) > 1e-6:
                raise RuntimeError(f'{name}: inicio diferente entre NPZ, escenario y referencia fija')
    return {'npz': npz, 'scenario': scenario, 'clock': clock, 'alignment': alignment,
            'digest': digest, 'starts': EXPECTED_STARTS}


def get_parameter(node: str, param: str) -> str:
    try:
        p = subprocess.run(['ros2', 'param', 'get', node, param], capture_output=True,
                           text=True, timeout=8, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RuntimeError(f'{node}: no se pudo leer {param}: {exc}') from exc
    if p.returncode:
        raise RuntimeError(f'{node}: ros2 param get {param} falló: {(p.stdout+p.stderr).strip()}')
    output = p.stdout.strip()
    if not output:
        raise RuntimeError(f'{node}: {param} sin respuesta')
    return output.split()[-1]


def runtime_check() -> dict:
    results = {}
    for name in EXPECTED_STARTS:
        node = f'/{name}/path_follower'
        tol = float(get_parameter(node, 'waypoint_tolerance'))
        max_speed = float(get_parameter(node, 'max_speed'))
        enabled = get_parameter(node, 'hybrid_enabled').lower()
        sim = get_parameter(node, 'use_sim_time').lower()
        v3speed = float(get_parameter(node, 'hybrid_timed_max_speed'))
        if not (math.isfinite(tol) and 0 < tol <= 0.03):
            raise RuntimeError(f'{node}: waypoint_tolerance={tol}; reinicia con -p waypoint_tolerance:=0.02')
        if not (math.isfinite(max_speed) and 0 < max_speed <= 0.30):
            raise RuntimeError(f'{node}: max_speed={max_speed}; reinicia con -p max_speed:=0.25')
        if enabled != 'true' or sim != 'true':
            raise RuntimeError(f'{node}: hybrid_enabled={enabled}, use_sim_time={sim}; ambos deben ser true')
        if not (math.isfinite(v3speed) and 0 < v3speed <= 2.0):
            raise RuntimeError(f'{node}: hybrid_timed_max_speed inválido: {v3speed}')
        results[name] = {'waypoint_tolerance_m': tol, 'alignment_max_speed_m_s': max_speed,
                         'hybrid_timed_max_speed_m_s': v3speed}
    return results


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--workspace', type=Path, default=Path.home() / 'swarm_ws')
    p.add_argument('--execute', action='store_true', help='Alineación ACTIVA; puede mover los Summit')
    p.add_argument('--output', type=Path, help='Ruta JSON; por defecto scripts/resultados/supermarket_...')
    args = p.parse_args(argv)
    try:
        checked = static_check(args.workspace)
        print('Escenario/NPZ/reloj: VALIDADO; SHA256:', checked['digest'], flush=True)
        if args.execute:
            print('Comprobando parámetros efectivos de los tres seguidores...', flush=True)
            print(json.dumps(runtime_check(), indent=2), flush=True)
        report = args.output or (args.workspace / 'scripts/resultados' /
                                ('supermarket_pickup1_alignment_REAL.json' if args.execute else
                                 'supermarket_pickup1_alignment_DRY.json'))
        cmd = [sys.executable, str(checked['alignment']), '--trajectory', str(checked['npz']),
               '--output', str(report), '--position-tolerance', '0.03', '--max-offset', '0.30',
               '--stationary-speed', '0.03', '--settle-seconds', '1.0', '--timeout', '30.0']
        if args.execute:
            cmd.append('--execute')
        print('Invocando D9.9.7 ORIGINAL:', ' '.join(cmd), flush=True)
        proc = subprocess.run(cmd, check=False)
        if report.is_file():
            result = json.loads(report.read_text(encoding='utf8'))
            allowed = {'ALIGNED', 'ALREADY_ALIGNED'} if args.execute else {'DRY_RUN_READY', 'ALREADY_ALIGNED'}
            if result.get('status') not in allowed or result.get('problems'):
                print('ALINEACIÓN NO APROBADA: '+str(result.get('problems')), file=sys.stderr)
                return 2
        else:
            print('No se generó informe de alineación', file=sys.stderr)
            return 2
        print('Informe:', report, flush=True)
        print('Nota: este procedimiento no autoriza por sí solo ejecutar V3; repetir dry-run del coordinador.', flush=True)
        return proc.returncode
    except (RuntimeError, ValueError, KeyError, OSError) as exc:
        print('BLOQUEADO ANTES DE MOVER:', str(exc), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
