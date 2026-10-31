#!/usr/bin/env python3
"""Adaptador conservador de pickup2/delivery2: reutiliza D9.9.7 y D9.9.13 originales.

Por defecto no publica comandos. NO restaura el mundo Gazebo tras un reinicio.
"""
import argparse
import hashlib
import json
import math
import re
from pathlib import Path
import subprocess
import sys

EXPECTED = {
    'pickup2': {
        'sha': '7f05c330ef25bb2c0b7bb84c3b1acb4661134919c39480d39cf22cf59435ed91',
        'starts': [(5.0, -10.0), (-2.5, -1.25), (-5.0, 15.0)],
        'ends': [(-6.25, -17.5), (-1.25, -5.0), (2.5, 12.5)],
        'predecessor': 'delivery1',
        'predecessor_sha': '500234f35039471f15f42801d3c76ebc5686cbe4c480fac6eda98c142fdec0e3',
        'predecessor_physical_stage': 'delivery1',
    },
    'delivery2': {
        'sha': '33aff6468eea915cc8f3e6da3d3df896392ca2fc996737bde7c02918dba65b70',
        'starts': [(-6.25, -17.5), (-1.25, -5.0), (2.5, 12.5)],
        'ends': [(5.0, -10.0), (-2.5, 15.0), (-2.5, 1.25)],
        'predecessor': 'pickup2',
        'predecessor_sha': '7f05c330ef25bb2c0b7bb84c3b1acb4661134919c39480d39cf22cf59435ed91',
        'predecessor_physical_stage': 'pickup2',
    }
}


def validate_files(ws, stage):
    import numpy as np
    spec = EXPECTED[stage]
    folder = ws / 'scripts/supermarket_v3/data'
    npz = folder / f'{stage}_supermarket_seed100_v3.npz'
    clock = folder / f'{stage}_supermarket_seed100_clock.json'
    for p in (npz, clock, ws/'scripts/d997_initial_alignment_path.py', ws/'scripts/d9913_stage_coordinator.py'):
        if not p.is_file():
            raise RuntimeError(f'Archivo faltante: {p}')
    sha = hashlib.sha256(npz.read_bytes()).hexdigest()
    if sha != spec['sha']:
        raise RuntimeError('El SHA-256 del NPZ no coincide con el calculado en el paquete validado')
    meta = json.loads(clock.read_text(encoding='utf-8'))
    if (meta.get('input_sha256') != sha or meta.get('stage') != stage or meta.get('seed') != 100
            or meta.get('status') != 'OFFLINE_SCHEDULE_CERTIFIED_NOT_EXECUTABLE'
            or meta.get('common_time_warp') is not True):
        raise RuntimeError('El reloj V3 no corresponde a la etapa, semilla o trayectoria')
    if not 0 < meta.get('duration_successive_s', 0) <= meta.get('duration_original_s', 0):
        raise RuntimeError('Duración sucesiva incoherente')
    with np.load(npz, allow_pickle=False) as arrays:
        for i in range(3):
            cps = arrays[f'R{i}_controls']; durations = arrays[f'R{i}_durations_s']
            if (cps.ndim != 3 or cps.shape[1:] != (6, 2) or len(durations) != len(cps)
                    or len(cps) < 1 or not np.isfinite(cps).all() or not np.isfinite(durations).all()
                    or not np.all(durations > 0)):
                raise RuntimeError(f'Controles o duraciones inválidos para Summit {i}')
            if math.dist(cps[0, 0], spec['starts'][i]) > 1e-6 or math.dist(cps[-1, -1], spec['ends'][i]) > 1e-6:
                raise RuntimeError(f'Summit {i}: inicio/fin no coinciden con CBS')
    return npz, clock, sha, meta


def check_previous(stage, navigation_path, physical_paths):
    spec = EXPECTED[stage]
    if navigation_path is None or len(physical_paths) != 3:
        raise RuntimeError('Se requiere el reporte de navegación anterior y los tres reportes físicos de esa MISMA sesión Gazebo')
    nav = json.loads(navigation_path.read_text(encoding='utf-8'))
    old_stage = spec['predecessor']
    if (nav.get('stage') != old_stage or nav.get('status') != 'DONE_ALL_'+old_stage.upper()
            or nav.get('trajectory_sha256') != spec['predecessor_sha']
            or nav.get('commands_sent') != 3 or nav.get('problems') or not nav.get('execution_requested')):
        raise RuntimeError('El reporte de navegación previo no acredita la etapa anterior')
    seen = set()
    for report_path in physical_paths:
        d = json.loads(report_path.read_text(encoding='utf-8'))
        robot = d.get('robot')
        if (robot not in ('summit0', 'summit1', 'summit2') or robot in seen
                or d.get('stage') != spec['predecessor_physical_stage']
                or d.get('status') != 'PHYSICAL_ACK_REQUIRES_GAZEBO_INSPECTION'
                or d.get('problems') or d.get('execution_requested') is not True
                or d.get('original_exit_code') != 0):
            raise RuntimeError(f'Reporte físico previo no apto: {report_path}')
        seen.add(robot)
    if seen != {'summit0','summit1','summit2'}:
        raise RuntimeError('Faltan evidencias físicas de alguno de los tres Summit')
    # Nota: los JSON anteriores no tienen un identificador de sesión Gazebo;
    # la confirmación de sesión ACTUAL debe hacerla el operador.
    return {'navigation_session_id':nav.get('session_id'),'physical_robots':sorted(seen)}


def run(cmd):
    print('Comando:', ' '.join(map(str, cmd)), flush=True)
    return subprocess.run([str(x) for x in cmd], check=False).returncode


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--workspace', type=Path, default=Path.home()/'swarm_ws')
    ap.add_argument('--run-id', default='prueba01', help='Identificador de esta nueva sesión; usar uno distinto cada vez, solo letras, números, _ o -')
    ap.add_argument('--stage', choices=tuple(EXPECTED), required=True)
    ap.add_argument('--mode', choices=('check','alignment-dry','alignment-execute','coordinator-dry','coordinator-execute'), default='check')
    ap.add_argument('--previous-navigation-report', type=Path)
    ap.add_argument('--previous-physical-report', type=Path, action='append', default=[], help='Repetir tres veces: Summit 0, 1 y 2')
    ap.add_argument('--confirm-current-session', action='store_true', help='Confirmo que los reportes previos corresponden al Gazebo actualmente abierto, NO a una sesión antigua')
    ap.add_argument('--confirm-physical-state', action='store_true', help='Confirmo visualmente: tras delivery1 los tres robots vacíos, o tras pickup2 los tres cargados')
    ap.add_argument('--confirm-ready', action='store_true', help='Sin movimiento simultáneo, tres seguidores, TF, entorno inspeccionado y parada accesible')
    a = ap.parse_args()
    try:
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,48}', a.run_id):
            raise RuntimeError('run-id inválido')
        ws = a.workspace.expanduser().resolve()
        npz, clock, sha, meta = validate_files(ws, a.stage)
        print(json.dumps({'stage':a.stage,'mode':a.mode,'seed':100,'sha256':sha,
                          'starts':EXPECTED[a.stage]['starts'],'ends':EXPECTED[a.stage]['ends'],
                          'planned_collective_duration_s':meta['duration_successive_s']},indent=2),flush=True)
        if a.mode == 'check':
            return 0
        if not (a.confirm_current_session and a.confirm_physical_state):
            raise RuntimeError('Confirmación del estado físico y de que es la MISMA sesión Gazebo requerida')
        evidence = check_previous(a.stage, a.previous_navigation_report, a.previous_physical_report)
        print('Evidencia previa:',json.dumps(evidence,ensure_ascii=False),flush=True)
        active = a.mode.endswith('-execute')
        if active and not a.confirm_ready:
            raise RuntimeError('Movimiento bloqueado: falta --confirm-ready')
        out = ws/'scripts/resultados'
        out.mkdir(parents=True,exist_ok=True)
        if a.mode.startswith('alignment'):
            report = out/f'supermarket_{a.stage}_{a.run_id}_alignment_{"REAL" if active else "DRY"}.json'
            if report.exists() and active:
                raise RuntimeError(f'El reporte de alineación real ya existe, no sobrescribir: {report}')
            cmd = [sys.executable,ws/'scripts/d997_initial_alignment_path.py', '--trajectory',npz,
                   '--output',report,'--position-tolerance','0.03','--max-offset','0.30',
                   '--stationary-speed','0.03','--settle-seconds','1.0','--timeout','30.0']
            if active: cmd.append('--execute')
            code=run(cmd)
            if code: return code
            r=json.loads(report.read_text(encoding='utf-8'))
            accepted=('ALIGNED','ALREADY_ALIGNED') if active else ('DRY_RUN_READY','ALREADY_ALIGNED')
            return 0 if r.get('status') in accepted and not r.get('problems') else 2
        if active:
            alignment=out/f'supermarket_{a.stage}_{a.run_id}_alignment_REAL.json'
            if not alignment.is_file():
                raise RuntimeError('Falta evidencia de alineación real de esta etapa')
            result=json.loads(alignment.read_text(encoding='utf-8'))
            if result.get('status') not in ('ALIGNED','ALREADY_ALIGNED') or result.get('problems'):
                raise RuntimeError('Alineación real previa no aprobada')
        report=out/f'supermarket_{a.stage}_{a.run_id}_{"REAL" if active else "DRY"}.json'
        if report.exists() and active:
            raise RuntimeError(f'Reporte de ejecución real ya existe: {report}. No repetir automáticamente')
        cmd=[sys.executable, ws/'scripts/d9913_stage_coordinator.py',
             '--stage',a.stage,'--trajectory',npz,'--successive-clock',clock,
             '--expected-sha256',sha,'--nav-speed-limit','2.0','--nav-accel-limit','2.0',
             '--observe-seconds','8','--start-delay','15','--output',report]
        if active:cmd.extend(['--execute','--confirm-runtime-v2'])
        code=run(cmd)
        if code: return code
        result=json.loads(report.read_text(encoding='utf-8'))
        expected_status=('DONE_ALL_'+a.stage.upper()) if active else 'PASS_GUARDED_DRY_RUN_NO_COMMANDS'
        return 0 if result.get('status')==expected_status and not result.get('problems') else 2
    except Exception as exc:
        print('BLOQUEADO:',exc,file=sys.stderr)
        return 2

if __name__ == '__main__':
    raise SystemExit(main())
