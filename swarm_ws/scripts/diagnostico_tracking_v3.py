#!/usr/bin/env python3
"""Diagnóstico de seguimiento V3, solo lectura. No publica comandos ROS.

Ejemplo:
  python3 diagnostico_tracking_v3.py --workspace ~/swarm_ws \
      --run scripts/resultados/experiments/fixed_seed2026_... \
      --seed 2026 --mode fixed --stage pickup1 --robot summit2
"""
from __future__ import annotations
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


def percentile(values, q):
    return float(np.percentile(np.asarray(values, dtype=float), q))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--workspace', type=Path, default=Path.home()/'swarm_ws')
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--seed', type=int, required=True)
    p.add_argument('--mode', choices=('fixed','milp'), default='fixed')
    p.add_argument('--stage', default='pickup1')
    p.add_argument('--robot', default='summit2', choices=('summit0','summit1','summit2'))
    p.add_argument('--threshold-m', type=float, default=0.15)
    p.add_argument('--csv-output', type=Path, default=None)
    a = p.parse_args()

    try:
        from swarm_behavior.swarm_planning.hybrid_refinement.timed_protocol import (
            TimedCommand, SuccessiveTimedCommand, reference)
        from swarm_behavior.swarm_planning.hybrid_refinement.successive_clock import clock_from_payload
    except ImportError as exc:
        raise SystemExit('No se puede importar el protocolo V3: ejecuta "source ~/swarm_ws/install/setup.bash" dentro de Docker. ' + str(exc))

    ws = a.workspace.expanduser().resolve()
    run = a.run.expanduser()
    if not run.is_absolute():
        run = (ws/run).resolve()

    nav = json.loads((run/f'{a.stage}_navigation.json').read_text(encoding='utf-8'))
    start = float(nav['start_time_s'])
    warp = nav['successive_timewarp']
    if warp.get('model') != 'common_collective_successive_v1':
        raise SystemExit('Este diagnóstico requiere reloj V3 sucesivo')

    plan_root = ws/'scripts/supermarket_seed_pipeline/plans'/f'seed_{a.seed}'
    if a.mode == 'fixed':
        plan_root /= 'fixed'
    contract_path = plan_root/'prepared'/'mission_navigation_contract.json'
    contract = json.loads(contract_path.read_text(encoding='utf-8'))
    npz_path = Path(contract['stages'][a.stage]['trajectory'])
    if not npz_path.is_absolute():
        npz_path = ws/npz_path
    if not npz_path.is_file():
        raise SystemExit(f'No existe NPZ: {npz_path}')

    idx = int(a.robot[-1])
    with np.load(npz_path, allow_pickle=False) as z:
        controls = np.asarray(z[f'R{idx}_controls'], dtype=float)
        durations = np.asarray(z[f'R{idx}_durations_s'], dtype=float)
    source = TimedCommand(
        action='start', session_id='diagnostico', robot=a.robot,
        frame_id=f'{a.robot}/odom', start_time_s=start,
        control_points=tuple(tuple(tuple(float(c) for c in pt) for pt in piece) for piece in controls),
        segment_durations_s=tuple(float(x) for x in durations),
    )
    timed = SuccessiveTimedCommand(source, warp, clock_from_payload(warp))

    rows = []
    with (run/'odom.csv').open(newline='', encoding='utf-8') as f:
        for row in csv.DictReader(f):
            if row['robot'] != a.robot:
                continue
            t = float(row['stamp_s'])
            if not (start <= t <= start+timed.duration_s + 2):
                continue
            x = float(row['x_m']); y = float(row['y_m'])
            ref, vel, phase = reference(timed, t)
            ex, ey = ref[0]-x, ref[1]-y
            err = math.hypot(ex, ey)
            v = math.hypot(*vel)
            if v >= 0.10:
                # Positivo = robot retrasado en dirección del movimiento de la referencia.
                longitudinal = (ex*vel[0]+ey*vel[1])/v
                lateral = (vel[0]*ey-vel[1]*ex)/v
            else:
                longitudinal = lateral = float('nan')
            rows.append({
                'stamp_s':t, 'stage_elapsed_s':t-start,
                'x_odom_m':x,'y_odom_m':y,
                'x_ref_m':float(ref[0]),'y_ref_m':float(ref[1]),
                'reference_speed_m_s':v, 'tracking_error_m':err,
                'longitudinal_error_m':longitudinal,
                'lateral_error_m':lateral,
                'phase':phase,
            })
    if not rows:
        raise SystemExit('No hay odometría del robot durante la etapa solicitada')
    rows.sort(key=lambda r:r['stamp_s'])
    errs=[r['tracking_error_m'] for r in rows]
    first = next((r for r in rows if r['tracking_error_m'] > a.threshold_m),None)
    peak = max(rows,key=lambda r:r['tracking_error_m'])
    near = [r for r in rows if r['reference_speed_m_s']>=0.10]
    print(f'RUN: {run.name}\nRobot: {a.robot} | Etapa: {a.stage}')
    print(f'Inicio: {start:.3f} s (/clock) | Duración V3: {timed.duration_s:.3f} s')
    print(f'Muestras: {len(rows)} | intervalo de odometría: {rows[0]["stamp_s"]:.3f} a {rows[-1]["stamp_s"]:.3f} s')
    print(f'Error medio: {sum(errs)/len(errs):.3f} m | p95: {percentile(errs,95):.3f} m | máximo: {peak["tracking_error_m"]:.3f} m')
    print(f'Máximo a t_sim={peak["stamp_s"]:.3f}, tiempo desde inicio={peak["stage_elapsed_s"]:.3f}s')
    print(f'  Odometría: ({peak["x_odom_m"]:.3f}, {peak["y_odom_m"]:.3f})')
    print(f'  Referencia: ({peak["x_ref_m"]:.3f}, {peak["y_ref_m"]:.3f})')
    if first:
        print(f'Primer cruce de {a.threshold_m:.3f}m: t_sim={first["stamp_s"]:.3f}, elapsed={first["stage_elapsed_s"]:.3f}s')
        if math.isfinite(first['longitudinal_error_m']):
            print(f'  Longitudinal={first["longitudinal_error_m"]:+.3f}m, lateral={first["lateral_error_m"]:+.3f}m, velocidad ref={first["reference_speed_m_s"]:.3f}m/s')
        else:
            print('  Velocidad de referencia baja; no es fiable descomponer el error en longitudinal/lateral')
    else:
        print('No se detectó cruce del umbral en el CSV (el registro puede terminar antes del fallo).')
    if near:
        r=max(near,key=lambda x:abs(x['longitudinal_error_m']))
        print(f'Mayor error longitudinal absoluto (v_ref >= 0.10): {r["longitudinal_error_m"]:+.3f}m, t_sim={r["stamp_s"]:.3f}')
        r=max(near,key=lambda x:abs(x['lateral_error_m']))
        print(f'Mayor error lateral absoluto (v_ref >= 0.10): {r["lateral_error_m"]:+.3f}m, t_sim={r["stamp_s"]:.3f}')

    out=a.csv_output or (run/f'{a.stage}_{a.robot}_diagnostico_tracking_v3.csv')
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('w', newline='', encoding='utf-8') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]))
        w.writeheader();w.writerows(rows)
    print('DETALLE_CSV:',out)
    print('NOTA: el cálculo supone que la odometría y los controles usan el mismo marco cartesiano, como exige el seguidor.')
    print('NOTA: picos posteriores a la cancelación pueden diferir del error que activó el fallo.')

if __name__=='__main__':
    main()
