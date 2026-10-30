#!/usr/bin/env python3
"""Subdivide certified D5.1 quintic Bezier pieces for the conservative ROS V1 protocol.

Does NOT change the geometric curves or their time parametrization.
Does NOT certify Gazebo safety or authorize movement.
"""
import argparse
import json
from pathlib import Path
import numpy as np


def bisect(p):
    levels = [np.asarray(p, dtype=float)]
    for _ in range(5):
        prev = levels[-1]
        levels.append((prev[:-1] + prev[1:]) * 0.5)
    return np.array([v[0] for v in levels]), np.array([v[-1] for v in levels[::-1]])


def refine(pieces, depth):
    out = np.asarray(pieces, dtype=float)
    for _ in range(depth):
        out = np.array([s for p in out for s in bisect(p)])
    return out


def conservative_bounds(arr, dt):
    v = np.linalg.norm(5 * np.diff(arr, axis=2), axis=-1).max() / dt
    a = np.linalg.norm(20 * np.diff(arr, n=2, axis=2), axis=-1).max() / dt**2
    return float(v), float(a)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--report', type=Path, required=True)
    ap.add_argument('--controls', type=Path, required=True)
    ap.add_argument('--stage', default='pickup1')
    ap.add_argument('--output-dir', type=Path, required=True)
    ap.add_argument('--max-speed', type=float, default=1.20)
    ap.add_argument('--max-accel', type=float, default=1.00)
    ap.add_argument('--max-depth', type=int, default=4)
    args = ap.parse_args()
    doc = json.loads(args.report.read_text(encoding='utf-8'))
    item = doc['stages'][args.stage]
    cert = item['hybrid_v2']
    if not item.get('certified') or not cert.get('certified') or cert.get('status') != 'CERTIFIED':
        raise ValueError('El informe de origen no está certificado')
    with np.load(args.controls, allow_pickle=False) as src:
        arr = np.stack([src[f'R{i}'] for i in range(3)])
        dt = float(src['dt_s'])
    if arr.ndim != 4 or arr.shape[2:] != (6,2) or not np.isfinite(arr).all() or not np.isfinite(dt) or dt <= 0:
        raise ValueError('NPZ inválido')
    if len(arr[0]) != int(cert['pieces']) or abs(dt - float(cert['dt_s'])) > 1e-6:
        raise ValueError('NPZ no coincide con informe')
    prev_duration = arr.shape[1] * dt
    selected = None
    for depth in range(args.max_depth + 1):
        factor = 2**depth
        candidate = np.stack([refine(a,depth) for a in arr])
        new_dt = dt / factor
        if candidate.shape[1] > 200 or new_dt < 0.02:
            break
        v,a = conservative_bounds(candidate,new_dt)
        if v <= args.max_speed + 1e-6 and a <= args.max_accel + 1e-6:
            selected=(depth,candidate,new_dt,v,a)
            break
    if selected is None:
        raise SystemExit('REJECTED: subdivisión insuficiente para satisfacer el protocolo V1')
    depth,candidate,new_dt,v,a=selected
    if abs(candidate.shape[1]*new_dt-prev_duration) > 1e-8:
        raise AssertionError('Duración no conservada')
    for i in range(3):
        if not np.allclose(candidate[i,0,0],arr[i,0,0],atol=1e-10) or not np.allclose(candidate[i,-1,-1],arr[i,-1,-1],atol=1e-10):
            raise AssertionError('Extremos modificados')
    item['hybrid_v2'].update(pieces=int(candidate.shape[1]),dt_s=new_dt,
        duration_s=prev_duration,speed_upper_bound_m_s=v,acceleration_upper_bound_m_s2=a)
    item['hybrid_v2']['protocol_subdivision_d51']={'depth':depth,'factor':2**depth,
        'original_pieces':int(arr.shape[1]),'new_pieces':int(candidate.shape[1]),
        'geometry_unchanged':True,'timing_unchanged':True,
        'warning':'Representación subdividida para comprobación de protocolo, no certificación de seguimiento en Gazebo'}
    args.output_dir.mkdir(parents=True,exist_ok=True)
    dest_npz=args.output_dir/(args.stage+'_protocol_controls.npz')
    dest_json=args.output_dir/(args.stage+'_protocol_report.json')
    np.savez_compressed(dest_npz, **{f'R{i}':candidate[i] for i in range(3)},dt_s=new_dt)
    item['controls_file']=str(dest_npz.resolve())
    dest_json.write_text(json.dumps(doc,ensure_ascii=False,indent=2),encoding='utf-8')
    print(f'PROTOCOL READY: depth={depth} pieces={candidate.shape[1]} dt={new_dt:.6f}s total={prev_duration:.3f}s speed_bound={v:.3f}m/s accel_bound={a:.3f}m/s²')
    print('JSON:',dest_json,'\nNPZ:',dest_npz)
    print('DRY RUN obligatorio; no se ha enviado movimiento')

if __name__=='__main__':main()
