#!/usr/bin/env python3
"""D.5.2 offline derivative bottleneck diagnosis for synchronized quintic Bezier paths.

Does not change controls, world, controller settings or send ROS commands.
Requires NumPy only.
"""
import argparse
import json
import math
from pathlib import Path
import numpy as np


def split_half(p):
    levels = [np.asarray(p, dtype=float)]
    while len(levels[-1]) > 1:
        levels.append((levels[-1][:-1] + levels[-1][1:]) / 2)
    left = np.array([lev[0] for lev in levels])
    right = np.array([lev[-1] for lev in levels[::-1]])
    return left, right


def bound(p, depth):
    parts = [np.asarray(p, dtype=float)]
    for _ in range(depth):
        parts = [sub for poly in parts for sub in split_half(poly)]
    return max(float(np.linalg.norm(poly, axis=1).max()) for poly in parts)


def diagnose(controls, dt, scenarios, factor=1.08, depth=4):
    if not (dt > 0 and math.isfinite(dt) and factor >= 1 and math.isfinite(factor)):
        raise ValueError('Invalid dt or safety factor')
    if not (0 <= depth <= 8):
        raise ValueError('Depth must be between 0 and 8')
    n_pieces = controls.shape[1]
    per_piece = []
    for rid in range(3):
        for piece in range(n_pieces):
            p = controls[rid, piece]
            vn = bound(5 * np.diff(p, axis=0), depth)
            an = bound(20 * np.diff(p, n=2, axis=0), depth)
            per_piece.append({'robot':f'summit{rid}', 'piece':piece,
                              'normalized_velocity_bound':vn,
                              'normalized_acceleration_bound':an,
                              'velocity_at_current_dt_m_s':vn/dt,
                              'acceleration_at_current_dt_m_s2':an/dt**2})
    out = []
    for vmax, amax in scenarios:
        if min(vmax, amax) <= 0 or not all(map(math.isfinite, (vmax,amax))):
            raise ValueError('Invalid limits')
        vel = max(per_piece, key=lambda row: row['normalized_velocity_bound']/vmax)
        acc = max(per_piece, key=lambda row: math.sqrt(row['normalized_acceleration_bound']/amax))
        tv = vel['normalized_velocity_bound']/vmax
        ta = math.sqrt(acc['normalized_acceleration_bound']/amax)
        new_dt = factor*max(tv, ta)
        limiter = vel if tv >= ta else acc
        out.append({'max_speed_m_s':vmax, 'max_acceleration_m_s2':amax,
                    'required_dt_from_speed_s':tv,
                    'required_dt_from_acceleration_s':ta,
                    'recommended_uniform_dt_s':new_dt,
                    'recommended_total_duration_s':new_dt*n_pieces,
                    'limiting_constraint':'speed' if tv >= ta else 'acceleration',
                    'limiting_robot':limiter['robot'], 'limiting_piece':limiter['piece'],
                    'theoretical_time_saving_pct':100*(1-new_dt/dt)})
    return {'source_dt_s':dt, 'pieces_per_robot':n_pieces, 'depth':depth,
            'factor':factor, 'source_duration_s':dt*n_pieces,
            'scenarios':out, 'piece_diagnostics':per_piece,
            'warning':'Analytical timing only. Controller motor limits, joint limits, obstacle safety and Gazebo tracking are NOT certified.'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controls',type=Path,required=True)
    parser.add_argument('--output',type=Path,default=Path('scripts/resultados/d52_analysis.json'))
    parser.add_argument('--factor',type=float,default=1.08)
    parser.add_argument('--depth',type=int,default=4)
    a=parser.parse_args()
    with np.load(a.controls,allow_pickle=False) as f:
        controls=np.stack([np.asarray(f[f'R{i}'],dtype=float) for i in range(3)])
        dt=float(f['dt_s'])
    if controls.ndim!=4 or controls.shape[2:]!=(6,2) or not np.isfinite(controls).all():
        raise ValueError('Controls must be 3 x pieces x 6 x 2, finite')
    combos=[(1.2,1.0),(1.4,1.0),(1.2,1.3),(1.4,1.3),(1.6,1.5)]
    result=diagnose(controls,dt,combos,a.factor,a.depth)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
    print(f"SOURCE: {a.controls}  dt={dt:.6f}s pieces={controls.shape[1]}")
    for x in result['scenarios']:
        print(f"v={x['max_speed_m_s']:.2f}, a={x['max_acceleration_m_s2']:.2f}: "
              f"duration={x['recommended_total_duration_s']:.3f}s, "
              f"limiting={x['limiting_constraint']} {x['limiting_robot']} "
              f"piece={x['limiting_piece']}")
    print('REPORT:',a.output)
    print(result['warning'])

if __name__=='__main__':
    main()
