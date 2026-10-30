#!/usr/bin/env python3
"""D.5.1: retime certified Hybrid V2 Bezier controls without changing geometry.

Uses conservative Bernstein derivative bounds after exact de Casteljau subdivisions.
Keeps ONE dt for every robot and piece, preserving synchronized temporal safety.
No ROS imports, no motion. Original certificate remains tied to geometry; dynamic
limits are recertified, and the output must pass the normal ROS preflight.
"""
from __future__ import annotations
import argparse, hashlib, json, math
from pathlib import Path
import numpy as np


def split_half(poly):
    levels=[np.asarray(poly,dtype=float)]
    while len(levels[-1])>1:
        levels.append((levels[-1][:-1]+levels[-1][1:])*0.5)
    return np.array([v[0] for v in levels]),np.array([v[-1] for v in levels[::-1]])


def bernstein_norm_upper(ctrl, degree, depth):
    """Upper bound for norm of a vector-valued polynomial over [0,1]."""
    pieces=[np.asarray(ctrl,dtype=float)]
    for _ in range(depth):
        nxt=[]
        for piece in pieces:
            a,b=split_half(piece);nxt.extend((a,b))
        pieces=nxt
    return max(float(np.linalg.norm(p,axis=1).max()) for p in pieces)


def bounds(controls,depth):
    c=np.asarray(controls,dtype=float)
    if c.ndim!=4 or c.shape[2:]!=(6,2) or not np.isfinite(c).all():
        raise ValueError('Control arrays must have shape (robots,pieces,6,2) and be finite')
    if c.shape[0]!=3 or c.shape[1]<1: raise ValueError('Expected 3 robots and at least 1 piece')
    vel=0.0;acc=0.0
    for piece in c.reshape(-1,6,2):
        vel=max(vel,bernstein_norm_upper(5*np.diff(piece,axis=0),4,depth))
        acc=max(acc,bernstein_norm_upper(20*np.diff(piece,n=2,axis=0),3,depth))
    return vel,acc


def retime(controls, old_dt, *, speed, accel, factor, depth):
    if any(not math.isfinite(x) for x in (old_dt,speed,accel,factor)) or min(old_dt,speed,accel)<=0 or factor<1:
        raise ValueError('Invalid dt/kinematic settings')
    if depth<0 or depth>8:raise ValueError('depth must be in [0,8]')
    vb,ab=bounds(controls,depth)
    dt=factor*max(vb/speed, math.sqrt(ab/accel),1e-3)
    # Avoid *increasing* speed relative to baseline only if the user explicitly chose otherwise;
    # here new dt can be larger if required by bounds, and is always reported.
    return {'old_dt_s':float(old_dt),'new_dt_s':float(dt),
            'old_total_s':float(old_dt*controls.shape[1]),'new_total_s':float(dt*controls.shape[1]),
            'speed_up':float(old_dt/dt), 'velocity_bound_normalized':float(vb),
            'acceleration_bound_normalized':float(ab), 'speed_bound_m_s':float(vb/dt),
            'acceleration_bound_m_s2':float(ab/dt**2), 'depth':depth,'factor':factor}


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--report',required=True,type=Path)
    ap.add_argument('--controls',required=True,type=Path)
    ap.add_argument('--stage',default='pickup1')
    ap.add_argument('--output-dir',required=True,type=Path)
    ap.add_argument('--max-speed',type=float,default=1.20)
    ap.add_argument('--max-accel',type=float,default=1.00)
    ap.add_argument('--safety-factor',type=float,default=1.08)
    ap.add_argument('--subdivisions',type=int,default=4)
    a=ap.parse_args()
    data=json.loads(a.report.read_text(encoding='utf-8'))
    stage=data['stages'][a.stage]
    cert=stage['hybrid_v2']
    if not stage.get('certified') or not cert.get('certified') or cert.get('status')!='CERTIFIED':
        raise ValueError('Source plan is not certified')
    if not a.controls.exists(): raise FileNotFoundError(a.controls)
    with np.load(a.controls,allow_pickle=False) as f:
        c=np.stack([np.asarray(f[f'R{i}'],dtype=float) for i in range(3)])
        old_dt=float(f['dt_s'])
    if abs(old_dt-float(cert['dt_s']))>1e-6:raise ValueError('Original NPZ dt differs from report')
    if c.shape[1]!=int(cert['pieces']):raise ValueError('Piece count differs from report')
    info=retime(c,old_dt,speed=a.max_speed,accel=a.max_accel,factor=a.safety_factor,depth=a.subdivisions)
    if info['speed_bound_m_s']>a.max_speed+1e-10 or info['acceleration_bound_m_s2']>a.max_accel+1e-10:
        raise AssertionError('Derivative bound failed')
    # The geometry is unmodified. New certificate is limited to time-invariant
    # geometric checks originally performed. No guarantee of Gazebo tracking.
    newdata=json.loads(json.dumps(data))
    item=newdata['stages'][a.stage]
    report=item['hybrid_v2']
    report['dt_s']=info['new_dt_s'];report['duration_s']=info['new_total_s']
    report['speed_upper_bound_m_s']=info['speed_bound_m_s']
    report['acceleration_upper_bound_m_s2']=info['acceleration_bound_m_s2']
    report['retiming_d51']={**info,'method':'de_casteljau_Bernstein_convex_hull',
                         'source_controls_sha256':hashlib.sha256(a.controls.read_bytes()).hexdigest(),
                         'source_report_sha256':hashlib.sha256(a.report.read_bytes()).hexdigest(),
                         'geometry_unchanged':True,
                         'warning':'Offline derivative recertification, not actual Gazebo control/obstacle certification'}
    a.output_dir.mkdir(parents=True,exist_ok=True)
    out_c=a.output_dir/(a.stage+'_retimed_controls.npz')
    out_r=a.output_dir/(a.stage+'_retimed_report.json')
    np.savez_compressed(out_c,**{f'R{i}':c[i] for i in range(3)},dt_s=info['new_dt_s'])
    item['controls_file']=str(out_c.resolve())
    out_r.write_text(json.dumps(newdata,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(info,indent=2))
    print('REPORT:',out_r,'\nCONTROLS:',out_c)
    print('DRY RUN FIRST. New timings still require live Gazebo preflight and a controlled test.')

if __name__=='__main__':main()
