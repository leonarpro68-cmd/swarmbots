#!/usr/bin/env python3
"""D9.6A: offline/read-only evaluation of independent-time quintic Bezier plans.

NO rclpy, ROS publishers, robot velocity commands, or execution messages.
Reads D9.5 manifests with strict integrity checks and evaluates the planned
state for all three robots against one shared mission clock.
"""
import argparse
import csv
import json
import hashlib
import math
from pathlib import Path
import numpy as np

from d95_protocol_preflight import ROBOTS, STAGES, SCHEMA, digest
from d94_convert_offline import converted_values


def load_stage(run, stage):
    manifest_path = run / 'd95_preflight' / f'{stage}_INDEPENDENT_TIMED_BEZIER_PREVIEW_ONLY.json'
    manifest = json.loads(manifest_path.read_text())
    if manifest.get('schema') != SCHEMA or manifest.get('stage') != stage:
        raise ValueError('Invalid stage or preview schema')
    if manifest.get('status') != 'PASS_READ_ONLY':
        raise ValueError('Stage has not passed D9.5 preflight')
    if manifest.get('ros_executable') is not False or manifest.get('publisher_enabled') is not False:
        raise ValueError('Refusing potentially executable manifest')
    if manifest.get('legacy_v1_compatible') is not False:
        raise ValueError('Unexpected legacy protocol compatibility claim')
    files = {'npz':run/'d94_conversion'/f'{stage}_converted_NON_EXECUTABLE.npz',
             'd94_report':run/'d94_conversion'/f'{stage}_conversion_report.json',
             'd92_report':run/'d92_offline_report.json'}
    for key, path in files.items():
        if digest(path) != manifest['source_sha256'][key]:
            raise ValueError(f'Integrity mismatch ({key}): {path}')
    trajectories = {}
    with np.load(files['npz'], allow_pickle=False) as archive:
        for index, robot in enumerate(ROBOTS):
            controls = np.asarray(archive[f'R{index}_controls'], dtype=float)
            durations = np.asarray(archive[f'R{index}_durations_s'], dtype=float)
            info = manifest['robots'][robot]
            if controls.ndim != 3 or controls.shape[1:] != (6, 2):
                raise ValueError(f'{robot}: invalid controls shape')
            if durations.shape != (len(controls),) or len(durations) == 0:
                raise ValueError(f'{robot}: invalid segment durations')
            if not np.all(np.isfinite(controls)) or not np.all(np.isfinite(durations)) or np.any(durations <= 0):
                raise ValueError(f'{robot}: invalid numeric data')
            if len(durations) != info['segments'] or not np.allclose(durations,info['segment_durations_s'],rtol=0,atol=1e-9):
                raise ValueError(f'{robot}: manifest/NPZ segment mismatch')
            if abs(float(durations.sum())-info['duration_s'])>1e-8 or not info['checks_passed']:
                raise ValueError(f'{robot}: manifest duration or validation mismatch')
            trajectories[robot] = (controls,durations)
    return manifest, trajectories


def sample(trajectories, t):
    if not np.isfinite(t) or t < 0:
        raise ValueError('Mission time must be finite and nonnegative')
    states={}
    for robot, (controls,durations) in trajectories.items():
        p,v,a=converted_values(controls,durations,np.array([t],dtype=float))
        T=float(np.sum(durations))
        states[robot]={'x_m':float(p[0,0]),'y_m':float(p[0,1]),
                       'vx_m_s':float(v[0,0]),'vy_m_s':float(v[0,1]),
                       'ax_m_s2':float(a[0,0]),'ay_m_s2':float(a[0,1]),
                       'done':bool(t>=T-1e-10)}
    return states


def run_preview(run,stage,step,out):
    if not math.isfinite(step) or step <= 0 or step>0.2:
        raise ValueError('step must be in (0, 0.2] seconds')
    manifest,trajs=load_stage(run,stage)
    horizon=max(float(dt.sum()) for _,dt in trajs.values())
    times=np.linspace(0.,horizon,max(2,math.ceil(horizon/step)+1))
    out.mkdir(parents=True,exist_ok=True)
    csvpath=out/f'{stage}_D96A_REFERENCE_READ_ONLY.csv'
    columns=['stage','t_s','robot','x_m','y_m','vx_m_s','vy_m_s','ax_m_s2','ay_m_s2','done']
    min_sep=float('inf')
    max_speed={r:0. for r in ROBOTS}
    max_accel={r:0. for r in ROBOTS}
    with csvpath.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=columns)
        writer.writeheader()
        for t in times:
            states=sample(trajs,float(t))
            for robot,st in states.items():
                writer.writerow({'stage':stage,'t_s':f'{t:.9f}','robot':robot,**st})
                max_speed[robot]=max(max_speed[robot],math.hypot(st['vx_m_s'],st['vy_m_s']))
                max_accel[robot]=max(max_accel[robot],math.hypot(st['ax_m_s2'],st['ay_m_s2']))
            for i in range(3):
                for j in range(i+1,3):
                    x,y=states[ROBOTS[i]],states[ROBOTS[j]]
                    min_sep=min(min_sep,math.hypot(x['x_m']-y['x_m'],x['y_m']-y['y_m']))
    report={'stage':stage,'status':'READ_ONLY_REPLAY_PASSED','ros_executable':False,
            'commands_sent':0,'duration_s':horizon,'samples':len(times),
            'robot_active_durations_s':{r:float(trajs[r][1].sum()) for r in ROBOTS},
            'sampled_min_separation_m':min_sep,'sampled_peak_speed_m_s':max_speed,
            'sampled_peak_accel_m_s2':max_accel,
            'preflight_certified_lower_m':manifest['separation']['conservative_lower_m'],
            'csv':str(csvpath),'note':'Sampled extrema are informational; D9.5 envelope provides conservative validity. No ROS commands.'}
    (out/f'{stage}_D96A_report.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run',required=True,type=Path)
    ap.add_argument('--stage',choices=STAGES)
    ap.add_argument('--step',type=float,default=0.02)
    args=ap.parse_args()
    out=args.run/'d96a_readonly'
    reports={}
    for stage in ([args.stage] if args.stage else STAGES):
        r=run_preview(args.run,stage,args.step,out)
        reports[stage]={'duration_s':r['duration_s'],'sampled_min_sep_m':r['sampled_min_separation_m'],'status':r['status']}
        print(stage, r['status'],f"T={r['duration_s']:.3f}s",f"sampled separation={r['sampled_min_separation_m']:.3f}m")
    (out/'d96a_summary.json').write_text(json.dumps({'ros_executable':False,'commands_sent':0,'stages':reports},indent=2)+'\n')
    print('Saved',out)

if __name__=='__main__':main()
