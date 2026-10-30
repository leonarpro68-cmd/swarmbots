#!/usr/bin/env python3
"""D.9.3/4: audit D9.2 smooth trajectories and export NON-EXECUTABLE previews.

No ROS dependencies, no command generation or network I/O. Reads the original
D8 Bézier controls, D9.1/2 offline reports and verifies each D9.2 candidate
with the original conservative verifier before exporting plots and CSV tables.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from d9_offline_retime import STAGES, active, load
from d92_smooth_retime import verify, values_for_robot


def run_stage(run, stage, output, step=.04):
    source = json.loads((run/'d92_offline_report.json').read_text())
    item = source.get('stages', {}).get(stage)
    if not item:
        raise ValueError(f'{stage}: absent from D.9.2 report')
    if item.get('status') != 'CERTIFIED_CONSERVATIVE_OFFLINE':
        raise ValueError(f'{stage}: D.9.2 status is not certified')
    cs, dt = load(run, stage)
    ns = [active(c) for c in cs]
    if ns != item['active_pieces']:
        raise ValueError(f'{stage}: active piece count does not match report')
    Ts = np.asarray(item['duration_s'], dtype=float)
    coeff = np.asarray(item['harmonics'], dtype=float)
    if Ts.shape != (3,) or coeff.shape != (3,2) or not np.isfinite(Ts).all() or not np.isfinite(coeff).all():
        raise ValueError('Invalid durations / coefficients')
    valid, certificate = verify(cs, ns, Ts, coeff, 1.2, 1., 1.15, step)
    if not valid:
        raise ValueError(f'{stage}: re-verification failed: {certificate}')
    if abs(certificate['makespan_s']-item['smooth_makespan_s']) > 1e-5:
        raise ValueError(f'{stage}: result duration mismatch')
    # Exact forward evaluation of smooth D9.2 reference. Never converted to commands.
    horizon = float(max(Ts))
    t = np.linspace(0, horizon, max(2,int(np.ceil(horizon/step))+1))
    # D8 original active trajectory, with per-robot completion and hold.
    origT = np.array(ns)*dt
    original = [values_for_robot(c,n,float(T),0.,0.,t) for c,n,T in zip(cs,ns,origT)]
    smooth = [values_for_robot(c,n,float(T),float(a),float(b),t)
              for c,n,T,(a,b) in zip(cs,ns,Ts,coeff)]
    output.mkdir(parents=True, exist_ok=True)
    csv_path = output/f'{stage}_reference_NON_EXECUTABLE.csv'
    with csv_path.open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['stage','robot','time_s','x_m','y_m','vx_m_s','vy_m_s','ax_m_s2','ay_m_s2','speed_m_s','accel_m_s2'])
        for i,(p,v,a) in enumerate(smooth):
            for j,tt in enumerate(t):
                writer.writerow([stage,i,float(tt),*map(float,p[j]),*map(float,v[j]),*map(float,a[j]),
                                 float(np.linalg.norm(v[j])),float(np.linalg.norm(a[j]))])
    fig_path = output/f'{stage}_d8_vs_d92.png'
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(2,3,figsize=(15,7),sharex=True)
        for i in range(3):
            for k,(label,component,lim) in enumerate((('Speed',1,1.2),('Acceleration',2,1.0))):
                ax=axes[k,i]
                old=np.linalg.norm(original[i][component],axis=1)
                new=np.linalg.norm(smooth[i][component],axis=1)
                ax.plot(t,old,'--',label='D.8 reference',lw=1.2)
                ax.plot(t,new,label='D.9.2 reference',lw=1.3)
                ax.axhline(lim,ls=':',color='black',alpha=.65,label='Limit')
                ax.set_title(f'Summit {i} — {label}')
                ax.set_ylabel('m/s' if k==0 else 'm/s²')
                if k==1:ax.set_xlabel('Stage time (s)')
                ax.grid(alpha=.25)
                if i==0 and k==0:ax.legend(fontsize=8)
        fig.suptitle(f'{stage}: D.8 vs D.9.2 (offline analytic references, NOT odometry)')
        fig.tight_layout()
        fig.savefig(fig_path,dpi=155)
        plt.close(fig)
    except ImportError:
        fig_path=None
    # Existing timed_bezier.v1 is a single common dt_s and cannot faithfully
    # represent the harmonic phase warp. Explicitly block ROS command emission.
    audit={
        'stage':stage,'status':'VERIFIED_OFFLINE_PREVIEW_ONLY',
        'ros_executable':False,
        'protocol_target':'timed_bezier.v1',
        'protocol_compatibility':'INCOMPATIBLE_WITH_EXACT_HARMONIC_RETIMING',
        'why':'timed_bezier.v1 carries a single common dt_s and fixed quintic control polygons; the per-robot smooth nonlinear time warp is not exactly representable.',
        'original_active_makespan_s':float(max(origT)),
        'd91_makespan_s':float(item['d91_makespan_s']),
        'd92_makespan_s':horizon,
        'certification':certificate,
        'preview_csv':str(csv_path), 'plot':str(fig_path) if fig_path else None,
        'important':'Do not publish this CSV as timed_bezier commands; no error-bounded converter or full execution certificate exists yet.'
    }
    (output/f'{stage}_audit.json').write_text(json.dumps(audit,indent=2)+'\n')
    return audit


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',required=True,type=Path)
    p.add_argument('--stage',choices=STAGES)
    p.add_argument('--output-dir',type=Path)
    p.add_argument('--step',type=float,default=.04)
    args=p.parse_args()
    if not (0 < args.step <= .04): p.error('--step must be in (0, 0.04]')
    out=args.output_dir or args.run/'d93_preview'
    results={}
    for stage in [args.stage] if args.stage else STAGES:
        result=run_stage(args.run,stage,out,args.step)
        results[stage]=result
        print(f"{stage}: {result['status']}, D9.2={result['d92_makespan_s']:.3f}s; protocol=INCOMPATIBLE")
    (out/'d93_summary.json').write_text(json.dumps({'stages':results,'ros_executable':False},indent=2)+'\n')
    print('Output:',out)

if __name__=='__main__': main()
