#!/usr/bin/env python3
"""D9.8C.7: offline three-wave D9.2 phase-warp verification, NOT executable.
Reuses D9.2 analytical conservative verifier; only optimizes a uniform scale.
Does NOT certify Hybrid V2 obstacle clearance or payload dynamics.
"""
import argparse, json, hashlib
from pathlib import Path
import numpy as np
from d98c7_d92_verifier import verify
PHASES=('wave1','clear_deposit','wave2')
ROBOTS=('summit0','summit1','summit2')
def analyze(preview, npz, step=.08, scales=(1., .95, .9, .85, .8)):
    if preview.get('status')!='BEZIER_PREVIEW_LIMITS_PASS_NOT_EXECUTABLE': raise ValueError('BAD_PREVIEW_STATUS')
    if preview.get('execution_authorized') is not False:raise ValueError('UNEXPECTED_AUTHORIZATION')
    if not (0<step<=.1):raise ValueError('INVALID_STEP')
    out=[]; accepted={}
    for phase in PHASES:
        cs=[];ns=[];base=[]
        for r in ROBOTS:
            key=f'{phase}_{r}'
            c=np.asarray(npz[f'{key}_controls'],dtype=float)
            d=np.asarray(npz[f'{key}_durations_s'],dtype=float)
            if c.ndim!=3 or c.shape[1:]!=(6,2) or len(c)!=len(d) or not np.all(np.isfinite(c)) or not np.all(np.isfinite(d)) or np.any(d<=0):raise ValueError('BAD_CONTROLS:'+key)
            if not np.allclose(d,d[0]):raise ValueError('NONUNIFORM_DURATIONS_NOT_SUPPORTED:'+key)
            if np.any(np.linalg.norm(c[1:,0]-c[:-1,-1],axis=1)>1e-6):raise ValueError('BROKEN_PATH:'+key)
            cs.append(c);ns.append(len(c));base.append(float(d.sum()))
        if phase=='wave1' and ('summit1' not in preview['phases'][0]['stationary_robots']):raise ValueError('MISSING_STATIONARY')
        baseline_ok,baseline=verify(cs,ns,base,[(0.,0.)]*3,1.2,1.,1.15,step)
        phase_report={'phase':phase,'input_durations_s':dict(zip(ROBOTS,base)), 'baseline_verified':baseline_ok,
            'baseline_certificate':baseline if baseline_ok else None,'attempts':[]}
        selected=None
        if baseline_ok:
            for k in scales:
                if not 0<k<=1:raise ValueError('BAD_SCALE')
                trial=[v*k for v in base]
                ok,cert=verify(cs,ns,trial,[(0.,0.)]*3,1.2,1.,1.15,step)
                phase_report['attempts'].append({'scale':k,'pass':bool(ok),'reason':None if ok else cert.get('reason')})
                if ok:selected=(trial,cert,k)
        if selected:
            times,cert,k=selected
            phase_report.update(status='D92_CONSERVATIVE_PHASE_MODEL_PASS',retimed_durations_s=dict(zip(ROBOTS,times)),
                scale=k,certificate=cert,time_saving_vs_conservative_preview_s=round(max(base)-max(times),6),
                harmonic_coefficients={r:[0.,0.] for r in ROBOTS})
            accepted[phase]=(times,cs)
        else:phase_report['status']='BLOCKED_D92_CONSERVATIVE_CHECK'
        out.append(phase_report)
    return out,accepted

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--preview',required=True,type=Path);p.add_argument('--npz',type=Path)
    p.add_argument('--output',type=Path);p.add_argument('--step',type=float,default=.08)
    a=p.parse_args(); npz_path=a.npz or a.preview.with_suffix('.npz')
    preview=json.loads(a.preview.read_text());digest=hashlib.sha256(npz_path.read_bytes()).hexdigest()
    with np.load(npz_path,allow_pickle=False) as npz:
        results,accepted=analyze(preview,npz,a.step)
    ok=len(accepted)==3
    report={'schema':'swarm.d98c7.phase_d92_offline.v1','status':'D92_PHASE_MODEL_THREE_PHASES_PASS_NOT_EXECUTABLE' if ok else 'BLOCKED_OFFLINE_CERTIFICATE',
      'execution_authorized':False,'hybrid_v2_obstacle_certified':False,'payload_joint_certified':False,
      'robot_odom_fresh':False,'release_verified':False,'d94_d95_live_certified':False,
      'source_preview':str(a.preview),'source_npz':str(npz_path),'source_sha256':digest,'phases':results,
      'limitations':['D9.2 conservative verifier reused, but no harmonic coefficient search (a=b=0).',
       'Uniform phase scale only; no hybrid V2 obstacle certification or restored Gazebo state.',
       'No plan from fresh odometry and no authorization to actuate or release cargo.']}
    dest=a.output or a.preview.parent/'delivery1_d92_three_phases_offline.json'
    dest.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({'status':report['status'],'output':str(dest),'phases':[{'phase':x['phase'],'status':x['status'],'scale':x.get('scale')} for x in results]},indent=2))
    return 0 if ok else 2
if __name__=='__main__':raise SystemExit(main())
