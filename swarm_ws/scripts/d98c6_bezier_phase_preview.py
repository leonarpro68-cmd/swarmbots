#!/usr/bin/env python3
"""D9.8C.6: offline quintic Bezier preview for three D9.8C.5 phases.
NOT a Hybrid V2 or D9.2 timing certificate, and NEVER executable.
"""
import argparse
import json
import math
from pathlib import Path
import numpy as np

ROBOTS=('summit0','summit1','summit2')
ACTIVE={'wave1':('summit0','summit2'),'clear_deposit':('summit0',),'wave2':('summit1',)}
STOPS={'wave1':('summit1',),'clear_deposit':('summit1','summit2'),'wave2':('summit0','summit2')}

def controls_for_path(points):
    """Piecewise linear geometry with quintic smoothstep time law; C2 at joins."""
    p=np.asarray(points,dtype=float)
    if p.ndim!=2 or p.shape[1]!=2 or not np.all(np.isfinite(p)):
        raise ValueError('INVALID_POINTS')
    if len(p)==1:p=np.vstack([p,p])
    if len(p)<2:raise ValueError('EMPTY_PATH')
    out=[]
    for a,b in zip(p[:-1],p[1:]):
        # Bernstein form of 10u^3-15u^4+6u^5 => [0,0,0,1,1,1]
        out.append(np.stack([a,a,a,b,b,b]))
    return np.stack(out)

def evaluate(controls,durations,t):
    ends=np.cumsum(durations)
    i=min(int(np.searchsorted(ends,t,side='right')),len(durations)-1)
    u=np.clip((t-(ends[i]-durations[i]))/durations[i],0,1)
    c=controls[i]
    v=(1-u)*c.copy()+u*c.copy() # allocate
    for k in range(5,0,-1):v=(1-u)*v[:k]+u*v[1:k+1]
    return v[0]

def derivative_bounds(controls,durations):
    d1=5*np.diff(controls,axis=1)/durations[:,None,None]
    d2=20*np.diff(controls,n=2,axis=1)/durations[:,None,None]**2
    return float(np.linalg.norm(d1,axis=2).max()),float(np.linalg.norm(d2,axis=2).max())

def make_phase(phase,paths,initial,step_duration=5.0,clearance=1.15,dt=.05):
    if step_duration<=0 or dt<=0 or clearance<=0:raise ValueError('INVALID_LIMIT')
    ctrl={};dur={};last={};bounds={}
    for r in ROBOTS:
        if r in ACTIVE[phase]:
            pts=paths[r]
            if len(pts)<2:raise ValueError('MISSING_ROUTE:'+r)
            if np.linalg.norm(np.asarray(pts[0])-np.asarray(initial[r]))>1.55:
                raise ValueError('START_MISMATCH:'+r)
            cs=controls_for_path(pts); ds=np.full(len(cs),step_duration)
        else:
            cs=controls_for_path([initial[r]]);ds=np.asarray([step_duration])
        ctrl[r]=cs;dur[r]=ds;last[r]=cs[-1,-1].tolist()
        bounds[r]=derivative_bounds(cs,ds)
    horizon=max(float(np.sum(dur[r])) for r in ROBOTS)
    t=np.arange(0,horizon+dt/2,dt)
    min_dist=float('inf'); pair=None
    for i,a in enumerate(ROBOTS):
        for b in ROBOTS[i+1:]:
            md=min(float(np.linalg.norm(evaluate(ctrl[a],dur[a],tt)-evaluate(ctrl[b],dur[b],tt))) for tt in t)
            conservative=md-(bounds[a][0]+bounds[b][0])*dt/2
            if conservative<min_dist:min_dist=conservative;pair=(a,b)
    vmax=max(x[0] for x in bounds.values());amax=max(x[1] for x in bounds.values())
    result={'phase':phase,'active_robots':list(ACTIVE[phase]),'stationary_robots':list(STOPS[phase]),
      'preview_horizon_s':round(horizon,5),'min_sampled_lipschitz_separation_lower_m':round(min_dist,5),
      'closest_pair':list(pair),'conservative_max_speed_bound_m_s':round(vmax,5),
      'conservative_max_accel_bound_m_s2':round(amax,5),
      'limits_preview_passed':bool(min_dist>=clearance and vmax<=1.2 and amax<=1.),
      'end_points_predicted_m':last,'piece_counts':{r:len(ctrl[r]) for r in ROBOTS}}
    return result,ctrl,dur,last

def build(raw,step_duration=5.,dt=.05):
    if raw.get('status')!='CBS_DISCRETE_THREE_PHASES_READY_NOT_EXECUTABLE':
        raise ValueError('UNCERTIFIED_CBS_INPUT')
    p1=raw['wave1']['paths'];park=raw['clear_deposit']['path'];p2=raw['wave2']['path']
    starts={r:None for r in ROBOTS}
    starts['summit0']=p1['summit0']['waypoints_xy_m'][0]
    starts['summit2']=p1['summit2']['waypoints_xy_m'][0]
    # stationary robot's initial coordinate must be provided by source snapshot
    if 'stationary_positions_xy_m' not in raw:
        raise ValueError('SNAPSHOT_REQUIRED_FOR_STATIONARY_ROBOTS')
    starts['summit1']=raw['stationary_positions_xy_m']['summit1']
    specs={'wave1':{r:p1[r]['waypoints_xy_m'] for r in ACTIVE['wave1']},
           'clear_deposit':{'summit0':park['waypoints_xy_m']},
           'wave2':{'summit1':p2['waypoints_xy_m']}}
    results=[];arrays={};current=starts
    for phase in ('wave1','clear_deposit','wave2'):
        res,cs,ds,end=make_phase(phase,specs[phase],current,step_duration,dt=dt)
        results.append(res)
        for r in ROBOTS:
            arrays[f'{phase}_{r}_controls']=cs[r]
            arrays[f'{phase}_{r}_durations_s']=ds[r]
        current=end
    return results,arrays

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--cbs',type=Path,required=True)
    ap.add_argument('--snapshot',type=Path,required=True)
    ap.add_argument('--output',type=Path)
    ap.add_argument('--segment-duration',type=float,default=8.)
    a=ap.parse_args()
    raw=json.loads(a.cbs.read_text());snapshot=json.loads(a.snapshot.read_text())
    raw['stationary_positions_xy_m']={'summit1':snapshot['robots']['summit1']['odom_xy_m']}
    result={'schema':'swarm.d98c6.offline_bezier_preview.v1','status':'PREVIEW_ONLY_NOT_D92_CERTIFIED',
      'execution_authorized':False,'hybrid_v2_certified':False,'d92_certified':False,
      'cargo_joint_certified':False,'source_cbs':str(a.cbs),'source_snapshot':str(a.snapshot)}
    try:
        stages,arrays=build(raw,a.segment_duration)
        result['phases']=stages
        result['status']='BEZIER_PREVIEW_LIMITS_PASS_NOT_EXECUTABLE' if all(p['limits_preview_passed'] for p in stages) else 'BLOCKED_PREVIEW_LIMITS'
    except (ValueError,KeyError,TypeError) as e:
        result['status']='BLOCKED_INVALID_INPUT';result['issues']=[str(e)];arrays={}
    out=a.output or a.cbs.parent/'delivery1_bezier_three_phases_preview.json'
    out.parent.mkdir(parents=True,exist_ok=True)
    result['controls_npz']=str(out.with_suffix('.npz')) if arrays else None
    out.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    if arrays:np.savez_compressed(out.with_suffix('.npz'),**arrays)
    print(json.dumps({'status':result['status'],'output':str(out),'controls_npz':result['controls_npz']},indent=2))
    return 0 if result['status']=='BEZIER_PREVIEW_LIMITS_PASS_NOT_EXECUTABLE' else 2
if __name__=='__main__':raise SystemExit(main())
