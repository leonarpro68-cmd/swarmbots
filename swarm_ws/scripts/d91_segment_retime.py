#!/usr/bin/env python3
"""D.9.1 offline: piecewise Bézier retiming with C1 velocity and conservative separation.

No ROS commands. Reuses certified D8 control polygons; outputs only JSON.
Independent timing blocks are split ONLY at full stops (zero endpoint tangent).
"""
import argparse
import json
from pathlib import Path
import numpy as np
from d9_offline_retime import STAGES, bezier, load, active, max_bounds, check as d9_check


def blocks(c, n, tol=1e-7):
    """Group adjacent moving pieces where nonzero derivatives require identical dt scaling."""
    if n == 0:
        return []
    out = [[0]]
    for j in range(1, n):
        vleft = 5 * (c[j-1,-1]-c[j-1,-2])
        vright = 5 * (c[j,1]-c[j,0])
        if np.linalg.norm(c[j-1,-1]-c[j,0]) > tol:
            raise ValueError(f'Piezas {j-1}/{j} discontinuas en posición')
        if np.linalg.norm(vleft-vright)>tol:
            raise ValueError(f'Piezas {j-1}/{j} discontinuas en velocidad del protocolo base')
        if np.linalg.norm(vleft)<=tol and np.linalg.norm(vright)<=tol:
            out.append([j])
        else:
            out[-1].append(j)
    return out


def eval_robot(c, n, durations, t):
    if n == 0:
        return c[0,0]
    end=float(np.sum(durations))
    if t>=end:
        return c[n-1,-1]
    q=max(0.,t)
    for j in range(n):
        if q <= durations[j] or j==n-1:
            return bezier(c[j], np.clip(q/durations[j], 0, 1))
        q -= durations[j]
    return c[n-1,-1]


def bounds_piece(c, durations):
    d=5*(c[:,1:]-c[:,:-1])/durations[:,None,None]
    a=20*(c[:,2:]-2*c[:,1:-1]+c[:,:-2])/durations[:,None,None]**2
    return float(np.linalg.norm(d,axis=2).max()),float(np.linalg.norm(a,axis=2).max())


def check_piecewise(cs, ns, ds, vmax=1.2, amax=1., sep=1.15, step=0.1):
    vs=[]; aa=[]; totals=[]
    for c,n,d in zip(cs,ns,ds):
        if n==0:
            vs.append(0.); aa.append(0.); totals.append(0.);continue
        if len(d)!=n or np.any(np.asarray(d)<=0):
            return False, {'reason':'bad_durations'}
        v,a=bounds_piece(c[:n],np.asarray(d))
        if v>vmax+1e-9 or a>amax+1e-9:
            return False,{'reason':'dynamics','v':v,'a':a}
        vs.append(v);aa.append(a);totals.append(float(np.sum(d)))
    T=max(totals)
    ts=np.linspace(0,T,max(2,int(np.ceil(T/step))+1))
    h=float(ts[1]-ts[0]); low=float('inf'); sampled=float('inf')
    for t in ts:
        pts=[eval_robot(c,n,d,t) for c,n,d in zip(cs,ns,ds)]
        for i in range(3):
            for j in range(i+1,3):
                dist=float(np.linalg.norm(pts[i]-pts[j]))
                lower=dist-(vs[i]+vs[j])*h/2
                low=min(low,lower);sampled=min(sampled,dist)
                if lower+1e-9<sep:
                    return False,{'reason':'separation','certified_lower_m':low,'sampled_m':sampled}
    return True,{'makespan_s':T,'durations_s':totals,'speed_bounds_m_s':vs,
                 'accel_bounds_m_s2':aa,'certified_lower_m':low,'sampled_m':sampled,
                 'sample_interval_s':h}


def solve(cs,dt, vmax=1.2,amax=1.,sep=1.15,step=0.1, levels=5, passes=3):
    ns=[active(c) for c in cs]
    groups=[blocks(c,n) for c,n in zip(cs,ns)]
    ds=[np.full(n,dt) for n in ns]
    valid,base=check_piecewise(cs,ns,ds,vmax,amax,sep,step)
    result={'active_pieces':ns,'blocks':groups,'baseline_valid':valid,'baseline':base,
            'original_makespan_s':max(n*dt for n in ns)}
    if not valid:
        return dict(result,status='UNVERIFIED_BASELINE')
    # Each block has one constant time scale: at joints with nonzero velocity,
    # independent segment durations violate C1; this restriction preserves C1.
    lower=[]
    for c,n,bs in zip(cs,ns,groups):
        row=[]
        for inds in bs:
            v,a=max_bounds(c[inds],dt,len(inds))
            row.append(max(v/vmax,np.sqrt(a/amax),0.001))
        lower.append(row)
    if any(s>1+1e-9 for r in lower for s in r):
        return dict(result,status='UNVERIFIED_BASELINE',reason='base dynamic envelope exceeded')
    scales=[[1.]*len(x) for x in groups]
    def durations(sc):
        return [np.array([dt*sc[i][next(k for k,g in enumerate(groups[i]) if j in g)] for j in range(n)])
                for i,n in enumerate(ns)]
    best=base
    trials=0; accepts=0
    # Coordinate descent: deterministic and intentionally conservative.
    for _ in range(passes):
        changed=False
        for i in range(3):
            for k,_ in enumerate(groups[i]):
                current=scales[i][k]
                for s in np.linspace(lower[i][k],current,levels):
                    if s>=current-1e-9:continue
                    trial=[r.copy() for r in scales]
                    trial[i][k]=float(s)
                    trials+=1
                    ok,info=check_piecewise(cs,ns,durations(trial),vmax,amax,sep,step)
                    if ok and info['makespan_s']<=best['makespan_s']+1e-9:
                        scales=trial;best=info;accepts+=1;changed=True
                        break
        if not changed:break
    actual=durations(scales)
    # Report full scales to reproduce timing offline, without overwriting protocol.
    final_ok,final=check_piecewise(cs,ns,actual,vmax,amax,sep,step)
    if not final_ok:raise AssertionError('Optimized result lost verification')
    result.update(status='CERTIFIED_CONSERVATIVE_OFFLINE',block_scales=scales,
                  piece_durations_s=[d.tolist() for d in actual],certificate=final,
                  optimized_makespan_s=final['makespan_s'],
                  saved_s=base['makespan_s']-final['makespan_s'],
                  improvement_pct=100*(base['makespan_s']-final['makespan_s'])/base['makespan_s'],
                  trials=trials,accepted=accepts,
                  note='Only position/velocity-continuous piece retiming and conservative inter-robot separation. No full Hybrid V2/ROS execution certificate.')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',required=True,type=Path)
    p.add_argument('--stage',choices=STAGES)
    p.add_argument('--output',type=Path)
    p.add_argument('--max-speed',type=float,default=1.2)
    p.add_argument('--max-accel',type=float,default=1.0)
    p.add_argument('--min-separation',type=float,default=1.15)
    p.add_argument('--step',type=float,default=.1)
    p.add_argument('--levels',type=int,default=5)
    p.add_argument('--passes',type=int,default=3)
    a=p.parse_args()
    if any(not np.isfinite(x) or x<=0 for x in [a.max_speed,a.max_accel,a.min_separation,a.step]) or a.levels<2 or a.passes<1:p.error('invalid numerical settings')
    stages={}
    d9_path=a.run/'d9_offline_report.json'
    try:
        d9_previous=json.loads(d9_path.read_text()).get('stages',{}) if d9_path.exists() else {}
    except (OSError,ValueError):
        d9_previous={}
    for st in ([a.stage] if a.stage else STAGES):
        cs,dt=load(a.run,st)
        r=solve(cs,dt,a.max_speed,a.max_accel,a.min_separation,a.step,a.levels,a.passes)
        if st in d9_previous:
            prev=d9_previous[st]
            r['d9_constant_per_robot_duration_s']=prev.get('optimized_duration_s')
            r['d9_constant_per_robot_status']=prev.get('status')
        stages[st]=r
        print(st, r['status'],f"baseline={r['original_makespan_s']:.3f}s",f"optimized={r.get('optimized_makespan_s',float('nan')):.3f}s")
    output=a.output or (a.run/'d91_offline_report.json')
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps({'method':'D9.1 offline independent C1-safe block retiming', 'stages':stages},indent=2)+'\n')
    print('Report:',output)
if __name__=='__main__':main()
