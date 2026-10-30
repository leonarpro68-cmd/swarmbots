#!/usr/bin/env python3
"""D9.9.16. Analyze logged odometry vs historical quintic Bézier, no ROS needed.

Old D9.9.15 data without orientation: speed & position only; acceleration not reported.
New D9.9.16 data: body-frame velocity rotated to odom frame before differentiation.
"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

STAGES = ('pickup1','delivery1','pickup2','delivery2')
COLORS = ('tab:blue','tab:green','tab:red')
TITLES = ('Recogida 1','Entrega 1','Recogida 2','Entrega 2')


def reference(path, rid, t):
    with np.load(path) as z:
        P = np.asarray(z[f'R{rid}_controls'],float)
        D = np.asarray(z[f'R{rid}_durations_s'],float)
    edges = np.r_[0.,np.cumsum(D)]
    tc = np.clip(t,0.,edges[-1])
    ix = np.searchsorted(edges[1:],tc,side='right')
    ix = np.minimum(ix,len(D)-1)
    u = np.clip((tc-edges[ix])/D[ix],0.,1.)
    def bez(q,deg):
        from math import comb
        return sum(comb(deg,k)*(u**k * (1-u)**(deg-k))[:,None]*q[:,k,:] for k in range(deg+1))
    pos = bez(P[ix],5)
    vel = bez(5*np.diff(P[ix],axis=1)/D[ix,None,None],4)
    acc = bez(20*np.diff(P[ix],n=2,axis=1)/(D[ix,None,None]**2),3)
    return pos, np.linalg.norm(vel,axis=1), np.linalg.norm(acc,axis=1), edges[-1]


def load_records(path):
    result = {f'summit{i}':[] for i in range(3)}
    for line in path.open(encoding='utf-8'):
        if not line.strip(): continue
        obj=json.loads(line)
        r=obj.get('robot')
        if r in result: result[r].append(obj)
    return result


def dedup(records, st, en):
    selected={}
    for r in records:
        t=float(r['clock_s'])
        if st<=t<=en and np.isfinite(t): selected[t]=r
    return [selected[t] for t in sorted(selected)]


def acceleration(records, t):
    # Orientation is mandatory: Odometry.twist is normally in child_frame_id.
    if len(records)<5 or not all(all(k in r for k in ('qx','qy','qz','qw')) for r in records):
        return None
    q=np.array([[r['qx'],r['qy'],r['qz'],r['qw']] for r in records],float)
    norm=np.linalg.norm(q,axis=1)
    if np.any(norm<1e-10): return None
    q=q/norm[:,None]
    x,y,z,w=q.T
    yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
    v=np.array([[r['vx_m_s'],r['vy_m_s']] for r in records],float)
    # Rotation from child/base xy to odom xy; appropriate for planar motion.
    cg=np.cos(yaw); sg=np.sin(yaw)
    vg=np.column_stack([cg*v[:,0]-sg*v[:,1], sg*v[:,0]+cg*v[:,1]])
    # Central difference; smooth only for a more readable visualization, not metrics.
    if np.min(np.diff(t))<=1e-8: return None
    ag=np.gradient(vg,t,axis=0,edge_order=2)
    return np.linalg.norm(ag,axis=1)


def scores(y,ref,prefix):
    d=y-ref
    return {f'{prefix}_mae':float(np.mean(np.abs(d))),f'{prefix}_rmse':float(np.sqrt(np.mean(d*d))),
            f'{prefix}_p95_abs':float(np.percentile(np.abs(d),95)),
            f'{prefix}_max_abs':float(np.max(np.abs(d)))}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--odom-jsonl',required=True)
    p.add_argument('--stage-windows',required=True)
    p.add_argument('--reference-dir',default='scripts/d996_reference')
    p.add_argument('--out-dir',default='scripts/resultados/d9916_analysis')
    p.add_argument('--accel-limit',type=float,default=1.0)
    a=p.parse_args()
    root=Path(a.out_dir).resolve(); root.mkdir(parents=True,exist_ok=True)
    odom=load_records(Path(a.odom_jsonl))
    windows={w['stage']:w for w in json.loads(Path(a.stage_windows).read_text())['stages']}
    fig,axes=plt.subplots(4,2,figsize=(16,18),layout='constrained')
    rows=[]
    for si,stage in enumerate(STAGES):
        w=windows[stage]; start=float(w['start_time_s'])
        # Use start + planned duration, excluding post-finish settling extension.
        dur=float(w['max_duration_s']); end=start+dur
        ref_path=Path(a.reference_dir)/f'{stage}_d994_NON_EXECUTABLE.npz'
        if not ref_path.exists():
            ref_path=Path(w['trajectory'])
        if not ref_path.exists(): raise FileNotFoundError(ref_path)
        for rid in range(3):
            name=f'summit{rid}'; samples=dedup(odom[name],start,end)
            if len(samples)<3: raise RuntimeError(f'{stage}/{name}: fewer than 3 samples')
            t=np.array([float(r['clock_s'])-start for r in samples]); pos=np.array([[r['x_m'],r['y_m']] for r in samples],float)
            speed=np.array([r['speed_m_s'] for r in samples],float)
            pref,sref,aref,rdur=reference(ref_path,rid,t)
            err=np.linalg.norm(pos-pref,axis=1)
            row={'stage':stage,'robot':name,'n_samples':len(t),
                 'duration_s':rdur,**scores(speed,sref,'speed_m_s'),
                 'spatial_mean_m':float(np.mean(err)), 'spatial_rmse_m':float(np.sqrt(np.mean(err*err))),
                 'spatial_p95_m':float(np.percentile(err,95)), 'spatial_max_m':float(np.max(err))}
            axv,axa=axes[si]; c=COLORS[rid]
            axv.plot(t,sref,color=c,lw=1.7,label=f'Summit {rid} teórica')
            axv.plot(t,speed,color=c,lw=1.0,linestyle='--',label=f'Summit {rid} odometría')
            axa.plot(t,aref,color=c,lw=1.7,label=f'Summit {rid} teórica')
            areal=acceleration(samples,t)
            if areal is None:
                row['acceleration_status']='NOT_MEASURABLE_NO_ORIENTATION_OR_VALID_TIMES'
            else:
                row['acceleration_status']='ESTIMATED_ODOM_GLOBAL_FRAME'
                row.update(scores(areal,aref,'acceleration_m_s2'))
                row['acceleration_peak_m_s2']=float(np.max(areal))
                row['acceleration_fraction_over_limit']=float(np.mean(areal>a.accel_limit))
                axa.plot(t,areal,color=c,lw=0.85,ls='--',alpha=.7,label=f'Summit {rid} odometría')
            rows.append(row)
        for ax,title in zip(axes[si],(f'{TITLES[si]} · Rapidez (m/s)',f'{TITLES[si]} · Aceleración |a| (m/s²)')):
            ax.set_title(title,fontweight='bold'); ax.set_xlabel('Tiempo de trayectoria (s)');ax.grid(alpha=.25)
        axes[si,0].set_ylabel('Rapidez (m/s)'); axes[si,1].set_ylabel('|a| (m/s²)')
        axes[si,0].axhline(1.2,color='gray',ls=':')
        axes[si,1].axhline(a.accel_limit,color='gray',ls=':')
    for ax in axes[0]: ax.legend(loc='upper right',fontsize=7,ncol=2)
    fig.suptitle('SwarmBots · validación cuantitativa D9.9.16',fontweight='bold',fontsize=17)
    fig.savefig(root/'comparacion_cuantitativa.png',dpi=170);plt.close(fig)
    fields=list(dict.fromkeys(k for row in rows for k in row))
    with (root/'metricas.csv').open('w',newline='',encoding='utf-8') as f:
        wr=csv.DictWriter(f,fieldnames=fields);wr.writeheader();wr.writerows(rows)
    report={'schema':'swarm.d9916.quantitative.v1','status':'ANALYZED',
            'note':'Acceleration requires quaternion orientation recorded by D9.9.16; acceleration computed in odom frame via planar yaw rotation.',
            'rows':rows}
    (root/'metricas.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print(json.dumps({'status':'ANALYZED','out_dir':str(root),'acceleration_available':all('acceleration_peak_m_s2' in r for r in rows)},ensure_ascii=False))

if __name__=='__main__': main()
