#!/usr/bin/env python3
"""Comparacion teórica/odometría D9931 V3. Respeta el reloj sucesivo REAL.

No modifica el graficador D9915 del modo original.
"""
from pathlib import Path
import argparse,sys,json,math,bisect
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from math import comb
from swarm_behavior.swarm_planning.hybrid_refinement.successive_clock import clock_from_payload
STAGES=('pickup1','delivery1','pickup2','delivery2')
COLORS=('tab:blue','tab:green','tab:red')

def bezier(pts,u):
    n=len(pts)-1
    return sum(comb(n,j)*(1-u)**(n-j)*u**j*pts[j] for j in range(n+1))

def theory(npz,clockpath,robot,step=.05):
    obj=json.loads(Path(clockpath).read_text())
    payload={'model':'common_collective_successive_v1','baseline_scale':obj['uniform_scale'],
             'ramps':obj['ramps'],'duration_original_s':obj['duration_original_s'],
             'duration_successive_s':obj['duration_successive_s']}
    clock=clock_from_payload(payload)
    with np.load(npz,allow_pickle=False) as z:
        ctrl=z[f'R{robot}_controls'];durs=z[f'R{robot}_durations_s']
    times=np.linspace(0,clock.duration_s,max(2,math.ceil(clock.duration_s/step)+1))
    knots=np.r_[0.,np.cumsum(durs)]
    vv=[];aa=[]
    for t in times:
        tau=clock.inverse(float(t))
        k=max(0,min(len(durs)-1,int(bisect.bisect_right(knots,tau)-1)))
        d=durs[k];u=max(0.,min(1.,(tau-knots[k])/d))
        p=ctrl[k]
        v0=bezier(5*np.diff(p,axis=0)/d,u)
        a0=bezier(20*np.diff(p,n=2,axis=0)/(d*d),u)
        q=clock.scale(tau)
        qprime=0.
        for r in clock.ramps:
            if r['start_tau']<=tau<r['end_tau']:
                from swarm_behavior.swarm_planning.hybrid_refinement.successive_clock import smoothstep_prime
                width=r['end_tau']-r['start_tau']
                qprime=(r['to_scale']-r['from_scale'])/width*smoothstep_prime((tau-r['start_tau'])/width)
                break
        vel=v0*q
        acc=a0*q*q+v0*q*qprime
        vv.append(np.linalg.norm(vel));aa.append(np.linalg.norm(acc))
    return times,np.array(vv),np.array(aa)

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--odom-jsonl',required=True)
    ap.add_argument('--stage-windows',required=True)
    ap.add_argument('--output',required=True)
    ap.add_argument('--nav-speed-limit',type=float,default=2.)
    ap.add_argument('--nav-accel-limit',type=float,default=2.)
    args=ap.parse_args()
    data={f'summit{i}':[] for i in range(3)}
    for line in Path(args.odom_jsonl).read_text().splitlines():
        if not line.strip():continue
        record=json.loads(line)
        if record.get('robot') in data:data[record['robot']].append(record)
    stages={r['stage']:r for r in json.loads(Path(args.stage_windows).read_text())['stages']}
    fig,axes=plt.subplots(4,2,figsize=(16,18))
    for si,stage in enumerate(STAGES):
        entry=stages[stage];start=float(entry['start_time_s']);end=float(entry['end_time_s'])
        if entry.get('time_model')!='common_collective_successive_v1' or not entry.get('clock_json'):
            raise ValueError(f'{stage}: falta reloj V3')
        for i in range(3):
            robot=f'summit{i}'
            x,v,a=theory(entry['trajectory'],entry['clock_json'],i)
            axes[si,0].plot(x,v,color=COLORS[i],label=f'{robot} teoría',lw=2)
            axes[si,1].plot(x,a,color=COLORS[i],label=f'{robot} teoría',lw=2)
            rows=[r for r in data[robot] if start<=float(r['clock_s'])<=end]
            if len(rows)<3:continue
            tx=np.array([float(r['clock_s'])-start for r in rows]);
            rx=np.array([float(r.get('vx_m_s',0)) for r in rows]);ry=np.array([float(r.get('vy_m_s',0)) for r in rows])
            speed=np.array([float(r.get('speed_m_s',0)) for r in rows])
            axes[si,0].plot(tx,speed,'--',color=COLORS[i],alpha=.7,label=f'{robot} odom')
            dt=np.diff(tx); dt=np.where(np.abs(dt)>1e-9,dt,np.nan)
            acc=np.hypot(np.diff(rx)/dt,np.diff(ry)/dt)
            axes[si,1].plot((tx[1:]+tx[:-1])/2,acc,'--',color=COLORS[i],alpha=.5,label=f'{robot} odom')
        for ax,limit,name in zip(axes[si],(args.nav_speed_limit,args.nav_accel_limit),('Rapidez (m/s)','Aceleración (m/s²)')):
            ax.axhline(limit,color='0.3',ls=':',label=f'Límite {limit}')
            ax.set_title(f'{stage}: {name}')
            ax.set_xlabel('Tiempo de reloj real (s)')
            ax.grid(alpha=.25)
        if si==0:axes[si,0].legend(fontsize=8);axes[si,1].legend(fontsize=8)
    fig.suptitle('SwarmBots MILP · V3 Escalado sucesivo · teoría vs odometría',fontsize=18)
    fig.tight_layout(rect=(0,0,.99,.97))
    Path(args.output).parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(args.output,dpi=160)
    print(json.dumps({'status':'PLOT_READY_V3','output':args.output}))

if __name__=='__main__':main()
