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

def smooth_uniform(t, values, window_s=0.30):
    """Resample then smooth; never differentiate irregular noisy raw odometry."""
    if len(t) < 4:
        return t,values
    order=np.argsort(t);t=np.asarray(t)[order];values=np.asarray(values)[order]
    t, unique=np.unique(t,return_index=True);values=values[unique]
    if len(t)<4:return t,values
    dt=max(.02,min(.08,float(np.median(np.diff(t)))))
    grid=np.arange(t[0],t[-1]+dt/2,dt)
    interp=np.interp(grid,t,values)
    span=max(3,int(round(window_s/dt)));span+=1-span%2
    if span >=len(grid):span=max(1,len(grid)//2*2-1)
    if span<=1:return grid,interp
    kernel=np.ones(span)/span
    padded=np.pad(interp,(span//2,span//2),mode='edge')
    return grid,np.convolve(padded,kernel,mode='valid')

def onset_time(t, speed, threshold=.08, hold_s=.30):
    """First sustained movement; otherwise report no detectable movement."""
    if len(t)<2:return None
    for i in range(len(t)):
        if speed[i] < threshold:continue
        end=np.searchsorted(t,t[i]+hold_s,side='left')
        if end>=len(t):break
        if np.mean(speed[i:end+1]>=threshold)>=.75:return float(t[i])
    return None

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--odom-jsonl',required=True)
    ap.add_argument('--stage-windows',required=True)
    ap.add_argument('--output',required=True)
    ap.add_argument('--alignment-report',default=None)
    ap.add_argument('--nav-speed-limit',type=float,default=2.)
    ap.add_argument('--nav-accel-limit',type=float,default=2.)
    ap.add_argument('--movement-threshold',type=float,default=.08)
    ap.add_argument('--smoothing-window-s',type=float,default=.30)
    args=ap.parse_args()
    data={f'summit{i}':[] for i in range(3)}
    for line in Path(args.odom_jsonl).read_text().splitlines():
        if not line.strip():continue
        record=json.loads(line)
        if record.get('robot') in data:data[record['robot']].append(record)
    stages={r['stage']:r for r in json.loads(Path(args.stage_windows).read_text())['stages']}
    fig,axes=plt.subplots(4,2,figsize=(16,17))
    report={'schema':'supermarket.v3_plot_alignment.v2',
            'method':'first_sustained_motion_per_robot; onset aligned to theoretical onset',
            'warning':'onset alignment compares motion profiles; offsets include coordinator startup and possible tracking lag; not clock calibration',
            'stages':{}}
    for si,stage in enumerate(STAGES):
        entry=stages[stage];start=float(entry['start_time_s']);end=float(entry['end_time_s'])
        if entry.get('time_model')!='common_collective_successive_v1' or not entry.get('clock_json'):
            raise ValueError(f'{stage}: missing V3 clock')
        report['stages'][stage]={}
        for i in range(3):
            robot=f'summit{i}'
            x,v,a=theory(entry['trajectory'],entry['clock_json'],i)
            axes[si,0].plot(x,v,color=COLORS[i],label=f'{robot} teoría',lw=1.9)
            axes[si,1].plot(x,a,color=COLORS[i],label=f'{robot} teoría',lw=1.9)
            rows=sorted((r for r in data[robot] if start<=float(r['clock_s'])<=end),key=lambda r:float(r['clock_s']))
            diag={'samples':len(rows),'recorded_window_s':[start,end]}
            report['stages'][stage][robot]=diag
            if len(rows)<5:
                diag['status']='INSUFFICIENT_SAMPLES';continue
            tx=np.array([float(r['clock_s'])-start for r in rows]);
            rx=np.array([float(r.get('vx_m_s',0)) for r in rows]);ry=np.array([float(r.get('vy_m_s',0)) for r in rows]);
            speed=np.array([float(r.get('speed_m_s',0)) for r in rows])
            tt,sv=smooth_uniform(tx,speed,args.smoothing_window_s)
            _,sx=smooth_uniform(tx,rx,args.smoothing_window_s)
            _,sy=smooth_uniform(tx,ry,args.smoothing_window_s)
            if len(tt)<5:
                diag['status']='INSUFFICIENT_RESAMPLED_SAMPLES';continue
            measured_onset=onset_time(tt,sv,args.movement_threshold)
            theory_onset=onset_time(x,v,args.movement_threshold)
            if measured_onset is None or theory_onset is None:
                diag['status']='NO_DETECTABLE_ONSET';continue
            lag=measured_onset-theory_onset
            tplot=tt-lag
            diag.update({'status':'ALIGNED_BY_MOTION','measured_onset_after_event_s':round(measured_onset,4),
                         'theory_onset_s':round(theory_onset,4),'applied_shift_s':round(lag,4),
                         'threshold_m_s':args.movement_threshold})
            # Only show the measured portion relevant to theoretical stage horizon.
            keep=(tplot>=-0.5)&(tplot<=float(x[-1])+1)
            axes[si,0].plot(tplot[keep],sv[keep],'--',color=COLORS[i],alpha=.85,
                            label=f'{robot} odom (alineada)')
            acceleration=np.hypot(np.gradient(sx,tt),np.gradient(sy,tt))
            axes[si,1].plot(tplot[keep],acceleration[keep],'--',color=COLORS[i],alpha=.68,
                            label=f'{robot} odom (filtrada)')
        for ax,limit,name in zip(axes[si],(args.nav_speed_limit,args.nav_accel_limit),('Rapidez (m/s)','Aceleración (m/s²)')):
            ax.axhline(limit,color='0.3',ls=':',label=f'Límite {limit}')
            ax.set_title(f'{stage}: {name}');ax.set_xlabel('Tiempo desde inicio V3 alineado (s)')
            ax.set_xlim(left=-0.5);ax.grid(alpha=.25)
        if si==0:
            axes[si,0].legend(fontsize=7,loc='upper right')
            axes[si,1].legend(fontsize=7,loc='upper right')
    fig.suptitle('SwarmBots · V3 teoría vs odometría · inicio de movimiento alineado',fontsize=16)
    fig.text(.5,.01,'Odometría alineada por inicio sostenido de movimiento (no por reloj); aceleración estimada con velocidad filtrada.',
             fontsize=9,ha='center')
    fig.tight_layout(rect=(0,.02,1,.97))
    out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True);fig.savefig(out,dpi=160);plt.close(fig)
    diagnostic=Path(args.alignment_report) if args.alignment_report else out.with_name('alineacion_temporal.json')
    diagnostic.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({'status':'PLOT_ALIGNED','output':str(out),'alignment_report':str(diagnostic)}))

if __name__=='__main__':main()
