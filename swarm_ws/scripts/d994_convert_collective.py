#!/usr/bin/env python3
"""D9.9.4 — Quintic time-Bézier preview from D9.9 common-q clock (offline).

Computes one monotonically increasing common clock q->t, with independent
stop times coming from the COMMON clock (not independent robot optimization).
Converts each moving robot's exact time-parametric path into piecewise quintic
Hermite Bézier segments. This conversion is approximative, so errors are
explicitly bounded via first/second/third derivative Lipschitz certificates.

Read-only, NON EXECUTABLE. Map clearance is NOT recertified here.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from d9_offline_retime import load, active, STAGES
from d99_collective_phase_retime import solve, _time_at_q, _s_and_d, _bezier_sample
from d94_convert_offline import hermite_quintic, converted_values, polynomial_bounds


def sha256(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for chunk in iter(lambda:f.read(1<<20),b''):h.update(chunk)
    return h.hexdigest()


def source_values(c,n,dt,events,targets,q):
    q=np.asarray(q,dtype=float)
    p,d1,d2=_bezier_sample(c,n,q)
    s,ds=_s_and_d(q,events,targets)
    v=d1/(dt*s[:,None])
    a=d2/(dt*dt*s[:,None]**2)-d1*ds[:,None]/(dt*dt*s[:,None]**3)
    return p,v,a


def inverted_q(t,dt,events,targets,end,iterations=54):
    """Monotone invert t(q). No interpolation drift in source evaluation."""
    t=np.asarray(t,dtype=float)
    lo=np.zeros_like(t);hi=np.full_like(t,float(end))
    for _ in range(iterations):
        mid=(lo+hi)*0.5
        t_mid=_time_at_q(mid,dt,events,targets)
        lo=np.where(t_mid<t,mid,lo)
        hi=np.where(t_mid>=t,mid,hi)
    q=(hi+lo)*0.5
    return np.where(t<=0,0.,np.where(t>=_time_at_q(np.array([end]),dt,events,targets)[0],float(end),q))


def _scale_bounds(events,targets):
    # This D9.9 implementation only permits monotone non-increasing scale.
    candidates=[1.]+[float(x) for x in targets]
    min_s=min(candidates);max_s=max(candidates)
    prev=1.;D1=D2=0.
    for (e,w),after in zip(events,targets):
        dz=after-prev
        D1=max(D1,1.5*abs(dz)/w) # max smoothstep derivative
        D2=max(D2,6.*abs(dz)/(w*w))
        prev=after
    return min_s,max_s,D1,D2


def source_jerk_bound(c,n,dt,events,targets):
    """Conservative jerk envelope of exactly time-warped original Béziers.

    x'''=P'''(q)*qdot^3+3P''(q)*qdot*qddot+P'(q)*q''' .
    Bounds include s'' of each smoothstep, and remain valid at joins by C2.
    """
    smin,smax,d1,d2=_scale_bounds(events,targets)
    vcp=5*np.diff(c[:n],axis=1)
    acp=20*np.diff(c[:n],n=2,axis=1)
    jcp=60*np.diff(c[:n],n=3,axis=1)
    M1=float(np.linalg.norm(vcp,axis=2).max())
    M2=float(np.linalg.norm(acp,axis=2).max())
    M3=float(np.linalg.norm(jcp,axis=2).max())
    bound=(M3/smin**3 + 3*M2*d1/smin**4 + M1*(3*d1*d1+smax*d2)/smin**5)/dt**3
    return float(bound)


def converted_jerk_bound(polys,durs):
    j=60*np.diff(polys,n=3,axis=1)/durs[:,None,None]**3
    return float(np.linalg.norm(j,axis=2).max())


def converted_joins(polys,durs):
    if len(polys)==1:return {'position_m':0.,'velocity_m_s':0.,'accel_m_s2':0.}
    dp=np.linalg.norm(polys[:-1,5]-polys[1:,0],axis=1)
    v1=5*(polys[:-1,5]-polys[:-1,4])/durs[:-1,None]
    v2=5*(polys[1:,1]-polys[1:,0])/durs[1:,None]
    a1=20*(polys[:-1,5]-2*polys[:-1,4]+polys[:-1,3])/durs[:-1,None]**2
    a2=20*(polys[1:,2]-2*polys[1:,1]+polys[1:,0])/durs[1:,None]**2
    return {'position_m':float(dp.max()),'velocity_m_s':float(np.linalg.norm(v1-v2,axis=1).max()),
            'accel_m_s2':float(np.linalg.norm(a1-a2,axis=1).max())}


def fit_one(c,n,dt,events,targets,subdivisions):
    # Split on original Bézier boundaries; events are at integer knots.
    qs=np.linspace(0,n,n*subdivisions+1)
    times=_time_at_q(qs,dt,events,targets)
    ps,vs,accs=source_values(c,n,dt,events,targets,qs)
    durs=np.diff(times)
    if np.min(durs)<=1e-9:raise ValueError('Nonpositive conversion duration')
    polys=np.stack([hermite_quintic(ps[k],vs[k],accs[k],ps[k+1],vs[k+1],accs[k+1],durs[k]) for k in range(len(durs))])
    return polys,durs


def error_bounds(c,n,dt,events,targets,polys,durs,step):
    T=float(durs.sum())
    dense=np.linspace(0,T,max(2,int(np.ceil(T/step))+1))
    knots=np.r_[0.,np.cumsum(durs)]
    grid=np.unique(np.r_[dense,knots])
    h=float(np.diff(grid).max())
    qs=inverted_q(grid,dt,events,targets,n)
    orig=source_values(c,n,dt,events,targets,qs)
    converted=converted_values(polys,durs,grid)
    errs=[float(np.linalg.norm(a-b,axis=1).max()) for a,b in zip(orig,converted)]
    pv,pa=polynomial_bounds(polys,durs)
    pj=converted_jerk_bound(polys,durs)
    # Bound temporal velocity and acceleration of source from a source certificate
    # of EXACT D9.9, fed separately via source speed/acc cap.
    # v_err is Lipschitz with <= a_source + a_converted.
    # a_err is Lipschitz with <= jerk_source + jerk_converted.
    oj=source_jerk_bound(c,n,dt,events,targets)
    return dict(sampled_position_error_m=errs[0],sampled_velocity_error_m_s=errs[1],
        sampled_accel_error_m_s2=errs[2],source_jerk_upper_m_s3=oj,
        converted_jerk_upper_m_s3=pj,converted_speed_upper_m_s=pv,
        converted_accel_upper_m_s2=pa,sample_step_max_s=h,
        # filled below once caller supplies source dynamics bound
        source_speed_upper_m_s=None,source_accel_upper_m_s2=None,
        position_error_upper_m=None,velocity_error_upper_m_s=None,
        accel_error_upper_m_s2=errs[2]+(oj+pj)*h/2)


def convert(run,stage,out,subdivisions=4,step=.0005,
            eps_pos=.03,eps_speed=.12,eps_accel=.25):
    if subdivisions<1 or subdivisions>32 or step<=0 or step>.02:raise ValueError('Invalid conversion resolution')
    cs,dt=load(run,stage)
    src=solve(cs,dt)
    if src.get('status')!='CERTIFIED_OFFLINE_COLLECTIVE':
        raise ValueError(f'D9.9 source rejected: {src.get("status")}')
    events=list(zip(src['events_q'],src['transition_width_in_original_pieces']))
    targets=src['phase_scales_after_events']
    polys=[];durations=[];reports=[]
    for i,c in enumerate(cs):
        n=active(c)
        p,h=fit_one(c,n,dt,events,targets,subdivisions)
        b=error_bounds(c,n,dt,events,targets,p,h,step)
        vs=src['dynamics'][i]['v_upper_m_s']
        ac=src['dynamics'][i]['a_upper_m_s2']
        b['source_speed_upper_m_s']=vs;b['source_accel_upper_m_s2']=ac
        b['position_error_upper_m']=b['sampled_position_error_m']+(vs+b['converted_speed_upper_m_s'])*b['sample_step_max_s']/2
        b['velocity_error_upper_m_s']=b['sampled_velocity_error_m_s']+(ac+b['converted_accel_upper_m_s2'])*b['sample_step_max_s']/2
        b['joins_C2']=converted_joins(p,h)
        b['duration_s']=float(h.sum())
        b['original_pieces']=n
        b['converted_pieces']=len(h)
        b['time_matches_original_collective']=abs(float(h.sum())-src['finishing_times_s'][str(i)])<=1e-8
        b['dynamics_ok']=b['converted_speed_upper_m_s']<=1.2+1e-9 and b['converted_accel_upper_m_s2']<=1.+1e-9
        b['errors_ok']=b['position_error_upper_m']<=eps_pos and b['velocity_error_upper_m_s']<=eps_speed and b['accel_error_upper_m_s2']<=eps_accel
        b['joins_ok']=max(b['joins_C2'].values())<1e-5
        polys.append(p);durations.append(h);reports.append(b)
    # Separation of converted paths: absolute error per robot w.r.t original source
    # times => min pair >= source minimum - error_i - error_j.
    margin=min(src['geometry']['certified_lower_m']-reports[i]['position_error_upper_m']-reports[j]['position_error_upper_m']
                for i in range(3) for j in range(i+1,3))
    passed=(all(b['time_matches_original_collective'] and b['dynamics_ok'] and b['errors_ok'] and b['joins_ok'] for b in reports)
            and margin>=1.15)
    out.mkdir(parents=True,exist_ok=True)
    npz=out/f'{stage}_d994_NON_EXECUTABLE.npz'
    np.savez_compressed(npz, **{f'R{i}_controls':polys[i] for i in range(3)},
                        **{f'R{i}_durations_s':durations[i] for i in range(3)})
    original=run/'mission'/stage/'protocol'
    protocol=original/f'{stage}_protocol_controls.npz'
    report={'stage':stage,'schema':'d99_collective_timed_quintic.preview.v1',
      'status':'CONVERTED_OFFLINE_CHECKS_PASSED' if passed else 'CONVERSION_REJECTED',
      'ros_executable':False,'obstacle_map_recertified':False,
      'source_method':'D9.9 one common q(t); phase-wide rescaling only on finishing events',
      'source_duration_s':src['optimized_makespan_s'],
      'converted_horizon_s':max(float(x.sum()) for x in durations),
      'source_finish_times_s':src['finishing_times_s'],
      'events_q':src['events_q'],'phase_scales':targets,
      'source_min_separation_lower_m':src['geometry']['certified_lower_m'],
      'converted_separation_lower_via_source_m':margin,
      'tolerances':{'position_m':eps_pos,'velocity_m_s':eps_speed,'acceleration_m_s2':eps_accel},
      'acceleration_error_continuous_certified':True,
      'checks':{'robot_conversions':all(b['dynamics_ok'] and b['errors_ok'] and b['joins_ok'] and b['time_matches_original_collective'] for b in reports),
                'separation':margin>=1.15},
      'source_sha256':{'original_protocol_controls':sha256(protocol),
                       'converted_npz':sha256(npz)},
      'robots':reports,'npz':str(npz),
      'notes':'Non executable, source certified geometry is unchanged but converted geometry deviates; clearance to obstacles not independently checked.'}
    path=out/f'{stage}_d994_conversion_report.json'
    path.write_text(json.dumps(report,indent=2)+'\n')
    return report


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--run',type=Path,required=True)
    ap.add_argument('--stage',choices=STAGES,default='pickup1')
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--subdivisions',type=int,default=4)
    ap.add_argument('--step',type=float,default=.0005)
    args=ap.parse_args()
    r=convert(args.run,args.stage,args.out,args.subdivisions,args.step)
    print(args.stage,r['status'], 'T=',r['converted_horizon_s'])
    for i,row in enumerate(r['robots']):
        print('robot',i,'segments=',row['converted_pieces'],'errors_upper(pos,v,a)=',
              row['position_error_upper_m'],row['velocity_error_upper_m_s'],row['accel_error_upper_m_s2'],
              'dynamic=',row['dynamics_ok'])
    print('Saved:',args.out)
    if r['status']=='CONVERSION_REJECTED':raise SystemExit(2)

if __name__=='__main__':main()
