#!/usr/bin/env python3
"""D9.9 — Reescalado COLECTIVO SUCESIVO, solo offline (sin ROS).

Usa un UNICO reloj de progreso q(t) para todos los Summit. Inicialmente
s(q)=1 (D5.1). Solo DESPUES de que un robot alcanza q=n_i, puede reducirse
suavemente dt/dq = dt_base*s(q), sin saltos de velocidad/aceleracion.

El conjunto de robots activos es {i: q < n_i}; los terminados permanecen
estacionarios en sus metas. Conserva p_i(q) geometrica y la sincronizacion
espacial por parametro q de la solucion Hybrid V2 inicial.

PREVIEW OFFLINE: no genera mensajes/control ROS ni certificados de seguimiento.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from d9_offline_retime import STAGES, active, load


def smoothstep(z):
    z = np.clip(np.asarray(z, dtype=float), 0., 1.)
    return z*z*(3.-2.*z)


def _s_and_d(q, events, targets):
    q=np.asarray(q,dtype=float)
    s=np.ones_like(q)
    deriv=np.zeros_like(q)
    before=1.
    for (e,w), after in zip(events,targets):
        dz=float(after)-before
        z=np.clip((q-e)/w,0.,1.)
        s += dz*smoothstep(z)
        inside=(q>e)&(q<e+w)
        deriv += np.where(inside,dz*(6*z-6*z*z)/w,0.)
        before=float(after)
    return s,deriv


def _time_at_q(q,dt,events,targets):
    """Integracion ANALITICA de dt/dq (sin error acumulado de integracion)."""
    q=np.asarray(q,dtype=float)
    t=dt*q.copy()
    before=1.
    for (e,w),after in zip(events,targets):
        dz=float(after)-before
        L=np.maximum(q-e,0.)
        z=np.minimum(L/w,1.)
        integ=w*(z**3-0.5*z**4)+np.maximum(L-w,0.)
        t += dt*dz*integ
        before=float(after)
    return t


def _bezier_sample(c,n,q):
    """Vectorized p, dp/dq and d2p/dq2. Robot holds after its active n."""
    q=np.asarray(q,dtype=float).reshape(-1)
    idx=np.minimum(np.floor(np.clip(q,0,n-1e-12)).astype(int),n-1)
    z=np.clip(q-idx,0.,1.)
    # q= n sits at the LAST end point before the stationary mask.
    z=np.where(q>=n,1.,z)
    cp=c[idx]
    def ev(x):
        z0=z[:,None,None]
        for k in range(x.shape[1]-1,0,-1):
            x=(1.-z0)*x[:,:k]+z0*x[:,1:k+1]
        return x[:,0]
    p=ev(cp.copy())
    d=ev(5*np.diff(cp,axis=1))
    a=ev(20*np.diff(cp,n=2,axis=1))
    hold=q>=n
    p[hold]=c[n-1,-1]
    d[hold]=0.
    a[hold]=0.
    return p,d,a


def _check_joins(cs,ns,tol=2e-6):
    errors=[]
    for i,(c,n) in enumerate(zip(cs,ns)):
        if n==0:
            raise ValueError('Trayectoria estacionaria no soportada en el prototipo')
        p=np.linalg.norm(c[:n-1,-1]-c[1:n,0],axis=1)
        v=np.linalg.norm(5*(c[:n-1,-1]-c[:n-1,-2])-5*(c[1:n,1]-c[1:n,0]),axis=1)
        a=np.linalg.norm(20*(c[:n-1,-1]-2*c[:n-1,-2]+c[:n-1,-3])-20*(c[1:n,2]-2*c[1:n,1]+c[1:n,0]),axis=1)
        endpoint=np.array([np.linalg.norm(5*(c[0,1]-c[0,0])),
                           np.linalg.norm(20*(c[0,2]-2*c[0,1]+c[0,0])),
                           np.linalg.norm(5*(c[n-1,-1]-c[n-1,-2])),
                           np.linalg.norm(20*(c[n-1,-1]-2*c[n-1,-2]+c[n-1,-3]))])
        error=dict(robot=i,position_max=float(max(p,default=0)),
                   velocity_max=float(max(v,default=0)),
                   acceleration_max=float(max(a,default=0)),
                   endpoints_max=float(endpoint.max()))
        errors.append(error)
        if max(error['position_max'],error['velocity_max'],
               error['acceleration_max'],error['endpoints_max'])>tol:
            raise ValueError(f'Geometria no C2 o inicio/fin no detenido: {error}')
    return errors


def _event_setup(ns,ramp_pieces):
    # At most 2 non-final finishing events for three robots.
    changes=sorted(set(ns))[:-1]
    events=[]
    for k,e in enumerate(changes):
        next_e=changes[k+1] if k+1<len(changes) else max(ns)
        if next_e<=e:raise AssertionError('Eventos no ordenados')
        w=min(float(ramp_pieces),float(next_e-e))
        events.append((float(e),w))
    return events


def _interval_max_ds(lo,hi,events,targets):
    test=[lo,hi]
    for e,w in events:
        center=e+w/2.
        if lo<center<hi:test.append(center)
    _,ds=_s_and_d(np.asarray(test),events,targets)
    return float(np.max(np.abs(ds)))


def dynamic_certificate(cs,ns,dt,events,targets,vmax,amax,subdivisions=8):
    """Guaranteed hull upper bound for speed/acceleration under smooth q(t).

    For each piece, M1 and M2 bound |dp/dq| and |d²p/dq²| via
    the Bernstein convex hull. On every subinterval, smin and sup|ds/dq|
    imply the upper bounds for chain-rule time derivatives.
    """
    per=[]
    for i,(c,n) in enumerate(zip(cs,ns)):
        vel_cp=5*np.diff(c[:n],axis=1)
        acc_cp=20*np.diff(c[:n],n=2,axis=1)
        m1=np.linalg.norm(vel_cp,axis=2).max(axis=1)
        m2=np.linalg.norm(acc_cp,axis=2).max(axis=1)
        max_v=0.;max_a=0.;critical=None
        for k in range(n):
            for sub in range(subdivisions):
                lo=k+sub/subdivisions
                hi=k+(sub+1)/subdivisions
                sv,_=_s_and_d(np.array([lo,hi]),events,targets)
                smin=float(min(sv))
                if smin<=0:return False,{'reason':'nonpositive_time_scale'}
                max_ds=_interval_max_ds(lo,hi,events,targets)
                v=m1[k]/(dt*smin)
                a=m2[k]/(dt*dt*smin*smin)+m1[k]*max_ds/(dt*dt*smin**3)
                if v>max_v:max_v=float(v)
                if a>max_a:max_a=float(a);critical=(k,sub)
                if v>vmax+1e-9 or a>amax+1e-9:
                    return False,{'reason':'dynamic_hull_limit','robot':i,
                         'segment':k,'subinterval':sub,'v_upper_m_s':float(v),
                         'a_upper_m_s2':float(a),'limits':[vmax,amax]}
        per.append(dict(robot=i,v_upper_m_s=max_v,a_upper_m_s2=max_a,
                        critical_accel_cell=critical))
    return True,per


def separation_certificate(cs,ns,sep,q_step):
    """One-time geometric joint-path certificate, invariant under common q(t)."""
    end=max(ns)
    q=np.linspace(0,end,max(2,int(np.ceil(end/q_step))+1))
    h=float(q[1]-q[0])
    pts=[_bezier_sample(c,n,q)[0] for c,n in zip(cs,ns)]
    # Global bound wrt common q includes active piece derivative hull.
    m=[float(np.linalg.norm(5*np.diff(c[:n],axis=1),axis=2).max()) for c,n in zip(cs,ns)]
    pairs=[]
    for i in range(3):
        for j in range(i+1,3):
            dist=np.linalg.norm(pts[i]-pts[j],axis=1)
            closest=float(np.min(dist))
            lower=closest-(m[i]+m[j])*h/2
            pairs.append(dict(robots=[i,j],sampled_min_m=closest,
                              certified_lower_m=float(lower)))
            if lower<sep-1e-9:
                return False,dict(reason='separation',pairs=pairs,q_step=h)
    return True,dict(pairs=pairs,certified_lower_m=min(r['certified_lower_m'] for r in pairs),
                     q_step=h,method='Common q preserves joint geometric configurations')


def solve(cs,dt,vmax=1.2,amax=1.,sep=1.15,ramp_pieces=1.0,
          target_step=.02,q_step=.01,subdivisions=8):
    if dt<=0 or ramp_pieces<=0 or target_step<=0 or q_step<=0 or subdivisions<1:
        raise ValueError('Parametros numericos invalidos')
    ns=[active(c) for c in cs]
    joins=_check_joins(cs,ns)
    sep_ok,geom=separation_certificate(cs,ns,sep,q_step)
    events=_event_setup(ns,ramp_pieces)
    baseline_ok,baseline=dynamic_certificate(cs,ns,dt,events,[1.]*len(events),vmax,amax,subdivisions)
    report={'method':'D9.9 collective successive phase retiming (single common q)',
            'execution_authorized':False,'active_pieces':ns,'original_dt_s':dt,
            'original_makespan_s':max(ns)*dt,'joins_C2':joins,
            'events_q':[e for e,w in events],
            'transition_width_in_original_pieces':[w for e,w in events],
            'baseline_dynamics_ok':baseline_ok,'baseline_dynamics':baseline,
            'baseline_geometry_ok':sep_ok,'geometry':geom,
            'limits':dict(max_speed_m_s=vmax,max_accel_m_s2=amax,min_separation_m=sep),
            'notes':'OFFLINE ONLY; original certified obstacle geometry unchanged; no motor/ROS tracking certificate.'}
    if not baseline_ok or not sep_ok:
        return dict(report,status='BASELINE_REJECTED')
    targets=[]; searches=[]
    for idx,(e,w) in enumerate(events):
        lo=1. if idx==0 else targets[-1]
        # Impose non-increasing common scale. Search descending and take
        # the shortest verified choice, without changing previous phases.
        trial_values=np.round(np.arange(lo,0.24,-target_step),10)
        winning=lo; trials=0; reject=None
        for candidate in trial_values:
            if candidate<=0:continue
            proposed=targets+[float(candidate)]
            ok,cert=dynamic_certificate(cs,ns,dt,events[:idx+1],proposed,
                                        vmax,amax,subdivisions)
            trials+=1
            if ok:winning=float(candidate)
            else:reject=cert
        targets.append(winning)
        searches.append(dict(q_event=e,active_after=[i for i,n in enumerate(ns) if n>e],
                             target_scale=winning,candidates_checked=trials,
                             last_rejection=reject))
    ok,cert=dynamic_certificate(cs,ns,dt,events,targets,vmax,amax,subdivisions)
    if not ok:return dict(report,status='FINAL_DYNAMICS_REJECTED',error=cert)
    end=max(ns)
    finish_times={str(i):float(_time_at_q(np.array([n]),dt,events,targets)[0])
                  for i,n in enumerate(ns)}
    original_finish={str(i):float(n*dt) for i,n in enumerate(ns)}
    T=float(_time_at_q(np.array([end]),dt,events,targets)[0])
    phases=[dict(q_start=float(a),q_end=float(b),
                 active_robots=[i for i,n in enumerate(ns) if n>a],
                 start_s=float(_s_and_d(np.array([a]),events,targets)[0][0]),
                 end_s=float(_s_and_d(np.array([b]),events,targets)[0][0]),
                 t_start_s=float(_time_at_q(np.array([a]),dt,events,targets)[0]),
                 t_end_s=float(_time_at_q(np.array([b]),dt,events,targets)[0]))
            for a,b in zip([0.]+[e for e,w in events],[e for e,w in events]+[float(end)])]
    report.update(status='CERTIFIED_OFFLINE_COLLECTIVE',phase_scales_after_events=targets,
                  searches=searches,dynamics=cert,finishing_times_s=finish_times,
                  original_finishing_times_s=original_finish,optimized_makespan_s=T,
                  saving_s=report['original_makespan_s']-T,
                  saving_pct=100*(report['original_makespan_s']-T)/report['original_makespan_s'],
                  phases=phases)
    return report


def sample_solution(cs,dt,report,q_step=.01):
    if report['status']!='CERTIFIED_OFFLINE_COLLECTIVE':
        raise ValueError('Solo se pueden muestrear resultados certificados')
    ns=report['active_pieces']
    events=list(zip(report['events_q'],report['transition_width_in_original_pieces']))
    targets=report['phase_scales_after_events']
    q=np.linspace(0,max(ns),int(np.ceil(max(ns)/q_step))+1)
    t=_time_at_q(q,dt,events,targets)
    s,sd=_s_and_d(q,events,targets)
    data={'time_s':t,'q_common':q,'scale_common':s,'scale_derivative_q':sd}
    for i,(c,n) in enumerate(zip(cs,ns)):
        p,d1,d2=_bezier_sample(c,n,q)
        v=d1/(dt*s[:,None])
        a=d2/(dt*dt*s[:,None]**2)-d1*sd[:,None]/(dt*dt*s[:,None]**3)
        data[f'R{i}_position_xy']=p
        data[f'R{i}_velocity_xy']=v
        data[f'R{i}_acceleration_xy']=a
        data[f'R{i}_active']=(q<n).astype(np.uint8)
    return data


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',required=True,type=Path,help='Carpeta D8 con mission/pickup1/protocol')
    p.add_argument('--stage',choices=STAGES,default='pickup1')
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--max-speed',type=float,default=1.2)
    p.add_argument('--max-accel',type=float,default=1.)
    p.add_argument('--min-separation',type=float,default=1.15)
    p.add_argument('--ramp-pieces',type=float,default=1.)
    p.add_argument('--step-scale',type=float,default=.02)
    p.add_argument('--step-q',type=float,default=.01)
    a=p.parse_args()
    cs,dt=load(a.run,a.stage)
    r=solve(cs,dt,a.max_speed,a.max_accel,a.min_separation,
            a.ramp_pieces,a.step_scale,a.step_q)
    a.out.mkdir(parents=True,exist_ok=True)
    (a.out/f'{a.stage}_d99_report.json').write_text(json.dumps(r,indent=2)+'\n')
    if r['status']=='CERTIFIED_OFFLINE_COLLECTIVE':
        samples=sample_solution(cs,dt,r,a.step_q)
        np.savez_compressed(a.out/f'{a.stage}_d99_OFFLINE_NON_EXECUTABLE.npz',**samples)
    print(a.stage,r['status'],'original_s=',r['original_makespan_s'],
          'new_s=',r.get('optimized_makespan_s'), 'saved_s=',r.get('saving_s'))
    print('Report:',a.out/f'{a.stage}_d99_report.json')

if __name__=='__main__':main()
