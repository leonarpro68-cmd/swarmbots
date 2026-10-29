#!/usr/bin/env python3
"""D9.4 OFFLINE: approximate D9.2 with time-domain quintic Hermite Béziers.

Output is an intermediate format with INDEPENDENT piece durations by robot.
It is intentionally NOT compatible with timed_bezier.v1, and is NEVER emitted
as an executable ROS command. Rechecks the converted paths, not just D9.2.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from d9_offline_retime import STAGES, load, active
from d92_smooth_retime import values_for_robot, verify


def hermite_quintic(p0,v0,a0,p1,v1,a1,h):
    """Quintic Bernstein control points for endpoint p, dp/dt, d2p/dt2."""
    c=np.empty((6,2),dtype=float)
    c[0]=p0; c[1]=p0+h*v0/5
    c[2]=h*h*a0/20+2*c[1]-c[0]
    c[5]=p1; c[4]=p1-h*v1/5
    c[3]=h*h*a1/20+2*c[4]-c[5]
    return c


def bernstein_eval(control,u):
    x=np.asarray(u,dtype=float)
    arr=np.broadcast_to(control,(len(x),)+control.shape).copy()
    for n in range(control.shape[0]-1,0,-1):
        arr=(1-x[:,None,None])*arr[:,:n,:]+x[:,None,None]*arr[:,1:n+1,:]
    return arr[:,0,:]


def converted_values(controls,durations,t):
    t=np.asarray(t,dtype=float)
    ends=np.cumsum(durations)
    idx=np.searchsorted(ends,t,side='right')
    idx=np.minimum(idx,len(durations)-1)
    start=np.r_[0.,ends[:-1]][idx]
    h=durations[idx]
    u=np.clip((t-start)/h,0.,1.)
    pos=np.empty((len(t),2));vel=np.empty_like(pos);acc=np.empty_like(pos)
    for j in np.unique(idx):
        mask=idx==j
        c=controls[j]
        pos[mask]=bernstein_eval(c,u[mask])
        vel[mask]=bernstein_eval(5*np.diff(c,axis=0)/durations[j],u[mask])
        acc[mask]=bernstein_eval(20*np.diff(c,n=2,axis=0)/durations[j]**2,u[mask])
    done=t>=ends[-1]
    pos[done]=controls[-1,-1];vel[done]=0.;acc[done]=0.
    return pos,vel,acc


def fit_robot(cs,n,T,coef,subdivisions):
    a,b=coef
    # Invert the smooth phase at every (subdivided) geometric piece boundary.
    goals=np.linspace(0,1,n*subdivisions+1)
    lo=np.zeros_like(goals);hi=np.ones_like(goals)
    for _ in range(54):
        mid=(lo+hi)/2
        test=mid+a*np.sin(2*np.pi*mid)/(2*np.pi)+b*np.sin(4*np.pi*mid)/(4*np.pi)
        lo=np.where(test<goals,mid,lo);hi=np.where(test>=goals,mid,hi)
    t=T*(lo+hi)/2;t[0]=0.;t[-1]=T
    pp,vv,aa=values_for_robot(cs,n,T,a,b,t)
    dt=np.diff(t)
    if np.min(dt)<=1e-9:raise ValueError('nonpositive_piece_duration')
    polys=np.stack([hermite_quintic(pp[k],vv[k],aa[k],pp[k+1],vv[k+1],aa[k+1],dt[k]) for k in range(len(dt))])
    return polys,dt


def polynomial_bounds(polys,dt):
    v=5*np.diff(polys,axis=1)/dt[:,None,None]
    a=20*np.diff(polys,n=2,axis=1)/dt[:,None,None]**2
    return float(np.linalg.norm(v,axis=2).max()),float(np.linalg.norm(a,axis=2).max())


def inspect(run,stage,out,subdivisions=2,step=.01,eps_p=.03,eps_v=.12,eps_a=.25,vmax=1.2,amax=1.,sep=1.15):
    report=json.loads((run/'d92_offline_report.json').read_text())['stages'][stage]
    if report.get('status')!='CERTIFIED_CONSERVATIVE_OFFLINE':raise ValueError('D9.2 not certified')
    cs,original_dt=load(run,stage)
    ns=[active(c) for c in cs]
    if ns!=report['active_pieces']:raise ValueError('active piece count mismatch')
    Ts=np.asarray(report['duration_s'],dtype=float)
    coeff=np.asarray(report['harmonics'],dtype=float)
    ok,source_cert=verify(cs,ns,Ts,coeff,vmax,amax,sep,min(step,.04))
    if not ok:raise ValueError(f'Source failed verification: {source_cert}')
    polys=[];dts=[];bounds=[];errors=[]
    horizon=float(max(Ts))
    for i in range(3):
        c,dt=fit_robot(cs[i],ns[i],Ts[i],coeff[i],subdivisions)
        poly_v,poly_a=polynomial_bounds(c,dt)
        knots=np.r_[0.,np.cumsum(dt)]
        grid=np.unique(np.r_[np.arange(0,Ts[i],step),Ts[i],knots])
        pr,vr,ar=values_for_robot(cs[i],ns[i],Ts[i],*coeff[i],grid)
        pc,vc,ac=converted_values(c,dt,grid)
        ep=float(np.max(np.linalg.norm(pr-pc,axis=1)))
        ev=float(np.max(np.linalg.norm(vr-vc,axis=1)))
        ea=float(np.max(np.linalg.norm(ar-ac,axis=1)))
        # Conservative position error between time samples. Both speed caps
        # are independently certified; nearest-sample Lipschitz extension.
        h=float(np.max(np.diff(grid)))
        ep_upper=ep+(source_cert['robots'][i]['speed_upper_m_s']+poly_v)*h/2
        # Analogous extension for velocity error using acceleration bounds.
        ev_upper=ev+(source_cert['robots'][i]['accel_upper_m_s2']+poly_a)*h/2
        polys.append(c);dts.append(dt)
        bounds.append({'speed_upper_m_s':poly_v,'accel_upper_m_s2':poly_a})
        errors.append({'position_sampled_m':ep,'position_upper_m':ep_upper,
                       'velocity_sampled_m_s':ev,'velocity_upper_m_s':ev_upper,
                       'accel_sampled_m_s2':ea,'accel_error_certified':False,
                       'sample_step_max_s':h})
    # Joint separation of converted curves, including stationary holds.
    grid=np.linspace(0,horizon,max(2,int(np.ceil(horizon/step))+1))
    xyz=[converted_values(c,dt,grid)[0] for c,dt in zip(polys,dts)]
    pair=[];separation=float('inf');sampling_h=float(np.max(np.diff(grid)))
    for i in range(3):
        for j in range(i+1,3):
            sampled=float(np.min(np.linalg.norm(xyz[i]-xyz[j],axis=1)))
            low=sampled-(bounds[i]['speed_upper_m_s']+bounds[j]['speed_upper_m_s'])*sampling_h/2
            separation=min(separation,low)
            pair.append({'robots':[i,j],'sampled_m':sampled,'certified_lower_m':low})
    dynamic_valid=all(x['speed_upper_m_s']<=vmax+1e-9 and x['accel_upper_m_s2']<=amax+1e-9 for x in bounds)
    tolerance_valid=all(e['position_upper_m']<=eps_p and e['velocity_upper_m_s']<=eps_v and e['accel_sampled_m_s2']<=eps_a for e in errors)
    sep_valid=separation>=sep
    status='CONVERTED_OFFLINE_CHECKS_PASSED' if dynamic_valid and tolerance_valid and sep_valid else 'CONVERSION_REJECTED'
    out.mkdir(parents=True,exist_ok=True)
    path=out/f'{stage}_converted_NON_EXECUTABLE.npz'
    np.savez_compressed(path,**{f'R{i}_controls':polys[i] for i in range(3)},
                        **{f'R{i}_durations_s':dts[i] for i in range(3)},
                        source_d92_durations_s=Ts,source_d92_harmonics=coeff)
    result={'stage':stage,'status':status,'ros_executable':False,'protocol_compatible':False,
            'reason':'timed_bezier.v1 requires a common dt_s; each robot here has independently timed segments',
            'duration_s':Ts.tolist(),'subdivisions_per_original_piece':subdivisions,
            'pieces_per_robot':[len(x) for x in dts], 'converted_bounds':bounds,'errors':errors,
            'separation_pairs':pair,'certified_lower_separation_m':separation,
            'limits':{'vmax':vmax,'amax':amax,'sep':sep,'eps_position_m':eps_p,'eps_velocity_m_s':eps_v,
                      'eps_acceleration_sampled_m_s2':eps_a},
            'checks':{'dynamic':dynamic_valid,'approximation':tolerance_valid,'separation':sep_valid},
            'npz':str(path),'notes':'Acceleration mismatch is sampled, NOT a certified continuous bound; no ROS execution. Convert full protocol and recertify before use.'}
    (out/f'{stage}_conversion_report.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--stage',choices=STAGES)
    p.add_argument('--output-dir',type=Path)
    p.add_argument('--subdivisions',type=int,default=2)
    p.add_argument('--step',type=float,default=.01)
    p.add_argument('--eps-position',type=float,default=.03)
    p.add_argument('--eps-velocity',type=float,default=.12)
    p.add_argument('--eps-acceleration',type=float,default=.25)
    args=p.parse_args()
    if not 1<=args.subdivisions<=16:p.error('subdivisions must be in [1,16]')
    if not 0<args.step<=.04:p.error('step must be in (0,.04]')
    if min(args.eps_position,args.eps_velocity,args.eps_acceleration)<=0:p.error('positive tolerances required')
    output=args.output_dir or args.run/'d94_conversion'
    results={}
    for stage in [args.stage] if args.stage else STAGES:
        result=inspect(args.run,stage,output,args.subdivisions,args.step,args.eps_position,args.eps_velocity,args.eps_acceleration)
        results[stage]={'status':result['status'],'checks':result['checks'],
                        'certified_lower_separation_m':result['certified_lower_separation_m']}
        print(stage,result['status'],result['checks'])
    (output/'d94_summary.json').write_text(json.dumps({'ros_executable':False,'stages':results},indent=2)+'\n')
    print('Saved:',output)

if __name__=='__main__':main()
