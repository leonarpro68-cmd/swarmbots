#!/usr/bin/env python3
"""D9.9.8: validate lossless per-segment timing and export non-executable preview.
No ROS imports, publishers or navigation. Does NOT modify the D8 follower.
"""
import argparse, bisect, hashlib, json, math
from pathlib import Path
import numpy as np

STAGES = ('pickup1','delivery1','pickup2','delivery2')

def evaluate(piece, u, dt):
    u=float(np.clip(u,0,1))
    n=piece.shape[0]-1
    weights=np.array([math.comb(n,j)*(u**j)*((1-u)**(n-j)) for j in range(n+1)])
    pos=weights@piece
    d=np.diff(piece,axis=0)*n/dt
    weights_d=np.array([math.comb(n-1,j)*(u**j)*((1-u)**(n-1-j)) for j in range(n)])
    return pos, weights_d@d

def ref(controls, durations, t):
    starts=np.concatenate(([0.],np.cumsum(durations)))
    if t<=0: return controls[0,0],np.zeros(2)
    if t>=starts[-1]: return controls[-1,-1],np.zeros(2)
    i=min(len(durations)-1,bisect.bisect_right(starts,float(t))-1)
    return evaluate(controls[i],(t-starts[i])/durations[i],durations[i])

def validate_robot(c,d,vmax,amax):
    if c.ndim!=3 or c.shape[1:]!=(6,2) or len(d)!=len(c): raise ValueError('Dimensiones NPZ invalidas')
    if not np.isfinite(c).all() or not np.isfinite(d).all() or np.any(d<=0.02): raise ValueError('Datos no finitos o duraciones invalidas')
    dx=np.diff(c,axis=1)*5/d[:,None,None]
    dd=np.diff(dx,axis=1)*4/d[:,None,None]
    maxv=float(np.linalg.norm(dx,axis=2).max())
    maxa=float(np.linalg.norm(dd,axis=2).max())
    pos_err=np.linalg.norm(c[:-1,-1,:]-c[1:,0,:],axis=1)
    vel_err=np.linalg.norm(dx[:-1,-1,:]-dx[1:,0,:],axis=1)
    acc_err=np.linalg.norm(dd[:-1,-1,:]-dd[1:,0,:],axis=1)
    return dict(segments=len(c),duration_s=float(d.sum()),first_xy=c[0,0].tolist(),last_xy=c[-1,-1].tolist(),max_speed_bound_m_s=maxv,max_accel_bound_m_s2=maxa,
      max_C0_error_m=float(pos_err.max(initial=0)),max_C1_error_m_s=float(vel_err.max(initial=0)),max_C2_error_m_s2=float(acc_err.max(initial=0)),
      speed_within_limit=maxv<=vmax+1e-6,accel_within_limit=maxa<=amax+1e-6)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trajectory',required=True)
    p.add_argument('--output',required=True)
    p.add_argument('--speed-limit',type=float,default=1.2)
    p.add_argument('--accel-limit',type=float,default=1.0)
    p.add_argument('--sample-dt',type=float,default=.02)
    a=p.parse_args()
    if a.sample_dt<=0 or a.speed_limit<=0 or a.accel_limit<=0: p.error('limites deben ser positivos')
    path=Path(a.trajectory)
    if not path.is_file(): p.error('NPZ no existe')
    z=np.load(path,allow_pickle=False)
    robots={}; times={}
    for i in range(3):
        c=np.asarray(z[f'R{i}_controls'],dtype=float)
        d=np.asarray(z[f'R{i}_durations_s'],dtype=float)
        robots[f'summit{i}']=validate_robot(c,d,a.speed_limit,a.accel_limit)
        samples=np.linspace(0,float(d.sum()),max(2,int(math.ceil(d.sum()/a.sample_dt))+1))
        xy=[]; vv=[]
        for t in samples:
            pos,vel=ref(c,d,t);xy.append(pos.tolist());vv.append(vel.tolist())
        times[f'summit{i}']={'time_s':samples.tolist(),'position_m':xy,'velocity_m_s':vv}
    issues=[]
    for name,r in robots.items():
        if not r['speed_within_limit']: issues.append(f'{name}: speed bound exceeds limit')
        if not r['accel_within_limit']: issues.append(f'{name}: accel bound exceeds limit')
        if r['max_C0_error_m']>1e-4: issues.append(f'{name}: position discontinuity')
        if r['max_C1_error_m_s']>1e-3: issues.append(f'{name}: velocity discontinuity')
        if r['max_C2_error_m_s2']>1e-2: issues.append(f'{name}: acceleration discontinuity')
    output={'schema':'swarm.d998.preview.readonly.v1','status':'PASS_TIMING_V2_OFFLINE' if not issues else 'REJECT_TIMING_V2_OFFLINE',
      'execution_authorized':False,'commands_sent':0,'source':str(path.resolve()),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
      'robots':robots,'issues':issues,'notes':['Solo certifica cotas de derivadas por convexidad y continuidad de segmentos; NO colisiones, TF o navegacion.','La interfaz ROS timed_bezier.v1 NO acepta duraciones variables.'],'sampled_reference':times}
    dst=Path(a.output);dst.parent.mkdir(parents=True,exist_ok=True);dst.write_text(json.dumps(output,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in output.items() if k!='sampled_reference'},indent=2))
    return 0 if not issues else 2
if __name__=='__main__': raise SystemExit(main())
