#!/usr/bin/env python3
"""D.9 experimental: temporalización independiente OFFLINE, sin publicar a ROS.

Busca aceleraciones escalares (constantes) por robot sobre el movimiento activo de
una etapa D.8 y deja al robot detenido en destino tras su llegada. Verifica
cinemática de Bézier y separación continua mediante cota Lipschitz. NO es una
certificación de colisiones contra obstáculos dinámicos ni ejecuta la misión.
"""
import argparse
import json
from pathlib import Path
from math import comb
import numpy as np

STAGES = ('pickup1','delivery1','pickup2','delivery2')


def bezier(c,u):
    n=len(c)-1
    return sum(comb(n,k)*(1-u)**(n-k)*u**k*c[k] for k in range(n+1))


def load(run,stage):
    d=run/'mission'/stage/'protocol'
    report=json.loads((d/f'{stage}_protocol_report.json').read_text())
    row=report.get('stages',{}).get(stage,{})
    if not row.get('certified') or not row.get('hybrid_v2',{}).get('certified'):
        raise ValueError(f'{stage}: protocolo no certificado')
    with np.load(d/f'{stage}_protocol_controls.npz',allow_pickle=False) as z:
        cs=[np.asarray(z[f'R{i}'],dtype=float) for i in range(3)]
        dt=float(z['dt_s'])
    if not np.isfinite(dt) or dt<=0 or any(c.ndim!=3 or c.shape[1:]!=(6,2) or not np.isfinite(c).all() for c in cs):
        raise ValueError('Controles/tiempo incorrectos')
    return cs,dt


def active(c,tol=1e-6):
    # Fin de última pieza con movimiento (las piezas posteriores son esperas).
    moving=np.where(np.max(np.linalg.norm(c-c[:,0:1,:],axis=2),axis=1)>tol)[0]
    if len(moving)==0:return 0
    end=int(moving[-1]+1)
    # Evitar ignorar piezas con desplazamiento neto pero controles idénticos no aplica.
    return end


def max_bounds(c,dt,n):
    if n==0:return 0.,0.
    d=5*(c[:n,1:,:]-c[:n,:-1,:])/dt
    a=20*(c[:n,2:,:]-2*c[:n,1:-1,:]+c[:n,:-2,:])/dt**2
    return float(np.linalg.norm(d,axis=2).max()),float(np.linalg.norm(a,axis=2).max())


def position(c,dt,n,scale,t):
    if n==0:return c[0,0].copy()
    if t>=n*dt*scale:return c[n-1,-1].copy()
    q=t/(dt*scale)
    j=min(int(q),n-1)
    return bezier(c[j],min(1.,max(0.,q-j)))


def check(cs,dt,ns,scales,max_speed,max_accel,min_sep,step):
    upper=[]; totals=[]
    for c,n,s in zip(cs,ns,scales):
        v,a=max_bounds(c,dt,n)
        v/=s;a/=s*s
        if v>max_speed+1e-9 or a>max_accel+1e-9:
            return False,{'reason':'dynamic_limits','v_upper':v,'a_upper':a}
        upper.append(v)
        totals.append(n*dt*s)
    T=max(totals)
    times=np.linspace(0,T,max(2,int(np.ceil(T/step))+1))
    h=float(times[1]-times[0])
    min_d=np.inf; min_cert=np.inf
    for t in times:
        xyz=[position(c,dt,n,s,t) for c,n,s in zip(cs,ns,scales)]
        for i in range(3):
            for j in range(i+1,3):
                dist=float(np.linalg.norm(xyz[i]-xyz[j]))
                # Distancia a cualquier tiempo intermedio >= distancia en la
                # muestra más cercana menos (v_i+v_j)*h/2.
                certified=dist-(upper[i]+upper[j])*h/2
                min_d=min(min_d,dist);min_cert=min(min_cert,certified)
                if certified+1e-9<min_sep:
                    return False,{'reason':'separation','min_sampled_m':min_d,'certified_lower_m':min_cert}
    return True,{'min_sampled_m':min_d,'certified_lower_m':min_cert,'v_bounds':upper,'durations_s':totals,'makespan_s':T,'sample_interval_s':h}


def solve_stage(cs,dt,max_speed,max_accel,min_sep,step,grid_steps):
    ns=[active(c) for c in cs]
    base_dur=len(cs[0])*dt
    orig=np.ones(3)
    base_ok,base=check(cs,dt,ns,orig,max_speed,max_accel,min_sep,step)
    minimum=[]
    for c,n in zip(cs,ns):
        v,a=max_bounds(c,dt,n)
        minimum.append(max(v/max_speed,np.sqrt(a/max_accel),0.001))
    # Discretización reproducible, conservadora: no aceleramos una ruta si las
    # cotas convexas de su velocidad/aceleración no garantizan el límite.
    candidates=[np.unique(np.r_[1.,np.linspace(max(0.001,m),1.,grid_steps)]) if m<=1 else np.array([1.]) for m in minimum]
    result={'original_common_duration_s':base_dur,'active_duration_original_s':[n*dt for n in ns],
            'strict_conservative_baseline_valid':base_ok,'baseline_check':base,'minimum_scale_upper_bound':minimum}
    if not base_ok:
        result.update(status='UNVERIFIED_BASELINE',note='No optimizar ni ejecutar: el muestreo Lipschitz conservador no certifica el plan original.')
        return result
    best=(base['makespan_s'],(1.,1.,1.),base)
    import itertools
    valid=0
    for cand in itertools.product(*candidates):
        if max(ns[i]*dt*cand[i] for i in range(3))>=best[0]-1e-8:continue
        ok,info=check(cs,dt,ns,cand,max_speed,max_accel,min_sep,step)
        if ok:
            valid+=1
            if info['makespan_s']<best[0]:best=(info['makespan_s'],cand,info)
    T,scales,info=best
    result.update(status='CERTIFIED_CONSERVATIVE' if all(s>=m-1e-9 for s,m in zip(scales,minimum)) else 'REJECTED',
                  scales=list(scales),optimized_duration_s=T,saved_s=base['makespan_s']-T,
                  improvement_pct=100*(base['makespan_s']-T)/base['makespan_s'] if base['makespan_s'] else 0,
                  certificate=info,valid_improving_candidates=valid,
                  note='Certificado exclusivamente temporal para curvas existentes, límites por envolvente convexa y separación Lipschitz; no autoriza ejecución ROS.')
    return result


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run',type=Path,required=True)
    ap.add_argument('--stage',choices=STAGES)
    ap.add_argument('--max-speed',type=float,default=1.20)
    ap.add_argument('--max-accel',type=float,default=1.00)
    ap.add_argument('--min-separation',type=float,default=1.15)
    ap.add_argument('--time-step',type=float,default=0.10)
    ap.add_argument('--grid-steps',type=int,default=5)
    ap.add_argument('--output',type=Path)
    a=ap.parse_args()
    if any(v<=0 or not np.isfinite(v) for v in (a.max_speed,a.max_accel,a.min_separation,a.time_step)) or not (2<=a.grid_steps<=20):
        ap.error('Límites y time-step deben ser positivos; grid-steps entre 2 y 20')
    output={}
    for st in ([a.stage] if a.stage else STAGES):
        cs,dt=load(a.run,st)
        output[st]=solve_stage(cs,dt,a.max_speed,a.max_accel,a.min_separation,a.time_step,a.grid_steps)
        x=output[st]
        print(f"{st:9s} {x['status']:25s} original={x['original_common_duration_s']:.2f}s "
              f"nuevo={x.get('optimized_duration_s',float('nan')):.2f}s ahorro={x.get('saved_s',0):.2f}s")
    destination=a.output or a.run/'d9_offline_report.json'
    destination.parent.mkdir(parents=True,exist_ok=True)
    destination.write_text(json.dumps({'method':'D9 discrete independent time scaling; conservative OFFLINE ONLY',
            'max_speed_m_s':a.max_speed,'max_accel_m_s2':a.max_accel,'min_separation_m':a.min_separation,
            'sampling_step_s':a.time_step,'stages':output},indent=2)+'\n')
    print('Reporte:',destination)

if __name__=='__main__':main()
