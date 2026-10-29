#!/usr/bin/env python3
"""Comparación de rapidez planificada (Bézier) y odometría medida en 4 etapas.

Las aceleraciones medidas son derivadas del vector velocidad de odometría
transformado al mundo con el yaw; son estimaciones sensibles al ruido, NO IMU ni fuerza.
"""
import argparse
import csv
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from plot_mission_d8 import STAGES,TITLES,load_stage,sample_kinematics

COLORS=('tab:blue','tab:green','tab:red')

def read_measurements(path):
    if not path.is_file():raise FileNotFoundError(f'No existe {path}; activar registro durante la misión')
    with path.open(newline='',encoding='utf-8') as f: rows=list(csv.DictReader(f))
    if len(rows)<3:raise ValueError(f'Muestras insuficientes: {path}')
    t=np.array([float(x['stamp_s']) for x in rows]);v=np.array([float(x['speed_xy_m_s']) for x in rows]);
    vx=np.array([float(x['vx_world_m_s']) for x in rows]);vy=np.array([float(x['vy_world_m_s']) for x in rows]);
    if not np.isfinite(t).all() or not np.isfinite(v).all():raise ValueError(f'NaN/Inf: {path}')
    valid=np.concatenate(([True],np.diff(t)>1e-6))
    t,v,vx,vy=t[valid],v[valid],vx[valid],vy[valid]
    if len(t)<3:raise ValueError(f'Tiempos repetidos o no crecientes: {path}')
    # El registro comienza antes de que se emita la trayectoria: etapa planificada
    # empieza tras una latencia de armado de ~8 s. NO alineamos t=0 por primer CSV.
    return t,v,vx,vy

def smooth_derivative(t,v,window=7):
    # Media móvil para mitigación de ruido de odometría; diferencias en sim time.
    w=min(window,len(v));w=w if w%2 else max(1,w-1)
    vp=np.pad(v,(w//2,w//2),mode='edge')
    vf=np.convolve(vp,np.ones(w)/w,mode='valid')
    return np.gradient(vf,t)

def render(run,output):
    fig,axes=plt.subplots(4,2,figsize=(18,17))
    for k,(stage,title) in enumerate(zip(STAGES,TITLES)):
        _,ctrls,_=load_stage(run,stage)
        with np.load(run/'mission'/stage/'protocol'/f'{stage}_protocol_controls.npz',allow_pickle=False) as z:dt=float(z['dt_s'])
        for i in range(3):
            robot=f'summit{i}'
            tp,vplan,aplan=sample_kinematics(ctrls[i],dt)
            tm,vm,vx,vy=read_measurements(run/'mission'/stage/'odom_record'/f'{robot}_odom.csv')
            # Al registrar desde ANTES de enviar la trayectoria, el tiempo medido
            # incluye el armado del seguidor. Al usar comienzo de movimiento como
            # ancla, esta gráfica visual es aproximada, no una sincronización exacta.
            moving=np.flatnonzero(vm>0.025)
            start=tm[moving[0]] if len(moving) else tm[0]
            tm=tm-start
            am=np.hypot(smooth_derivative(tm,vx),smooth_derivative(tm,vy))
            axv,axa=axes[k]
            axv.plot(tp,vplan,color=COLORS[i],linestyle='--',alpha=.6,label=f'{robot} plan')
            axv.plot(tm,vm,color=COLORS[i],linewidth=1.2,label=f'{robot} odom')
            axa.plot(tp,aplan,color=COLORS[i],linestyle='--',alpha=.6,label=f'{robot} plan')
            axa.plot(tm,am,color=COLORS[i],linewidth=1.1,label=f'{robot} |d(v⃗)/dt|')
            print(stage,robot,f'vmax_plan={max(vplan):.3f}',f'vmax_odom={max(vm):.3f}',
                  f'|dv/dt|_max_approx={max(am):.3f}',flush=True)
        for a,label,limit in ((axes[k,0],'Rapidez (m/s)',1.20),(axes[k,1],'Aceleración (m/s²)',1.0)):
            a.axhline(limit,color='black',linestyle=':',alpha=.65)
            a.set_title(title+' · '+label);a.set_xlabel('Tiempo relativo aproximado (s)')
            a.set_ylabel(label);a.grid(alpha=.25);a.set_xlim(left=0)
        if k==0:
            axes[k,0].legend(fontsize=8,ncol=2)
            axes[k,1].legend(fontsize=8,ncol=2)
    fig.suptitle(f'D.8 · Bézier planificado vs odometría · {run.name}',fontsize=15,fontweight='bold')
    fig.text(.03,.015,'Odometría registrada durante navegación Hybrid V2. Tiempo anclado al primer movimiento (aproximado). '+
             'Aceleración estimada = |d(v_world suavizada)/dt|, desde odometría; no es una medición de IMU.',fontsize=9)
    fig.tight_layout(rect=(0,.025,1,.965))
    output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(output,dpi=150,bbox_inches='tight');plt.close(fig)
    print('PNG:',output)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--output',type=Path)
    a=p.parse_args();run=a.run.resolve()
    render(run,a.output or run/'mission_hybrid_v2_comparacion_odom.png')
if __name__=='__main__':main()
