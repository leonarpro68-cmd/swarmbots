#!/usr/bin/env python3
"""Láminas de rapidez/aceleración 4x2 para D5.1 y D9.9, estilo D8."""
import argparse
import csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from d99_plot_common import stages_from_args, samples_for_stage, original_profiles

COLORS=['#1679ba','#188d34','#e31a1c']
NAMES={'pickup1':'Recogida 1','delivery1':'Entrega 1',
       'pickup2':'Recogida 2','delivery2':'Entrega 2'}

def plot_sheet(records, version, output, run_name):
    fig,axs=plt.subplots(4,2,figsize=(15.3,17.2),squeeze=False)
    for row, (stage, report, z, dt) in enumerate(records):
        time=z['q_common']*dt if version=='D5.1' else z['time_s']
        for col in range(2):
            ax=axs[row,col]
            for i,color in enumerate(COLORS):
                if version=='D5.1':
                    v,a=original_profiles(z,dt,i)
                else:
                    v=z[f'R{i}_velocity_xy']; a=z[f'R{i}_acceleration_xy']
                value=np.linalg.norm(v if col==0 else a,axis=1)
                ax.plot(time,value,color=color,lw=1.7)
            limit=1.2 if col==0 else 1.0
            ax.axhline(limit,color='#555',ls='--',lw=1.1)
            ax.set_ylim(0, max(limit*1.05,ax.get_ylim()[1]*1.05))
            ax.set_xlim(0, float(time[-1])*1.015)
            ax.grid(alpha=.23)
            title=f'{NAMES[stage]} · {"Rapidez (m/s)" if col==0 else "Aceleración |a| (m/s²)"}'
            ax.set_title(title,fontsize=10.5,weight='bold')
            ax.set_ylabel('Rapidez (m/s)' if col==0 else 'Aceleración |a| (m/s²)',fontsize=9)
            ax.set_xlabel('Tiempo de trayectoria (s)',fontsize=9)
            if row==0:
                labels=[Line2D([0],[0],color=c,lw=1.8,label=f'Summit {i}') for i,c in enumerate(COLORS)]
                labels.append(Line2D([0],[0],color='#555',ls='--',lw=1.1,label=f'Límite {limit:g}'))
                ax.legend(handles=labels,loc='upper right',fontsize=8,ncol=2)
            ax.tick_params(labelsize=8)
    for row in range(len(records),4):
        axs[row,0].axis('off');axs[row,1].axis('off')
    fig.suptitle(f'SwarmBots · velocidad y aceleración Bézier · {version} · {run_name}',
                 fontsize=14,fontweight='bold',y=.992)
    fig.text(.5,.008,'Perfiles de planificación offline; no son velocidades ni aceleraciones medidas en Gazebo.',
             ha='center',fontsize=8,color='#444')
    fig.tight_layout(rect=[0,.019,1,.968],h_pad=2.0,w_pad=1.2)
    output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(output,dpi=170,facecolor='white');plt.close(fig)
    print('CINEMÁTICA:',output)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    g=p.add_mutually_exclusive_group(required=True)
    g.add_argument('--run-dir',type=Path); g.add_argument('--manifest',type=Path)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--stages',nargs='+',default=None)
    p.add_argument('--step-q',type=float,default=.015)
    a=p.parse_args()
    if a.step_q<=0:p.error('--step-q debe ser positivo')
    run,stages,reports,_=stages_from_args(a.run_dir,a.manifest,a.stages)
    a.output_dir.mkdir(parents=True,exist_ok=True)
    records=[]; summary=[]
    for st in stages:
        rep,z,dt=samples_for_stage(run,st,reports[st],a.step_q)
        records.append((st,rep,z,dt))
        summary.append((st,rep['original_makespan_s'],rep['optimized_makespan_s'],rep['saving_s'],rep['saving_pct']))
    plot_sheet(records,'D5.1',a.output_dir/'d51_velocidad_aceleracion.png',run.name)
    plot_sheet(records,'D9.9',a.output_dir/'d99_velocidad_aceleracion.png',run.name)
    with (a.output_dir/'resumen_d99_graficos.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f);w.writerow(['etapa','D51_s','D99_s','ahorro_s','ahorro_pct']);w.writerows(summary)
    print('RESUMEN:',a.output_dir/'resumen_d99_graficos.csv')

if __name__=='__main__':main()
