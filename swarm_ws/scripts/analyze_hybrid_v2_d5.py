#!/usr/bin/env python3
"""D.5: análisis offline de odometría y referencia registrados por observer.
Requiere matplotlib, numpy; no ROS ni Gazebo.
"""
import argparse,csv,json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROBOTS=('summit0','summit1','summit2')

def analyze(rows,events,minimum_separation):
    active={r:sorted((x for x in rows if x['robot']==r and x['phase']=='active'),key=lambda x:float(x['t_sim_s'])) for r in ROBOTS}
    summary={'robots':{},'minimum_pair_separation_m':{},'events':events,'complete':False}
    for r,rr in active.items():
        if not rr:continue
        e=np.array([float(x['error_m']) for x in rr]);v=np.array([float(x['odom_v_m_s']) for x in rr]);t=np.array([float(x['t_sim_s']) for x in rr])
        summary['robots'][r]={'samples':len(rr),'rmse_position_m':float(np.sqrt(np.mean(e*e))),
            'mean_error_m':float(np.mean(e)),'max_error_m':float(np.max(e)),
            'mean_odom_speed_m_s':float(np.mean(v)),'max_odom_speed_m_s':float(np.max(v)),
            'observed_active_duration_s':float(t[-1]-t[0]),
            'frame_ids':sorted(set(x['frame'] for x in rr))}
    # Nearest-time alignment: odom samples may not have identical timestamps.
    # Do not extrapolate: only pairs within 0.08 s are comparable.
    for i,a in enumerate(ROBOTS):
        for b in ROBOTS[i+1:]:
            ra,rb=active[a],active[b]; j=0; distances=[]
            for x in ra:
                t=float(x['t_sim_s'])
                while j+1<len(rb) and abs(float(rb[j+1]['t_sim_s'])-t)<abs(float(rb[j]['t_sim_s'])-t):j+=1
                if rb and abs(float(rb[j]['t_sim_s'])-t)<=.08:
                    y=rb[j];distances.append(math.dist((float(x['x_m']),float(x['y_m'])),(float(y['x_m']),float(y['y_m']))))
            summary['minimum_pair_separation_m'][a+'-'+b]=min(distances) if distances else None
    states={e.get('robot'):e['status'] for e in events if e.get('kind')=='status'}
    summary['last_status']=states
    summary['complete']=all(states.get(r)=='done' for r in ROBOTS)
    summary['minimum_separation_threshold_m']=minimum_separation
    summary['sampled_separation_pass']=all(v is not None and v>=minimum_separation for v in summary['minimum_pair_separation_m'].values())
    summary['note']='Métricas de odometría muestreada; no certificación física continua ni frente a obstáculos.'
    return summary,active

def figure(active,output):
    fig,axs=plt.subplots(1,2,figsize=(13,5.8));colors=['tab:blue','tab:orange','tab:green']
    for i,r in enumerate(ROBOTS):
        rr=active[r]
        if not rr:continue
        xy=np.array([[float(x['x_m']),float(x['y_m'])] for x in rr]);ref=np.array([[float(x['ref_x_m']),float(x['ref_y_m'])] for x in rr]);t=np.array([float(x['t_sim_s']) for x in rr]);err=np.array([float(x['error_m']) for x in rr]);
        axs[0].plot(ref[:,0],ref[:,1],linestyle='--',color=colors[i],linewidth=2,label=f'{r} referencia')
        axs[0].plot(xy[:,0],xy[:,1],color=colors[i],linewidth=1.6,label=f'{r} odometría')
        axs[1].plot(t-t[0],err,color=colors[i],label=r)
    axs[0].set_title('Trayectorias: Bézier planificada vs odometría');axs[0].set_xlabel('x (m)');axs[0].set_ylabel('y (m)');axs[0].axis('equal')
    axs[1].set_title('Error de seguimiento');axs[1].set_xlabel('Tiempo activo desde primera muestra (s)');axs[1].set_ylabel('Error XY (m)')
    for ax in axs:ax.grid(alpha=.3);ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(output,dpi=180);plt.close(fig)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--input-dir',required=True);p.add_argument('--min-separation',type=float,default=1.15);a=p.parse_args()
    root=Path(a.input_dir);source=root/'odom_tracking.csv';evfile=root/'events.jsonl'
    with source.open(newline='',encoding='utf-8') as f:rows=list(csv.DictReader(f))
    events=[json.loads(line) for line in evfile.read_text(encoding='utf-8').splitlines() if line.strip()] if evfile.exists() else []
    result,active=analyze(rows,events,a.min_separation)
    output=root/'metrics.json';output.write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
    figure(active,root/'tracking_comparison.png')
    print('DATOS:',len(rows),'COMPLETED:',result['complete'],'SEPARATION_SAMPLED_PASS:',result['sampled_separation_pass'])
    print('PNG:',root/'tracking_comparison.png','JSON:',output)
if __name__=='__main__':main()
