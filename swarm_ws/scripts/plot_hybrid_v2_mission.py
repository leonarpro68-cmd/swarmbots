#!/usr/bin/env python3
"""D.2: misión completa CBS + Hybrid V2, offline, cuatro paneles.

Reusa literalmente el estilo y el muestreo inicial de plot_mission_routes.py.
No emite controles ROS ni declara segura una curva REJECTED.
"""
from __future__ import annotations
import argparse
import json
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

import plot_mission_routes as original
from swarm_behavior.swarm_planning.bridge import PlanningBridge
from swarm_behavior.swarm_planning.scenario import load as load_scenario
from swarm_behavior.swarm_planning.hybrid_v2.cbs_adapter import plan_v2

STAGES = ('pickup1', 'delivery1', 'pickup2', 'delivery2')
NAMES = ('Recogida · Ronda 1','Entrega · Ronda 1','Recogida · Ronda 2','Entrega · Ronda 2')


def plan_four(scenario, *, max_ct=20000, max_time=65, margin=.05,
              iterations=2, max_speed=1.2, max_accel=1.0, maxiter=450):
    bridge=PlanningBridge(scenario)
    # D.8.7: el spawn de Gazebo usa EXACTAMENTE scenario.robots.
    starts=tuple(tuple(node) for node in scenario.robots)
    if len(starts) != 3 or len(set(starts)) != 3 or any(node not in bridge.graph.vertices for node in starts):
        raise ValueError('Posiciones iniciales del escenario fuera del grafo CBS')
    products={}
    pending_before_r1=set(bridge.pending)

    def solve_stage(stage, plan):
        # Importantísimo: antes de mutar el mapa dinámico.
        report, controls=plan_v2(plan,bridge.graph,stage=stage,margin=margin,
                    iterations=iterations,max_speed=max_speed,max_accel=max_accel,maxiter=maxiter)
        products[stage]={'plan':plan,'report':report,'controls':controls,
                         'pending':set(bridge.pending)}
        print(f'{stage}: {report.get("status")} '
              f'cost={report.get("final_cost", "-")} '
              f'reason={report.get("reason", "-")}',flush=True)

    p1=bridge.plan_pickup(starts,max_ct=max_ct,max_time=max_time)
    solve_stage('pickup1',p1)
    picked1=tuple(name for name,_ in p1.assigned)
    for name in picked1: bridge.confirm_collected(name)
    pending_after_pick1=set(bridge.pending)
    d1=bridge.plan_delivery(tuple(path[-1] for path in p1.paths),picked1,max_ct=max_ct,max_time=max_time)
    solve_stage('delivery1',d1)
    pending_before_r2=set(bridge.pending)
    p2=bridge.plan_pickup(tuple(path[-1] for path in d1.paths),max_ct=max_ct,max_time=max_time)
    solve_stage('pickup2',p2)
    picked2=tuple(name for name,_ in p2.assigned)
    for name in picked2: bridge.confirm_collected(name)
    pending_after_pick2=set(bridge.pending)
    d2=bridge.plan_delivery(tuple(path[-1] for path in p2.paths),picked2,max_ct=max_ct,max_time=max_time)
    solve_stage('delivery2',d2)
    if bridge.pending: raise RuntimeError('Basuras pendientes: '+repr(sorted(bridge.pending)))
    contexts=[
      dict(pending=pending_before_r1,just_picked=picked1,already_removed=()),
      dict(pending=pending_after_pick1,just_picked=(),already_removed=picked1),
      dict(pending=pending_before_r2,just_picked=picked2,already_removed=picked1),
      dict(pending=pending_after_pick2,just_picked=(),already_removed=picked1+picked2),
    ]
    return products,contexts,{'picked1':picked1,'picked2':picked2,'starts':starts}


def bezier_points(control, samples=28):
    all_points=[]
    for k, piece in enumerate(np.asarray(control)):
        u=np.linspace(0,1,samples,endpoint=(k==len(control)-1))
        from math import comb
        basis=np.array([comb(5,j)*u**j*(1-u)**(5-j) for j in range(6)]).T
        all_points.append(basis@piece)
    return np.concatenate(all_points,axis=0)


def draw(scenario,products,contexts,summary,output,show_graph=False):
    fig,axes=plt.subplots(2,2,figsize=(18,24)); axes=axes.ravel()
    for idx,(stage,name,ctx) in enumerate(zip(STAGES,NAMES,contexts)):
        ax=axes[idx]; item=products[stage]; plan=item['plan']; r=item['report']
        original.draw_background(ax,show_graph=show_graph)
        original.draw_objects(ax,scenario,**ctx)
        for i,path in enumerate(plan.paths):
            xs,ys=original.nodes_to_xy(path); color=original.ROBOT_COLORS[i]
            ax.plot(xs,ys,'--o',color=color,linewidth=1.15,alpha=.5,markersize=2.2,zorder=8)
            ax.scatter(xs[0],ys[0],color=color,marker='s',s=70,zorder=12)
            ax.scatter(xs[-1],ys[-1],color=color,marker='*',s=110,zorder=12)
            ax.text(xs[0]+.15,ys[0]+.15,f'R{i}',color=color,weight='bold',fontsize=9,zorder=13)
            if r.get('certified') and item['controls'] is not None:
                xy=bezier_points(item['controls'][i]); ax.plot(xy[:,0],xy[:,1],color=color,linewidth=2.7,zorder=10)
        status='CERTIFIED' if r.get('certified') else 'REJECTED — solo CBS'
        ax.set_title(f'{name}\nCBS cost={plan.cost} | Hybrid V2: {status}',weight='bold',fontsize=12)
        if not r.get('certified'):
            reason=str(r.get('reason','sin solución'))[:160]
            ax.text(.5,.035,reason,ha='center',va='bottom',transform=ax.transAxes,
                    fontsize=8,color='#9a3412',bbox={'facecolor':'#ffedd5','alpha':.93,'edgecolor':'none'})
    handles=[Line2D([0],[0],color='gray',lw=1.2,ls='--',marker='o',label='CBS discreto'),
             Line2D([0],[0],color='gray',lw=2.8,label='Bézier V2 certificado'),
             Line2D([0],[0],color='black',lw=0,marker='s',label='Inicio'),
             Line2D([0],[0],color='black',lw=0,marker='*',label='Destino')]
    fig.legend(handles=handles,loc='upper center',ncol=4,bbox_to_anchor=(.5,.984))
    statuses=' | '.join(f'{s}: {products[s]["report"].get("status")}' for s in STAGES)
    fig.suptitle(f'SwarmBots · CBS + Hybrid V2 · Misión offline · seed {scenario.seed}',fontsize=18,weight='bold',y=.995)
    fig.text(.03,.021,statuses,fontsize=10)
    fig.tight_layout(rect=[0,.04,1,.955])
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(output,dpi=170);plt.close(fig)


def export_report(scenario,products,summary,filename,controls_dir=None):
    data={'seed':scenario.seed,'note':'Planificación offline, NO valida ejecución física en Gazebo',
          'starts_grid':summary['starts'],'picked_round1':summary['picked1'],
          'picked_round2':summary['picked2'],'stages':{}}
    if controls_dir: Path(controls_dir).mkdir(parents=True,exist_ok=True)
    for stage in STAGES:
        item=products[stage]; p=item['plan']; r=item['report']
        entry={'cbs_cost':p.cost,'cbs_expanded':p.expanded,'cbs_paths':p.paths,
               'certified':bool(r.get('certified')),'hybrid_v2':r}
        if controls_dir and entry['certified']:
            path=Path(controls_dir)/f'hybrid_v2_{scenario.seed}_{stage}.npz'
            np.savez_compressed(path,**{f'R{i}':np.asarray(c) for i,c in enumerate(item['controls'])},dt_s=r['dt_s'])
            entry['controls_file']=str(path)
        data['stages'][stage]=entry
    filename=Path(filename);filename.parent.mkdir(parents=True,exist_ok=True)
    filename.write_text(json.dumps(data,ensure_ascii=False,indent=2,default=str),encoding='utf-8')
    return data


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    src=ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--seed',type=int);src.add_argument('--scenario')
    ap.add_argument('--output',required=True)
    ap.add_argument('--json-output')
    ap.add_argument('--controls-dir')
    ap.add_argument('--iterations',type=int,default=2)
    ap.add_argument('--margin',type=float,default=.05)
    ap.add_argument('--max-speed',type=float,default=1.2)
    ap.add_argument('--max-accel',type=float,default=1.0)
    ap.add_argument('--max-ct',type=int,default=20000)
    ap.add_argument('--max-time',type=int,default=65)
    ap.add_argument('--maxiter',type=int,default=450)
    ap.add_argument('--show-graph',action='store_true')
    a=ap.parse_args()
    if a.seed is not None and a.seed<0: ap.error('Semilla no negativa')
    if a.scenario: scenario=load_scenario(a.scenario)
    else:
        generator=Path(original.__file__).resolve().with_name('escenario_aleatorio_swarm.py')
        if not generator.is_file(): ap.error(f'Falta el generador {generator}')
        with tempfile.TemporaryDirectory(prefix='swarm_v2_') as tmp:
            scenario_path=Path(tmp)/'scenario.json'
            subprocess.run([sys.executable,str(generator),'--seed',str(a.seed),'--output',str(scenario_path)],check=True)
            scenario=load_scenario(scenario_path)
    products,contexts,summary=plan_four(scenario,max_ct=a.max_ct,max_time=a.max_time,
                                        margin=a.margin,iterations=a.iterations,
                                        max_speed=a.max_speed,max_accel=a.max_accel,maxiter=a.maxiter)
    draw(scenario,products,contexts,summary,a.output,show_graph=a.show_graph)
    report_path=a.json_output or str(Path(a.output).with_suffix('.json'))
    result=export_report(scenario,products,summary,report_path,controls_dir=a.controls_dir)
    print('PNG:',a.output,'\nJSON:',report_path)
    print('Estados:',{s:result['stages'][s]['hybrid_v2']['status'] for s in STAGES})
    return 0 if all(result['stages'][s]['certified'] for s in STAGES) else 2

if __name__=='__main__': raise SystemExit(main())
