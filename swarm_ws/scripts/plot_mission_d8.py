#!/usr/bin/env python3
"""PNG 2x2 de las trayectorias PLANIFICADAS por D.8 a partir de archivos reales.

No reconstruye CBS ni supone que las trayectorias se siguieron exactamente en Gazebo.
No confundir el tramo local RGB-D de agarre/entrega con la trayectoria Hybrid V2.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

import plot_mission_routes as original
from plot_hybrid_v2_mission import bezier_points
from swarm_behavior.swarm_planning.scenario import load as load_scenario

STAGES = ('pickup1', 'delivery1', 'pickup2', 'delivery2')
TITLES = ('Recogida 1', 'Entrega 1', 'Recogida 2', 'Entrega 2')


def load_stage(run: Path, stage: str):
    folder = run / 'mission' / stage
    report_file = folder / 'protocol' / f'{stage}_protocol_report.json'
    controls_file = folder / 'protocol' / f'{stage}_protocol_controls.npz'
    if not report_file.is_file() or not controls_file.is_file():
        raise FileNotFoundError(f'{stage}: faltan los archivos protocol JSON/NPZ en {folder}')
    report = json.loads(report_file.read_text(encoding='utf-8'))
    row = report.get('stages', {}).get(stage)
    if not isinstance(row, dict) or not row.get('certified') or not row.get('hybrid_v2', {}).get('certified'):
        raise ValueError(f'{stage}: archivo de trayectoria sin certificación')
    with np.load(controls_file, allow_pickle=False) as z:
        controls = []
        for i in range(3):
            key=f'R{i}'
            if key not in z:
                raise ValueError(f'{stage}: falta {key}')
            arr=np.asarray(z[key], dtype=float)
            if arr.ndim!=3 or arr.shape[1:]!=(6,2) or len(arr)==0 or not np.isfinite(arr).all():
                raise ValueError(f'{stage}: controles Bézier inválidos para {key}: {arr.shape}')
            controls.append(arr)
        duration = float(z['dt_s']) * len(controls[0]) if 'dt_s' in z else None
    return row, controls, duration


def render(run: Path, output: Path, show_cbs: bool=False):
    scenario_path=run/'scenario.json'
    if not scenario_path.is_file():
        raise FileNotFoundError(f'Escenario no encontrado: {scenario_path}')
    scenario=load_scenario(scenario_path)
    fig, axes=plt.subplots(2,2,figsize=(18,23))
    axes=axes.ravel()
    data=[]
    for idx,(stage,title) in enumerate(zip(STAGES,TITLES)):
        row, ctrls, duration=load_stage(run,stage)
        ax=axes[idx]
        original.draw_background(ax)
        original.draw_objects(ax,scenario,pending={e.name for e in scenario.elements if e.kind!='deposit'})
        if show_cbs and 'cbs_paths' in row:
            for i,path in enumerate(row['cbs_paths']):
                xs,ys=original.nodes_to_xy(path)
                ax.plot(xs,ys,'--',color=original.ROBOT_COLORS[i],alpha=.35,lw=1.25,zorder=8)
        for i, ctrl in enumerate(ctrls):
            xy=bezier_points(ctrl)
            col=original.ROBOT_COLORS[i]
            ax.plot(xy[:,0],xy[:,1],color=col,lw=2.7,zorder=10)
            ax.scatter(xy[0,0],xy[0,1],marker='s',s=75,color=col,zorder=11)
            ax.scatter(xy[-1,0],xy[-1,1],marker='*',s=120,color=col,zorder=11)
            ax.annotate(f'S{i}',tuple(xy[0]),xytext=(4,4),textcoords='offset points',color=col,weight='bold')
        duration_label=f' · {duration:.1f} s' if duration is not None else ''
        ax.set_title(f'{title} · Hybrid V2 certificado{duration_label}',weight='bold')
        data.append({'stage':stage,'duration_s':duration,'initial_bezier_xy_m':[ctrl[0,0,:].tolist() for ctrl in ctrls]})
    fig.suptitle(f'SwarmBots D.8 · trayectorias planificadas de la ejecución {run.name}',weight='bold',fontsize=17)
    fig.legend(handles=[Line2D([0],[0],color='gray',lw=2.8,label='Bézier guardada'),Line2D([0],[0],marker='s',lw=0,color='black',label='Inicio etapa'),Line2D([0],[0],marker='*',lw=0,color='black',label='Fin etapa')],loc='upper center',ncol=3,bbox_to_anchor=(.5,.976))
    fig.text(.05,.015,'Fuente: archivos protocol JSON/NPZ de D.8. No incluye maniobras locales RGB-D ni posiciones físicas medidas durante el seguimiento.',fontsize=9)
    fig.tight_layout(rect=(0,.03,1,.95))
    output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(output,dpi=160,bbox_inches='tight')
    plt.close(fig)
    return data


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run',type=Path,required=True,help='Carpeta seed_XXX_run_YYY creada por D.8')
    ap.add_argument('--output',type=Path,help='PNG resultante; por defecto, en la carpeta run')
    ap.add_argument('--show-cbs',action='store_true')
    args=ap.parse_args()
    out=args.output or args.run/'mission_hybrid_v2.png'
    try:
        rows=render(args.run,out,args.show_cbs)
    except (ValueError,FileNotFoundError,KeyError) as exc:
        ap.error(str(exc))
    print('PNG:',out)
    print('Inicios Bézier pickup1:',rows[0]['initial_bezier_xy_m'])
    print('Nota: figura de trayectorias planificadas (no telemetría de seguimiento).')

if __name__=='__main__':main()
