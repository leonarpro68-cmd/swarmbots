#!/usr/bin/env python3
"""D9.9.22: cuatro rutas Bézier efectivamente comandadas, diseño D9.9 estilo D8.

Usa las referencias reescaladas del stage_windows.json; no recalcula CBS,
no modifica la misión y no atribuye posiciones odométricas a rutas planificadas.
"""
import argparse
import json
from math import comb
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

STAGES = [('pickup1','Recogida 1'), ('delivery1','Entrega 1'),
          ('pickup2','Recogida 2'), ('delivery2','Entrega 2')]
COLORS = ['#1679ba', '#188d34', '#e31a1c']
OBJECT_COLORS = {'verde':'#12bb50', 'naranja':'#ff731f', 'azul':'#2167d3'}


def curve_points(controls, n=30):
    xy = []
    for P in controls:
        P = np.asarray(P, dtype=float)
        if P.shape != (6, 2):
            raise ValueError(f'Puntos de control Bézier inesperados: {P.shape}')
        u = np.linspace(0, 1, n, endpoint=False)
        xy.append(sum(comb(5,k) * ((1-u)**(5-k) * u**k)[:,None] * P[k]
                      for k in range(6)))
    if not xy:
        raise ValueError('Trayectoria sin piezas Bézier')
    xy.append(np.asarray(controls[-1][-1], dtype=float)[None,:])
    return np.vstack(xy)


def draw_scenario(ax, scenario):
    if not isinstance(scenario, dict):
        return
    for element in scenario.get('elements', []):
        xy = element.get('xy_m')
        if not isinstance(xy, (list, tuple)) or len(xy) != 2:
            continue
        kind = element.get('type')
        if kind not in ('trash', 'deposit'):
            continue
        col = OBJECT_COLORS.get(element.get('color'), '#777777')
        ax.scatter(*xy, s=72 if kind=='trash' else 98,
                   marker='o' if kind=='trash' else 's', facecolor=col,
                   edgecolor='black', linewidth=.8, zorder=6)
        ax.annotate(element.get('name', ''), xy, xytext=(3,2),
                    textcoords='offset points', fontsize=6.4, color=col, zorder=7)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--stage-windows', required=True)
    ap.add_argument('--scenario', default=None)
    ap.add_argument('--output', required=True)
    ap.add_argument('--mission-label', default='seed_100_run_007')
    args = ap.parse_args()
    data = json.loads(Path(args.stage_windows).read_text(encoding='utf-8'))
    by_stage = {w['stage']:w for w in data['stages']}
    if any(s not in by_stage for s,_ in STAGES) or len(by_stage)!=4:
        ap.error('Se requieren las cuatro etapas; no se dibujarán etapas inventadas')
    scenario = None
    if args.scenario:
        path = Path(args.scenario)
        if not path.is_file():
            ap.error(f'Escenario no encontrado: {path}')
        scenario = json.loads(path.read_text(encoding='utf-8'))

    fig,axes=plt.subplots(2,2,figsize=(13.4,17),squeeze=False)
    for j,(stage,label) in enumerate(STAGES):
        ax=axes[j//2,j%2]
        window=by_stage[stage]
        draw_scenario(ax,scenario)
        with np.load(window['trajectory'], allow_pickle=False) as z:
            for i,color in enumerate(COLORS):
                xy=curve_points(z[f'R{i}_controls'])
                ax.plot(xy[:,0],xy[:,1],color=color,lw=2.0,zorder=3)
                ax.scatter(*xy[0],marker='s',s=51,color=color,zorder=7)
                ax.annotate(f'S{i}',xy[0],xytext=(3,3),textcoords='offset points',
                            color=color,fontsize=8,weight='bold')
                ax.scatter(*xy[-1],marker='*',s=105,color=color,zorder=7)
        ax.set_title(f'{label} · Hybrid V2 + D9.9 · {float(window["max_duration_s"]):.1f} s',
                     fontsize=10.3,fontweight='bold')
        ax.set_xlim(-14.1,14.1); ax.set_ylim(-24,24)
        ax.set_xticks([-10,-5,0,5,10]); ax.set_yticks([-20,-10,0,10,20])
        ax.set_xlabel('X (m)',fontsize=9); ax.set_ylabel('Y (m)',fontsize=9)
        ax.grid(alpha=.18)
        ax.set_aspect('equal',adjustable='box')
        for spine in ax.spines.values(): spine.set_linewidth(1.1)
    legend=[Line2D([0],[0],color='#444',lw=2,label='Bézier planificada'),
            Line2D([0],[0],marker='s',color='#333',linestyle='None',markersize=6,label='Inicio etapa'),
            Line2D([0],[0],marker='*',color='#333',linestyle='None',markersize=9,label='Fin etapa')]
    fig.legend(handles=legend,loc='upper center',bbox_to_anchor=(.5,.973),ncol=3,fontsize=9)
    fig.suptitle(f'SwarmBots D9.9 · trayectorias planificadas · {args.mission_label}',
                 fontsize=14,fontweight='bold',y=.991)
    fig.text(.5,.008,'Curvas planificadas ejecutadas en D9.9.22; no son recorridos medidos en Gazebo.',
             fontsize=8,ha='center',color='#444')
    fig.tight_layout(rect=[0,.018,1,.951],h_pad=2.1,w_pad=1.5)
    out=Path(args.output).resolve()
    out.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(out,dpi=175,facecolor='white');plt.close(fig)
    print(json.dumps({'status':'PLOT_READY','output':str(out)},ensure_ascii=False))
    return 0

if __name__=='__main__':
    raise SystemExit(main())
