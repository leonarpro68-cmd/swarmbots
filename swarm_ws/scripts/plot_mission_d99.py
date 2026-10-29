#!/usr/bin/env python3
"""Lámina 2x2 de rutas para las cuatro etapas; D5.1 y D9.9 comparten geometría."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from d99_plot_common import stages_from_args, samples_for_stage

COLORS = ['#1679ba', '#188d34', '#e31a1c']
STAGE_NAMES = {'pickup1':'Recogida 1', 'delivery1':'Entrega 1',
               'pickup2':'Recogida 2', 'delivery2':'Entrega 2'}
OBJECT_COLORS = {'verde':'#12bb50', 'naranja':'#ff731f', 'azul':'#2167d3'}

def draw_scenario(ax, scenario):
    if not scenario.exists():
        return
    data = json.loads(scenario.read_text(encoding='utf-8'))
    for element in data.get('elements', []):
        xy = element.get('xy_m')
        if not xy:
            continue
        color = OBJECT_COLORS.get(element.get('color'), '#777777')
        is_trash = element.get('type') == 'trash'
        if is_trash:
            ax.scatter(*xy, s=72, marker='o', facecolor=color, edgecolor='black', linewidth=.8, zorder=6)
        elif element.get('type') == 'deposit':
            ax.scatter(*xy, s=98, marker='s', facecolor=color, edgecolor='black', linewidth=.8, zorder=6)
        else:
            continue
        ax.annotate(element.get('name', ''), xy, xytext=(3,2), textcoords='offset points',
                    fontsize=6.4, color=color, zorder=7)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument('--run-dir', type=Path)
    src.add_argument('--manifest', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--stages', nargs='+', default=None)
    parser.add_argument('--step-q', type=float, default=.015)
    args = parser.parse_args()
    if args.step_q <= 0: parser.error('--step-q debe ser positivo')
    run, stages, reports, scenarios = stages_from_args(args.run_dir, args.manifest, args.stages)
    fig, axes = plt.subplots(2, 2, figsize=(13.4, 17), squeeze=False)
    for j, stage in enumerate(stages):
        ax = axes[j//2,j%2]
        rep, z, _ = samples_for_stage(run, stage, reports[stage], args.step_q)
        draw_scenario(ax, scenarios[stage])
        for i, color in enumerate(COLORS):
            xy = z[f'R{i}_position_xy']
            ax.plot(xy[:,0], xy[:,1], color=color, lw=2.0, zorder=3)
            ax.scatter(*xy[0], marker='s', s=51, color=color, zorder=7)
            ax.annotate(f'S{i}', xy[0], xytext=(3,3), textcoords='offset points',
                        color=color, fontsize=8, weight='bold')
            ax.scatter(*xy[-1], marker='*', s=105, color=color, zorder=7)
        ax.set_title(f'{STAGE_NAMES[stage]} · Hybrid V2 + D9.9 · {rep["optimized_makespan_s"]:.1f} s',
                     fontsize=10.3, fontweight='bold')
        ax.set_xlim(-14.1, 14.1); ax.set_ylim(-24,24)
        ax.set_xticks([-10,-5,0,5,10]); ax.set_yticks([-20,-10,0,10,20])
        ax.set_xlabel('X (m)', fontsize=9); ax.set_ylabel('Y (m)', fontsize=9)
        ax.grid(alpha=.18)
        ax.set_aspect('equal', adjustable='box')
        for spine in ax.spines.values(): spine.set_linewidth(1.1)
    for j in range(len(stages),4): axes[j//2,j%2].axis('off')
    legend=[Line2D([0],[0], color='#444', lw=2, label='Bézier planificada'),
            Line2D([0],[0], marker='s', color='#333', linestyle='None', markersize=6, label='Inicio etapa'),
            Line2D([0],[0], marker='*', color='#333', linestyle='None', markersize=9, label='Fin etapa')]
    fig.legend(handles=legend, loc='upper center', bbox_to_anchor=(.5,.973), ncol=3, fontsize=9)
    fig.suptitle(f'SwarmBots D9.9 · trayectorias planificadas · {run.name}',
                 fontsize=14, fontweight='bold', y=.991)
    fig.text(.5,.008,'Misma geometría para D5.1 y D9.9. Curvas planificadas offline; no son recorridos medidos en Gazebo.',
             fontsize=8, ha='center', color='#444')
    fig.tight_layout(rect=[0,.018,1,.951], h_pad=2.1, w_pad=1.5)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output,dpi=175,facecolor='white'); plt.close(fig)
    print('RUTAS:',args.output)

if __name__=='__main__':main()
