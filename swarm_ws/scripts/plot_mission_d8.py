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



def sample_kinematics(controls: np.ndarray, dt_s: float, samples_per_piece: int=30):
    """Derivadas analíticas de segmentos Bézier quínticos con dt uniforme.

    En los nudos se conserva la muestra del extremo derecho del segmento
    anterior: no se ocultan posibles discontinuidades entre segmentos.
    """
    if not np.isfinite(dt_s) or dt_s <= 0:
        raise ValueError('dt_s debe ser positivo y finito')
    if samples_per_piece < 3:
        raise ValueError('samples_per_piece debe ser >= 3')
    c = np.asarray(controls, dtype=float)
    if c.ndim != 3 or c.shape[1:] != (6, 2) or not len(c) or not np.isfinite(c).all():
        raise ValueError('Controles Bézier inválidos')
    u = np.linspace(0.0, 1.0, samples_per_piece)
    def bernstein(degree):
        from math import comb
        return np.stack([comb(degree,k)*u**k*(1-u)**(degree-k)
                         for k in range(degree+1)], axis=1)
    b4, b3 = bernstein(4), bernstein(3)
    v_control = 5.0 * (c[:,1:,:] - c[:,:-1,:]) / dt_s
    a_control = 20.0 * (c[:,2:,:] - 2*c[:,1:-1,:] + c[:,:-2,:]) / dt_s**2
    vxy = np.einsum('sk,nkd->nsd', b4, v_control)
    axy = np.einsum('sk,nkd->nsd', b3, a_control)
    t = np.arange(len(c))[:,None]*dt_s + u[None,:]*dt_s
    return t.ravel(), np.linalg.norm(vxy,axis=2).ravel(), np.linalg.norm(axy,axis=2).ravel()


def render_kinematics(run: Path, output: Path, samples_per_piece: int=30,
                      max_speed: float=1.20, max_accel: float=1.00):
    if not np.isfinite(max_speed) or max_speed <= 0 or not np.isfinite(max_accel) or max_accel <= 0:
        raise ValueError('Los límites deben ser positivos y finitos')
    fig, axes = plt.subplots(4, 2, figsize=(17, 16), squeeze=False)
    metrics = []
    for j, (stage,title) in enumerate(zip(STAGES,TITLES)):
        _, ctrls, duration = load_stage(run,stage)
        # El tiempo por pieza es el mismo del protocolo utilizado por D.8;
        # se lee del archivo, nunca se infiere solo de la duración de la misión.
        npz_path=run/'mission'/stage/'protocol'/f'{stage}_protocol_controls.npz'
        with np.load(npz_path, allow_pickle=False) as z:
            if 'dt_s' not in z:
                raise ValueError(f'{stage}: falta dt_s en los controles')
            dt = float(z['dt_s'])
        if len({len(c) for c in ctrls}) != 1:
            raise ValueError(f'{stage}: distinta cantidad de piezas entre robots')
        axv, axa = axes[j]
        for i,ctrl in enumerate(ctrls):
            t, speeds, accels = sample_kinematics(ctrl,dt,samples_per_piece)
            color=original.ROBOT_COLORS[i]
            axv.plot(t,speeds,color=color,label=f'Summit {i}',linewidth=1.5)
            axa.plot(t,accels,color=color,label=f'Summit {i}',linewidth=1.5)
            metrics.append({'stage':stage,'robot':f'summit{i}',
                            'max_speed_m_s':float(speeds.max()),
                            'max_accel_m_s2':float(accels.max()),
                            'duration_s':float(t[-1])})
        axv.axhline(max_speed,linestyle='--',color='black',alpha=.65,label=f'Límite {max_speed:g} m/s')
        axa.axhline(max_accel,linestyle='--',color='black',alpha=.65,label=f'Límite {max_accel:g} m/s²')
        for ax, ylabel in ((axv,'Rapidez (m/s)'),(axa,'Aceleración |a| (m/s²)')):
            ax.set_title(f'{title} · {ylabel}',fontweight='bold')
            ax.set_xlabel('Tiempo de trayectoria (s)')
            ax.set_ylabel(ylabel)
            ax.set_xlim(left=0,right=dt*len(ctrls[0]))
            ax.set_ylim(bottom=0)
            ax.grid(True,alpha=.25)
        if j==0:
            axv.legend(loc='upper right',fontsize=9,ncol=2)
            axa.legend(loc='upper right',fontsize=9,ncol=2)
    fig.suptitle(f'SwarmBots D.8 · velocidad y aceleración Bézier · {run.name}',fontsize=16,fontweight='bold')
    fig.text(.03,.01,'Derivadas analíticas de las trayectorias protocol/NPZ (sin maniobras locales RGB-D). No son velocidades medidas en Gazebo.',fontsize=9)
    fig.tight_layout(rect=(0,.02,1,.965))
    output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(output,dpi=150,bbox_inches='tight')
    plt.close(fig)
    return metrics

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
    ap.add_argument('--kinematics',action='store_true',help='Generar además PNG de velocidad y aceleración por etapa')
    ap.add_argument('--kinematics-output',type=Path,help='Ruta opcional del PNG de cinemática')
    ap.add_argument('--samples-per-piece',type=int,default=30)
    ap.add_argument('--max-speed',type=float,default=1.20)
    ap.add_argument('--max-accel',type=float,default=1.00)
    args=ap.parse_args()
    out=args.output or args.run/'mission_hybrid_v2.png'
    try:
        rows=render(args.run,out,args.show_cbs)
        if args.kinematics:
            kin_out=args.kinematics_output or args.run/'mission_hybrid_v2_cinematica.png'
            kin_rows=render_kinematics(args.run,kin_out,args.samples_per_piece,args.max_speed,args.max_accel)
            print('PNG cinemática:',kin_out)
            for row in kin_rows:
                print(f"{row['stage']} {row['robot']}: vmax={row['max_speed_m_s']:.3f} m/s, amax={row['max_accel_m_s2']:.3f} m/s²")
    except (ValueError,FileNotFoundError,KeyError) as exc:
        ap.error(str(exc))
    print('PNG:',out)
    print('Inicios Bézier pickup1:',rows[0]['initial_bezier_xy_m'])
    print('Nota: figura de trayectorias planificadas (no telemetría de seguimiento).')

if __name__=='__main__':main()
