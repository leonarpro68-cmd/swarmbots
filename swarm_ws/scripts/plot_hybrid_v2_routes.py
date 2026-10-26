#!/usr/bin/env python3
"""Offline PNG for Hybrid V2 stage C.2 (Entrega ronda 2).

Reads certified Bézier control points produced by pipeline_c2. Requires
numpy, matplotlib and the swarm_behavior package on PYTHONPATH.
"""
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from swarm_behavior.swarm_planning.hybrid_v2.scenario import reference_inputs, TRASH, DEPOSITS
from swarm_behavior.swarm_planning.hybrid_v2 import geometry_reference as geo


def bezier_samples(control, n=30):
    t=np.linspace(0,1,n)
    weights=np.array([(1-t)**5,5*t*(1-t)**4,10*t*t*(1-t)**3,
                      10*t**3*(1-t)**2,5*t**4*(1-t),t**5]).T
    return weights @ control


def main():
    p=argparse.ArgumentParser(description='Visualizar offline las trayectorias certificadas Hybrid V2')
    p.add_argument('--controls',type=Path,required=True,help='Archivo .npz producido por pipeline_c2')
    p.add_argument('--output',type=Path,required=True,help='Ruta PNG de salida')
    p.add_argument('--hide-cbs',action='store_true',help='Ocultar rutas discretas de referencia')
    args=p.parse_args()
    if not args.controls.exists(): p.error(f'No existe: {args.controls}')
    node_paths,points,obstacles=reference_inputs()
    colors=['#e35d5b','#2784c2','#27956b']
    fig,ax=plt.subplots(figsize=(10,12),dpi=160)
    ax.set_facecolor('#f9fafc')
    ax.add_patch(Rectangle((geo.X_MIN,geo.Y_MIN),geo.X_MAX-geo.X_MIN,geo.Y_MAX-geo.Y_MIN,fill=False,ec='#6b7280',lw=1.2))
    for c in geo.COLUMNS_XY:
        ax.add_patch(Circle(c,geo.COLUMN_RADIUS_M,fc='#51555b',ec='white',lw=.8,zorder=4))
        ax.add_patch(Circle(c,geo.CLEARANCE_COLUMN_M,fc='none',ec='#7c8086',ls=':',lw=.9,zorder=2))
    for q in TRASH:
        ax.scatter(*q,c='#eaac36',s=65,marker='s',edgecolors='black',linewidths=.5,zorder=5)
    for q in DEPOSITS:
        ax.scatter(*q,c='#7856a6',s=95,marker='D',edgecolors='black',linewidths=.5,zorder=5)
    data=np.load(args.controls)
    dt=float(data['dt_s']) if 'dt_s' in data else None
    for i in range(3):
        key=f'R{i}'
        if key not in data: p.error(f'Archivo NPZ sin clave {key}')
        ctrl=np.asarray(data[key])
        if ctrl.ndim!=3 or ctrl.shape[1:]!=(6,2): p.error(f'Formato inesperado de {key}: {ctrl.shape}')
        path=np.concatenate([bezier_samples(piece) for piece in ctrl],axis=0)
        if not args.hide_cbs:
            ax.plot(points[i,:,0],points[i,:,1],ls='--',lw=1.4,color=colors[i],alpha=.42,zorder=2)
        ax.plot(path[:,0],path[:,1],lw=2.75,color=colors[i],label=f'Summit {i} · Bézier',zorder=6)
        ax.scatter(*path[0],s=80,color=colors[i],marker='o',edgecolors='white',zorder=8)
        ax.scatter(*path[-1],s=95,color=colors[i],marker='*',edgecolors='black',linewidths=.3,zorder=8)
    ax.set(xlim=(geo.X_MIN-.9,geo.X_MAX+.9),ylim=(geo.Y_MIN-.9,geo.Y_MAX+.9),xlabel='X (m)',ylabel='Y (m)')
    ax.set_aspect('equal',adjustable='box'); ax.grid(alpha=.16)
    title='Hybrid V2 · Entrega de ronda 2 · Trayectorias refinadas'
    ax.set_title(title+(f'\nDuración común estimada: {dt*19:.2f} s' if dt else ''),fontsize=13,weight='bold')
    ax.legend(loc='lower right',framealpha=.95)
    fig.text(.5,.04,'Línea continua: Bézier certificada · Discontinua: CBS de referencia · círculo: inicio · estrella: destino',ha='center',fontsize=8)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(args.output,dpi=180,bbox_inches='tight'); plt.close(fig)
    print('PNG:',args.output)

if __name__=='__main__': main()
