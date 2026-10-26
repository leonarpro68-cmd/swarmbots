"""Exportación diagnóstica del plan CBS/Bézier real previo a su ejecución ROS.

No publica comandos ni altera trayectorias. Matplotlib es opcional y solo se
importa para escribir PNG; errores de exportación deben abortar la demo si el
usuario solicitó reportes obligatorios (fail closed).
"""
from __future__ import annotations

import json
from pathlib import Path

from .bezier_optimizer import evaluate_bezier
from .adapter import from_cbs_paths


def _points(robot, n=30):
    out=[]
    for seg in robot.control_points:
        out.extend(evaluate_bezier(seg,j/n) for j in range(n))
    out.append(evaluate_bezier(robot.control_points[-1],1.0))
    return out


def save_run_report(directory, session, bridge, plan, refinement, scaled, *, max_speed, max_accel):
    directory=Path(directory)
    directory.mkdir(parents=True,exist_ok=True)
    safe_session=''.join(c for c in session if c.isalnum() or c in '-_')
    png=directory/f'{safe_session}_cbs_vs_bezier.png'
    data_file=directory/f'{safe_session}_plan.json'
    scenario=bridge.scenario.to_dict() if bridge.scenario else None
    grid=[[[int(v) for v in node] for node in path] for path in plan.paths]
    data={
        'session':session, 'status':'planned_not_yet_executed',
        'scenario':scenario, 'cbs':{'cost':plan.cost,'grid_paths':grid},
        'assigned':list(plan.assigned), 'refinement':refinement.to_dict(),
        'time_scaled':scaled.to_dict(),
        'limits':{'speed_m_s':max_speed,'acceleration_m_s2':max_accel},
    }
    data_file.write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    from .. import geometry
    colors=('#2563eb','#16a34a','#dc2626')
    fig,axes=plt.subplots(1,2,figsize=(16,8),sharex=True,sharey=True)
    fig.suptitle(f'SwarmBots — CBS vs trayectoria híbrida Bézier | costo CBS={plan.cost}, T={scaled.duration_s:.1f} s',fontsize=13,weight='bold')
    obs=refinement.final_corridors.obstacles
    for ax,title in zip(axes,('CBS: caminos discretos','Híbrido: trayectorias Bézier C²')):
        for obstacle in obs:
            ax.add_patch(Circle(obstacle.center,obstacle.exclusion_radius_m,
                                ec='#e58a1f',fc='none',lw=.8,ls=':',alpha=.8))
        if scenario:
            for item in scenario['elements']:
                x,y=item['xy_m']
                is_deposit=item['type']=='deposit'
                ax.scatter([x],[y],marker='s' if is_deposit else 'o',s=65 if is_deposit else 32,
                           c=item['color'] if item['color'] in ('red','blue','green','orange') else '#64748b',
                           edgecolors='#222',linewidths=.5,zorder=4)
                ax.annotate(item['name'],(x,y),xytext=(4,5),textcoords='offset points',fontsize=6)
        ax.set_aspect('equal');ax.grid(alpha=.18);ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)');ax.set_title(title)
    for i,robot in enumerate(scaled.trajectory.robots):
        color=colors[i%len(colors)]
        discrete=[geometry.node_position(tuple(node)) for node in plan.paths[i]]
        smooth=_points(robot)
        axes[0].plot(*zip(*discrete),'-o',color=color,ms=3,lw=1.6,label=robot.name)
        axes[1].plot(*zip(*discrete),'--',color=color,lw=1,alpha=.35)
        axes[1].plot(*zip(*smooth),color=color,lw=2.2,label=robot.name)
        for ax,route in ((axes[0],discrete),(axes[1],smooth)):
            ax.scatter(*route[0],marker='s',s=65,c=color,edgecolors='black',zorder=6)
            ax.scatter(*route[-1],marker='*',s=95,c=color,edgecolors='black',zorder=6)
    for ax in axes:ax.legend(fontsize=9,loc='best')
    fig.tight_layout()
    fig.savefig(png,dpi=150)
    plt.close(fig)
    return dict(json=str(data_file),png=str(png))
