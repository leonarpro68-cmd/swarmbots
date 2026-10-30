"""CBS real -> adaptador -> corredores híbridos, sin ROS ni Gazebo.

Ejemplos desde ~/swarmbots (PYTHONPATH apuntando al paquete):
  python3 -m swarm_behavior.swarm_planning.hybrid_refinement.demo_corridors \
    --mode static --out /tmp/swarm_corridors_static.json
  python3 -m swarm_behavior.swarm_planning.hybrid_refinement.demo_corridors \
    --mode pickup --out /tmp/swarm_corridors_pickup.json

Los JSON exportados son GEOMÉTRICOS, no trayectorias ejecutables.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .. import cbs, geometry as geom
from ..bridge import PlanningBridge
from .adapter import from_cbs_paths
from .corridors import build_hybrid_corridors, CorridorError


def _parse_starts(value: str):
    try:
        starts=tuple(tuple(int(i) for i in part.split(',')) for part in value.split(';'))
    except ValueError as exc:
        raise argparse.ArgumentTypeError('Usa i,j;i,j;i,j') from exc
    if len(starts)!=3 or any(len(v)!=2 for v in starts):
        raise argparse.ArgumentTypeError('Se esperan tres posiciones i,j;i,j;i,j')
    return starts


def plot_tick(plan, graph, tick: int, filename: Path):
    """Vista diagnóstica opcional; matplotlib NO es dependencia de la biblioteca."""
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle, Polygon as PatchPolygon

    if not 0 <= tick < plan.original_ticks:
        raise ValueError('tick fuera de rango')
    factor=plan.subdivisions_per_tick
    inds=sorted(set(round(i*(factor-1)/3) for i in range(min(4,factor))))
    # Si factor=1, solo un panel. En otros casos se reparten en el tick.
    if factor>1:
        inds=sorted(set(round(i*(factor-1)/min(3,factor-1))
                        for i in range(min(4,factor))))
    colors=('#2563eb','#ea580c','#16a34a')
    fig, axes=plt.subplots(1,len(inds),figsize=(5.3*len(inds),6),squeeze=False)
    for ax, part in zip(axes.flat,inds):
        row=plan.corridors[tick*factor+part]
        focus=[p for c in row for p in (c.start,c.end)]
        xmin=min(p[0] for p in focus)-4; xmax=max(p[0] for p in focus)+4
        ymin=min(p[1] for p in focus)-4; ymax=max(p[1] for p in focus)+4
        ax.add_patch(Rectangle((geom.X_MIN, geom.Y_MIN),
                               geom.X_MAX-geom.X_MIN, geom.Y_MAX-geom.Y_MIN,
                               fill=False,linewidth=1.5,edgecolor='#64748b'))
        for o in plan.obstacles:
            if xmin-o.exclusion_radius_m <= o.center[0] <= xmax+o.exclusion_radius_m and \
               ymin-o.exclusion_radius_m <= o.center[1] <= ymax+o.exclusion_radius_m:
                ax.add_patch(Circle(o.center,o.exclusion_radius_m,fill=False,
                                    linewidth=.7,linestyle='--',edgecolor='#dc2626',alpha=.5))
        for i,c in enumerate(row):
            ax.add_patch(PatchPolygon(c.polygon,closed=True,facecolor=colors[i%3],
                                      edgecolor=colors[i%3],lw=1.4,alpha=.18))
            ax.plot([c.start[0],c.end[0]],[c.start[1],c.end[1]],'-o',
                    lw=2.8,ms=5,color=colors[i%3],label=c.robot_name)
        ax.set_aspect('equal');ax.grid(alpha=.15)
        ax.set_xlim(max(xmin,-14.5),min(xmax,14.5))
        ax.set_ylim(max(ymin,-24.5),min(ymax,24.5))
        ax.set_title(f'Tick CBS {tick}, parte {part+1}/{factor}')
        ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)')
        ax.legend(fontsize=8)
    fig.suptitle('SwarmBots | corredores híbridos por subintervalo (NO Bézier)',fontsize=14)
    fig.tight_layout()
    filename.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(filename,dpi=145)
    plt.close(fig)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mode', choices=('static','pickup'), default='static')
    ap.add_argument('--scenario', type=Path, help='JSON de escenario; solo con --mode pickup')
    ap.add_argument('--starts',type=_parse_starts, default=((8,18),(14,18),(11,14)))
    ap.add_argument('--out',type=Path,default=Path('/tmp/swarm_cbs_hybrid_corridors.json'))
    ap.add_argument('--max-speed',type=float,default=.60)
    ap.add_argument('--png',type=Path,help='Imagen diagnóstica de corredores (requiere matplotlib)')
    ap.add_argument('--plot-tick',type=int,default=1,help='Tick CBS para imagen, por defecto 1')
    ap.add_argument('--max-subdivisions',type=int,default=64)
    args=ap.parse_args()
    if args.scenario and args.mode != 'pickup':
        ap.error('--scenario solo se aplica a --mode pickup')
    if args.mode=='static':
        graph=cbs.Graph()
        result=cbs.solve(args.starts,((14,18),(8,18),(11,22)),graph=graph)
        paths=result['paths']
        label='cbs.solve (prueba estática)'
    else:
        bridge=(PlanningBridge.from_json(args.scenario) if args.scenario
                else PlanningBridge())
        graph=bridge.graph
        pickup=bridge.plan_pickup(args.starts)
        paths=pickup.paths
        label='PlanningBridge.plan_pickup (grafo dinámico)'
    schedule=from_cbs_paths(paths,graph=graph,max_speed_m_s=args.max_speed)
    try:
        plan=build_hybrid_corridors(schedule,graph=graph,max_subdivisions=args.max_subdivisions)
    except CorridorError as exc:
        ap.exit(2,f'No se pudo construir los corredores: {exc}\n')
    data=plan.to_dict()
    data['planner_source']=label
    data['schedule']=schedule.to_dict()
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    area=min((c.area_m2 for row in plan.corridors for c in row),default=0)
    waits=sum(c.wait for row in plan.corridors for c in row)
    print(f'Origen: {label}')
    print(f'Robots: {len(plan.robot_names)}; ticks CBS: {plan.original_ticks}; '
          f'subintervalos por tick: {plan.subdivisions_per_tick}')
    print(f'Corredores: {plan.substeps * len(plan.robot_names)}; '
          f'obstáculos circulares: {len(plan.obstacles)}')
    print(f'Área mínima: {area:.6f} m²; segmentos WAIT preservados: {waits}')
    print(f'Duración nominal: {plan.duration_s:.3f} s; '
          f'separación exigida: {plan.min_separation_m:.4f} m')
    print(f'JSON: {args.out}')
    if args.png:
        plot_tick(plan,graph,args.plot_tick,args.png)
        print(f'Imagen diagnóstica: {args.png}')
    print('ATENCIÓN: no son curvas Bézier; no ejecutar en los Summit.')


if __name__=='__main__':
    main()
