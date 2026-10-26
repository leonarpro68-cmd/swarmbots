"""Prueba OFFLINE: CBS real -> corredores Fase 2 -> Bézier C² Fase 3.

Desde ~/swarmbots (con PYTHONPATH=swarm_ws/src/swarm_behavior):
  python3 -m swarm_behavior.swarm_planning.hybrid_refinement.demo_bezier \
      --mode pickup --out /tmp/swarm_bezier_pickup.json

NUNCA publica /cmd_vel, ni ROS 2, ni Gazebo. El JSON es diagnóstico.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path

from .. import cbs
from ..bridge import PlanningBridge
from .adapter import from_cbs_paths
from .corridors import build_hybrid_corridors
from .bezier_optimizer import (BezierOptimizationError,
                               optimize_bezier, scale_trajectory_time,
                               evaluate_bezier)


def plot_plan(result, schedule, obstacles, path):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    fig,ax=plt.subplots(figsize=(10,8))
    colors=('#2563eb','#ea580c','#16a34a')
    for obs in obstacles:
        ax.add_patch(Circle(obs.center,obs.exclusion_radius_m,
                            facecolor='#e94e77',edgecolor='#dc2626',alpha=.05))
    for i,robot in enumerate(result.robots):
        xy=[]
        for seg in robot.control_points:
            for n in range(24):
                xy.append(evaluate_bezier(seg,n/24))
        xy.append(evaluate_bezier(robot.control_points[-1],1))
        color=colors[i%len(colors)]
        ax.plot(*zip(*schedule.metric_paths[i]),'--',lw=1,color=color,alpha=.45)
        ax.plot(*zip(*xy),lw=2.2,color=color,label=robot.name)
    ax.set_aspect('equal');ax.grid(alpha=.12)
    ax.set(xlabel='X del mapa (m)',ylabel='Y del mapa (m)',
           title='CBS (discontinuo) vs. Bézier C² (offline)')
    ax.legend();fig.tight_layout()
    path.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(path,dpi=145)
    plt.close(fig)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mode',choices=('static','pickup'),default='pickup')
    ap.add_argument('--out',type=Path,default=Path('/tmp/swarm_hybrid_bezier.json'))
    ap.add_argument('--png',type=Path,help='Gráfico opcional; requiere matplotlib')
    ap.add_argument('--max-speed',type=float,default=.60)
    ap.add_argument('--max-accel',type=float,default=.50)
    ap.add_argument('--maxiter',type=int,default=180)
    ap.add_argument('--free-tick-waypoints',action='store_true',
                    help='Permite desviarse de waypoints interiores; preserva WAIT')
    args=ap.parse_args()
    starts=((8,18),(14,18),(11,14))
    if args.mode=='static':
        graph=cbs.Graph()
        paths=cbs.solve(starts,((14,18),(8,18),(11,22)),graph=graph)['paths']
    else:
        bridge=PlanningBridge()
        graph=bridge.graph
        paths=bridge.plan_pickup(starts).paths
    schedule=from_cbs_paths(paths,graph=graph,max_speed_m_s=args.max_speed)
    corridors=build_hybrid_corridors(schedule,graph=graph)
    try:
        trajectory=optimize_bezier(schedule,corridors,maxiter=args.maxiter,
                                   anchor_ticks=not args.free_tick_waypoints)
        scaled=scale_trajectory_time(trajectory,max_speed_m_s=args.max_speed,
                                     max_acceleration_m_s2=args.max_accel)
    except BezierOptimizationError as exc:
        ap.exit(2,f'Fase 3 rechazada: {exc}\n')
    data=scaled.to_dict()
    data['source']=args.mode
    data['schedule']=schedule.to_dict()
    data['corridor_count']=corridors.substeps*len(schedule.robot_names)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
    print(f'Fase 3: {args.mode}, {len(trajectory.robots)} robots, {corridors.substeps} piezas/robot')
    for r in trajectory.robots:
        print(f' {r.name}: J={r.objective_acceleration_sq:.4f} '
              f'(semilla={r.seed_objective_acceleration_sq:.4f}); '
              f'SLSQP éxito={r.solver_success}')
    print(f'T nominal={trajectory.initial_duration_s:.2f} s, '
          f'factor temporal={scaled.factor:.3f}, T escalado={scaled.duration_s:.2f} s')
    print(f'JSON offline: {args.out}')
    if args.png:
        plot_plan(trajectory,schedule,corridors.obstacles,args.png)
        print(f'Imagen: {args.png}')
    print('No ejecutar en Summit: faltan integración ROS 2, control de seguimiento y cinodinámica.')


if __name__=='__main__':
    main()
