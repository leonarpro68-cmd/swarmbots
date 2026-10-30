"""Demo offline: CBS real -> corredores híbridos -> Bézier -> refinamiento.

No importa rclpy, no publica cmd_vel, no sustituye la misión existente.
Para ver mejora desde una solución poco optimizada: --initial-maxiter 1
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .. import cbs
from ..bridge import PlanningBridge
from .adapter import from_cbs_paths
from .bezier_optimizer import optimize_bezier
from .corridors import build_hybrid_corridors
from .demo_bezier import plot_plan
from .iterative_refinement import refine_iteratively, RefinementError


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',choices=('static','pickup'),default='pickup')
    parser.add_argument('--iterations',type=int,default=5)
    parser.add_argument('--samples',type=int,default=32)
    parser.add_argument('--initial-maxiter',type=int,default=150)
    parser.add_argument('--maxiter',type=int,default=180)
    parser.add_argument('--max-speed',type=float,default=.60)
    parser.add_argument('--max-accel',type=float,default=.50)
    parser.add_argument('--free-tick-waypoints',action='store_true')
    parser.add_argument('--out',type=Path,default=Path('/tmp/swarm_hybrid_iterative.json'))
    parser.add_argument('--png',type=Path,help='PNG opcional; requiere matplotlib')
    args=parser.parse_args()
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
    anchors=not args.free_tick_waypoints
    initial=optimize_bezier(schedule,corridors,anchor_ticks=anchors,maxiter=args.initial_maxiter)
    result=refine_iteratively(schedule,corridors,initial_trajectory=initial,
                               iterations=args.iterations,samples_per_piece=args.samples,
                               maxiter=args.maxiter,anchor_ticks=anchors,
                               max_speed_m_s=args.max_speed,
                               max_acceleration_m_s2=args.max_accel)
    report=result.to_dict()
    report['source']=args.mode
    report['schedule']=schedule.to_dict()
    report['final_corridors']=result.final_corridors.to_dict()
    args.out.parent.mkdir(exist_ok=True,parents=True)
    args.out.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    for item in result.history:
        print(f'Iter={item.index}: costo={item.objective:.8f}, '
              f'duración escalada={item.scaled_duration_s:.2f} s')
    print('Iteraciones aceptadas:',result.iterations_completed)
    print('Motivo de parada:',result.reason)
    print('Reporte offline:',args.out)
    if args.png:
        plot_plan(result.final_trajectory,schedule,corridors.obstacles,args.png)
        print('Gráfico:',args.png)
    print('IMPORTANTE: salida offline, NO es una orden válida de seguimiento ROS 2.')


if __name__=='__main__':
    main()
