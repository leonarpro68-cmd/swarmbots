#!/usr/bin/env python3
"""Prueba offline de corredores de Hybrid V2 en las 4 etapas CBS existentes.

NO llama SLSQP: este paso audita solo la geometría y el contrato de salida.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from swarm_maps import map_files
import numpy as np
from corridors_supermarket import Rectangle, corridors, check_segments_in_corridors
from clearance_config import (RECTANGLE_CENTER_CLEARANCE_M, WALL_CENTER_CLEARANCE_M, INTERROBOT_HALFSEPARATOR_M, INTERROBOT_CENTER_SEPARATION_M, TRASH_CENTER_EXCLUSION_M, VISUAL_DEPOSIT_NAV_EXCLUSION_M)

ROOT=Path(__file__).resolve().parent
TRASH_EXCLUSION_M=TRASH_CENTER_EXCLUSION_M
DEPOSIT_EXCLUSION_M=VISUAL_DEPOSIT_NAV_EXCLUSION_M


from stage_adapter import build_stage


def run(scenario,mission,mapdata, margin=0.0, detailed=False):
    rectangles=tuple(Rectangle.from_json(q) for q in mapdata['obstacles'])
    bounds=tuple(float(mapdata['bounds'][k]) for k in ('xmin','xmax','ymin','ymax'))
    stages=[]
    for stage in mission['best']['stages']:
        points,circles,access=build_stage(scenario,stage,mapdata)
        row={'stage':stage['stage'],'K':len(points[0])-1,'rectangles':len(rectangles),
             'robots':len(points),'accessible_deposits':sorted(access),
             'private_object_counts':[len(o) for o in circles],
             'status':'UNKNOWN'}
        options=dict(rectangles=rectangles,bounds=bounds,static_clearance_m=RECTANGLE_CENTER_CLEARANCE_M,
                     circles_by_robot=circles,operational_margin_m=margin,
                     robot_radius_m=INTERROBOT_HALFSEPARATOR_M,
                     wall_clearance_m=WALL_CENTER_CLEARANCE_M)
        try:
            hs=corridors(points,(),**options)
            issues=check_segments_in_corridors(points,hs)
            if issues:raise AssertionError(f'CBS fuera de semiplanos: {issues[:3]}')
            row['status']='PASS'
            row['halfspaces_total']=sum(len(p) for h in hs for p in h)
            row['minimum_constraints_per_piece']=min(len(p) for h in hs for p in h)
            row['maximum_constraints_per_piece']=max(len(p) for h in hs for p in h)
            row['min_CBS_endpoint_slack_m']=round(min(float(b - np.max(points[i,k:k+2]@n))
                        for i,robot in enumerate(hs) for k,hh in enumerate(robot) for n,b in hh),9)
        except (ValueError,AssertionError) as exc:
            row['status']='FAIL'
            row['reason']=str(exc)
            if detailed:
                failures=[]
                for k in range(len(points[0])-1):
                    try:
                        corridors(points[:,k:k+2],(),**options)
                    except ValueError as e:
                        failures.append({'tick':k,'reason':str(e)})
                row['interval_failures']=failures
        stages.append(row)
    return {'seed':scenario.get('seed'),'source':'swarmbots.zip:hybrid_v2/geometry_corridors.py',
            'model':'seguro geometrico por tramo CBS / sin SLSQP',
            'operational_margin_m':margin,'static_rectangle_clearance_m':RECTANGLE_CENTER_CLEARANCE_M,
            'wall_center_clearance_m':WALL_CENTER_CLEARANCE_M,
            'robot_pair_min_distance_m':INTERROBOT_CENTER_SEPARATION_M,
            'cbs_graph_clearance_m_not_used_in_bezier':mapdata['graph']['clearance_m'],
            'stages':stages,'pass_all':all(s['status']=='PASS' for s in stages),
            'warning':'La aprobación del corredor NO implica factibilidad SLSQP, continuidad C2 ni ejecución Gazebo.'}


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--seed',type=int,default=100)
    p.add_argument('--margin',type=float,default=0.0)
    p.add_argument('--detail',action='store_true')
    args=p.parse_args()
    scenario=json.loads((ROOT/f'examples/seed_{args.seed}.json').read_text())
    mission=json.loads((ROOT/f'examples/mission_{args.seed}.json').read_text())
    data=json.loads(map_files(ROOT.parents[1], 'supermarket')[1].read_text())
    result=run(scenario,mission,data,margin=args.margin,detailed=args.detail)
    path=ROOT/f'results/seed_{args.seed}_corredores.json'
    path.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print('seed',args.seed,'margin',args.margin)
    for s in result['stages']:
        print(s['stage'],s['status'],s.get('reason',''),s.get('halfspaces_total',''))
        if args.detail:
            for issue in s.get('interval_failures',[])[:8]:print(' ',issue)
    print('ALL PASS:',result['pass_all'],'\n',path)
    return 0 if result['pass_all'] else 2

if __name__=='__main__':raise SystemExit(main())
