#!/usr/bin/env python3
"""Distance-only audit of CBS approach goals to D8 target centers.

No reachability conclusion is possible from Euclidean distance alone.
"""
import argparse,json,math
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from swarm_maps import map_files
from audit_phase4 import create_mission

def run(cbs,scene,geometry):
    mission=create_mission(cbs)
    named={x['name']:x for x in scene['trash']+scene['deposits']}
    o=geometry['graph']['origin'];h=geometry['graph']['step_m']
    output={'status':'DISTANCE_ONLY_D8_COMPATIBILITY_NOT_VERIFIED','execution_authorized':False,'stages':{}}
    for st in mission['best']['stages']:
        rows=[]
        for i in range(3):
            g=st['selected_goals'][i]; xy=[o[d]+h*g[d] for d in (0,1)]
            target=st['targets'][i]; center=named[target]['xy_m']
            rows.append({'robot':f'summit{i}','target':target,'approach_xy_m':xy,
                        'target_xy_m':center,'distance_xy_m':round(math.dist(xy,center),5),
                        'd8_reach_verified':False})
        output['stages'][st['stage']]=rows
    return output

def main():
    p=argparse.ArgumentParser();p.add_argument('--cbs',type=Path,required=True)
    p.add_argument('--scenario',type=Path,required=True);p.add_argument('--geometry',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():p.error('No se sobrescribe: '+str(a.output))
    report=run(json.loads(a.cbs.read_text()),json.loads(a.scenario.read_text()),json.loads(a.geometry.read_text()))
    a.output.parent.mkdir(exist_ok=True,parents=True)
    a.output.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    for stage,rows in report['stages'].items():
        print(stage,[(r['robot'],r['target'],r['distance_xy_m']) for r in rows])
    print('No se ha validado el alcance ni orientación para D8.')
if __name__=='__main__':main()
