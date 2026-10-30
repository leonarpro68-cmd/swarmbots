#!/usr/bin/env python3
"""D9.8C.5 offline: CBS wave1 -> clear green depot -> CBS wave2.
No ROS, Gazebo, actuator commands, or executable timing certificates.
"""
import argparse
import json
import math
import sys
from pathlib import Path
from d98c4_wave_cbs_preview import (ROBOTS,CLEARANCE,validate_inputs,filtered_graph,nearest,distance)


def single_route(cbs, start, goal, stationary, clearance, max_snap=1.0):
    g=filtered_graph(cbs, stationary, clearance)
    s,se=nearest(g,start,forbidden=stationary,clearance=clearance)
    t,te=nearest(g,goal,forbidden=stationary,clearance=clearance)
    if se>max_snap or te>max_snap:
        raise ValueError(f'SNAP_TOO_FAR start={se} goal={te}')
    result=cbs.solve((s,),(t,),graph=g,max_ct=10000,max_time=120)
    path=result['paths'][0]
    return {'grid_nodes':[list(x) for x in path], 'waypoints_xy_m':[list(g.pos(x)) for x in path],
            'abstract_ticks':len(path)-1,'start_snap_m':se,'goal_snap_m':te}


def build(snapshot,waves,geometry,cbs,clearance=CLEARANCE, park_min_goal=2.8, park_radius=9.0):
    validate_inputs(snapshot,waves,geometry)
    base={'schema':'swarm.d98c5.parking_cbs_offline.v1','execution_authorized':False,
          'commands_sent':0,'gazebo_required':False,'cargo_joint_certified':False,
          'live_odom_certified':False,'wave2_start_positions_predicted':True,
          'hybrid_v2_certified':False,'d92_certified':False,'release_authorized':False,
          'mission_ledger_updated':False,'grid_clearance_m':clearance}
    stage1=waves['waves'][0];stage2=waves['waves'][1]
    g1=geometry['waves'][0];g2=geometry['waves'][1]
    if g1['number']!=1 or g2['number']!=2:raise ValueError('BAD_GEOMETRY_WAVE_ORDER')
    act1=[e['robot'] for e in stage1['robots']]
    if set(act1)!={'summit0','summit2'} or [x['robot'] for x in stage2['robots']]!=['summit1']:
        raise ValueError('UNSUPPORTED_WAVE_ASSIGNMENT')
    starts={r:snapshot['robots'][r]['odom_xy_m'] for r in ROBOTS}
    goals1={e['robot']:e['standoff_goal_xy_m'] for e in g1['nominal_straight_segments']}
    goal2=next(e['standoff_goal_xy_m'] for e in g2['nominal_straight_segments'] if e['robot']=='summit1')
    fixed=[starts['summit1']]
    g=filtered_graph(cbs,fixed,clearance)
    nodes_s=[];nodes_g=[];snapping={}
    for r in act1:
        sv,se=nearest(g,starts[r],forbidden=fixed,clearance=clearance)
        gv,ge=nearest(g,goals1[r],forbidden=fixed,clearance=clearance)
        if max(se,ge)>1:raise ValueError(f'WAVE1_{r}:SNAP_TOO_FAR')
        nodes_s.append(sv);nodes_g.append(gv);snapping[r]=[se,ge]
    solution=cbs.solve(tuple(nodes_s),tuple(nodes_g),graph=g,max_ct=20000,max_time=120)
    if cbs.first_conflict(solution['paths'],clearance):raise ValueError('WAVE1_CONFLICT')
    p1={r:{'grid_nodes':[list(n) for n in path], 'waypoints_xy_m':[list(g.pos(n)) for n in path],
           'abstract_ticks':len(path)-1,'snap_errors_m':snapping[r]}
        for r,path in zip(act1,solution['paths'])}
    end1={r:p1[r]['waypoints_xy_m'][-1] for r in act1}
    # After actual release: summit0 clears green area while summit1, summit2 remain stationary.
    # Candidate parking should preserve clearance from wave2 goal and from depot itself.
    full=cbs.Graph(); fixed_park=[starts['summit1'],end1['summit2']]
    candidates=[]
    for v in sorted(full.vertices):
        pt=full.pos(v)
        if distance(pt,end1['summit0'])>park_radius:continue
        if distance(pt,goal2)<park_min_goal:continue
        if distance(pt,goals1['summit0'])<park_min_goal:continue
        if any(distance(pt,f)<clearance for f in fixed_park):continue
        candidates.append((v,pt))
    candidates.sort(key=lambda it:(distance(it[1],end1['summit0']),-distance(it[1],goal2)))
    attempts=[]
    for v,pt in candidates:
        try:
            park=single_route(cbs,end1['summit0'],pt,fixed_park,clearance)
            # Wave2 path cannot approach parked summit0 or summit2.
            w2=single_route(cbs,starts['summit1'],goal2,[pt,end1['summit2']],clearance)
            if distance(pt,w2['waypoints_xy_m'][-1])<clearance:
                raise ValueError('PARK_TOO_CLOSE_TO_WAVE2')
            base.update({'status':'CBS_DISCRETE_THREE_PHASES_READY_NOT_EXECUTABLE','issues':[],
              'wave1':{'robots':act1,'stationary':['summit1'],'paths':p1,'ct_expanded':solution['expanded']},
              'clear_deposit':{'robot':'summit0','after_release_required':True,
                 'stationary':['summit1','summit2'],'parking_xy_m':list(pt),'path':park},
              'wave2':{'robots':['summit1'],'stationary':['summit0','summit2'],
                  'path':w2,'goal_nominal_xy_m':goal2},
              'parking_candidates_considered':len(attempts)+1,
              'caveats':['Wave1 end positions and parking are predicted grid vertices, not live odometry',
                 'Must verify detachable joints before transport; release before clearing',
                 'CBS ticks are abstract, not executable ROS2 timing',
                 'Hybrid V2 and D9.2 conversion/certification are pending',
                 'Replan clearing and wave2 from fresh odometry after each phase']})
            return base
        except (ValueError,RuntimeError) as e:
            attempts.append(str(e))
    base.update({'status':'BLOCKED_NO_SAFE_PARKING_OR_WAVE2','issues':['NO_VALID_PARKING_ROUTE'],
                 'parking_candidates_checked':len(attempts),'first_failures':attempts[:5],
                 'wave1':{'robots':act1,'paths':p1},'wave2':None})
    return base


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    for k in ('snapshot','waves','geometry'):ap.add_argument('--'+k,required=True,type=Path)
    ap.add_argument('--output',type=Path)
    a=ap.parse_args()
    try:
        from swarm_behavior.swarm_planning import cbs
    except ImportError:
        ap.error('Source ROS setup and set PYTHONPATH=$PWD/scripts:$PWD/src/swarm_behavior:$PYTHONPATH')
    args=[json.loads(getattr(a,k).read_text()) for k in ('snapshot','waves','geometry')]
    result=build(*args,cbs)
    out=a.output or a.geometry.parent/'delivery1_wave_parking_cbs_preview.json'
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({'status':result['status'],'issues':result['issues'],
         'parking':result.get('clear_deposit',{}).get('parking_xy_m'),'output':str(out)},indent=2))
    return 0 if not result['issues'] else 2
if __name__=='__main__':sys.exit(main())
