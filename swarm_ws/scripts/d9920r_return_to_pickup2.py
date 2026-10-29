#!/usr/bin/env python3
"""D9.9.20-R: safe CBS reposition from delivery1 parking to pickup2 historical start.

Uses existing path_follower; two other Summit remain stationary. No cmd_vel.
"""
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

ROBOTS = ('summit0', 'summit1', 'summit2')


def write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False) + '\n', encoding='utf8')
    os.replace(tmp, path)


def plan_cbs(start, other_poses, target_xy, clearance=1.55):
    """Plan single-moving-robot CBS path around both stationary Summit footprints.

    The exact start and goal connector segments must also pass the static
    robot clearance checks. A plan is rejected if either snap is too large.
    """
    from swarm_behavior.swarm_planning import cbs
    from d98c4_wave_cbs_preview import filtered_graph, nearest
    fixed = list(other_poses.values())
    if any(math.dist(start, p) < clearance for p in fixed):
        raise RuntimeError('Start overlaps stationary robot safety radius')
    if any(math.dist(target_xy, p) < clearance for p in fixed):
        raise RuntimeError('Target overlaps stationary robot safety radius')
    graph = filtered_graph(cbs, fixed, clearance)
    s, snap_s = nearest(graph, start, forbidden=fixed, clearance=clearance)
    g, snap_g = nearest(graph, target_xy, forbidden=fixed, clearance=clearance)
    if snap_s > .8 or snap_g > .8:
        raise RuntimeError(f'Graph snap too large: start={snap_s:.3f}, goal={snap_g:.3f} m')
    result = cbs.solve((s,), (g,), graph=graph, max_ct=10000, max_time=120)
    grid_path = [tuple(graph.pos(v)) for v in result['paths'][0]]
    path = [tuple(start), *grid_path, tuple(target_xy)]
    for a,b in zip(path, path[1:]):
        if any(cbs.geom.segment_point_distance(a, b, p) < clearance for p in fixed):
            raise RuntimeError('Connector/route violates stationary clearance')
    return path, tuple(target_xy), snap_s, snap_g


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trajectory', required=True, type=Path)
    p.add_argument('--prior-clearance-report', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--execute', action='store_true')
    p.add_argument('--timeout-s', type=float, default=100.)
    a = p.parse_args()
    report = {'schema':'swarm.d9920r.transition.v1','status':'REJECT_STOPPED',
              'commands_may_have_been_sent':False,'problems':[],
              'trajectory':str(a.trajectory.resolve()),
              'prior_clearance_report':str(a.prior_clearance_report.resolve())}
    try:
        prior = json.loads(a.prior_clearance_report.read_text(encoding='utf8'))
        if prior.get('status') != 'DEPOSIT_CLEAR_VERIFIED' or prior.get('stage') != 'delivery1':
            raise RuntimeError('Missing verified delivery1 clearance report')
        import numpy as np
        with np.load(a.trajectory, allow_pickle=False) as npz:
            targets = {f'summit{i}': tuple(map(float,npz[f'R{i}_controls'][0,0])) for i in range(3)}
        from d6_grasp_existing import snapshot_odom
        def get_poses():
            raw = snapshot_odom()
            xy, speed = {}, {}
            for r in ROBOTS:
                x=raw[r]
                xy[r]=tuple(map(float, x.get('xy',x.get('xy_m'))))
                speed[r]=float(x.get('speed_m_s',x.get('speed',0.0)))
            return xy,speed
        poses,speeds=get_poses()
        if any(v > .08 for v in speeds.values()):
            raise RuntimeError(f'Robots not stationary: {speeds}')
        if math.dist(poses['summit0'], prior['parking_xy_m']) > .30:
            raise RuntimeError('Summit0 not at verified parking position')
        # Summit1/2 are already at their historical pickup2 starts, within
        # conservative alignment radius. Otherwise a new joint plan is needed.
        for r in ('summit1','summit2'):
            if math.dist(poses[r], targets[r]) > .30:
                raise RuntimeError(f'{r} too far from pickup2 target; must replan jointly')
        path,goal,snap_s,snap_g=plan_cbs(poses['summit0'],
                         {r:poses[r] for r in ROBOTS if r!='summit0'}, targets['summit0'])
        report.update({'route_xy_m':path, 'target_xy_m':goal,
                      'initial_odometry':poses, 'start_snap_m':snap_s,
                      'goal_snap_m':snap_g})
        if not a.execute:
            report['status']='TRANSITION_ROUTE_PLANNED_NO_COMMANDS'
        else:
            from d9920t_timed_aux import execute_route
            report['status']='TRANSITION_IN_PROGRESS'
            write(a.output,report)
            report['commands_may_have_been_sent']=True
            write(a.output,report)
            details=execute_route(path, output_report=a.output, timeout_s=a.timeout_s)
            report.update(details)
            write(a.output,report)
            final,final_speed=get_poses()
            report['final_odometry']=final
            if any(v>.08 for v in final_speed.values()):
                raise RuntimeError(f'Final robots still moving: {final_speed}')
            if math.dist(final['summit0'],goal)>.12:
                raise RuntimeError('Summit0 not near pickup2 start after transition')
            # All three are now eligible for the existing fine aligner.
            for r in ROBOTS:
                if math.dist(final[r],targets[r])>.30:
                    raise RuntimeError(f'{r} outside fine alignment radius')
            report['status']='TRANSITION_COMPLETED_VERIFIED'
    except Exception as exc:
        report['problems'].append(f'{type(exc).__name__}: {exc}')
        report['status']='REJECT_STOPPED'
    write(a.output,report)
    print(json.dumps({'status':report['status'],'problems':report['problems'],
                     'target_xy_m':report.get('target_xy_m'),
                     'output':str(a.output)},ensure_ascii=False))
    return 0 if report['status'] in ('TRANSITION_ROUTE_PLANNED_NO_COMMANDS','TRANSITION_COMPLETED_VERIFIED') else 2

if __name__=='__main__':sys.exit(main())
