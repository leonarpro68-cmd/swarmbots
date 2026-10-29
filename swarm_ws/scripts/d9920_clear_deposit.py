#!/usr/bin/env python3
"""D9.9.20 clearance: route CBS from live odom, timed Bézier V2 path_follower, verified odom.

Never emits cmd_vel. One robot moves, other robots must remain stationary.
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


def plan_cbs(start, other_poses, dep_xy, goal2_xy, clearance=1.55, departure=2.8):
    from swarm_behavior.swarm_planning import cbs
    from d98c4_wave_cbs_preview import filtered_graph, nearest
    fixed = list(other_poses.values())
    g = filtered_graph(cbs, fixed, clearance)
    s, snap_s = nearest(g, start, forbidden=fixed, clearance=clearance)
    if snap_s > 0.8:
        raise RuntimeError(f'Start snap {snap_s:.3f} m exceeds 0.8 m')
    candidates = []
    for vertex in g.vertices:
        xy = g.pos(vertex)
        if (2.8 <= math.dist(xy, dep_xy) <= 9.0
            and math.dist(xy, goal2_xy) >= 2.8
            and all(math.dist(xy, p) >= clearance for p in fixed)):
            candidates.append((math.dist(xy, start), vertex))
    if not candidates:
        raise RuntimeError('No free parking vertex')
    for _, goal in sorted(candidates):
        try:
            result = cbs.solve((s,), (goal,), graph=g, max_ct=10000, max_time=120)
            path = [tuple(g.pos(v)) for v in result['paths'][0]]
            # First segment must reach graph from live pose without proximity hazard.
            if any(cbs.geom.segment_point_distance(start, path[0], p) < clearance for p in fixed):
                continue
            if any(cbs.geom.segment_point_distance(u, v, p) < clearance
                   for u, v in zip(path, path[1:]) for p in fixed):
                continue
            # Avoid ending on top of deposit and near future delivery point.
            return [tuple(start), *path], tuple(path[-1]), snap_s
        except Exception:
            continue
    raise RuntimeError('CBS could not find stationary-robot-safe clearing route')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stage', required=True, choices=('delivery1', 'delivery2'))
    p.add_argument('--prior-wave-report', required=True, type=Path)
    p.add_argument('--scenario', default='scripts/d996_reference/scenario.json', type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--execute', action='store_true')
    p.add_argument('--timeout-s', type=float, default=100.)
    a = p.parse_args()
    report = {'schema': 'swarm.d9920.clearance.v1', 'stage': a.stage, 'status':'REJECT',
              'commands_may_have_been_sent': False, 'problems': [],
              'after_wave_report': str(a.prior_wave_report.resolve())}
    try:
        prior = json.loads(a.prior_wave_report.read_text())
        if prior.get('stage') != a.stage or prior.get('status') != 'WAVE_COMPLETED_ACK_REQUIRES_GAZEBO_INSPECTION' or prior.get('wave_requested') != 1:
            raise RuntimeError('Wave1 report does not match this stage')
        if 'summit0' not in prior.get('completed_robots', []):
            raise RuntimeError('Summit0 did not complete wave1')
        scenario = json.loads(a.scenario.read_text())
        elems = {x['name']: x for x in scenario['elements']}
        assignments = prior['assignments']
        shared_deposit = assignments['summit0']['deposit']
        if assignments['summit1']['deposit'] != shared_deposit:
            raise RuntimeError('No shared deposit for summit0/summit1')
        dep_xy = tuple(map(float, elems[shared_deposit]['xy_m']))
        # The desired wave2 standoff is measured from the same live position and the deposit.
        from d6_grasp_existing import snapshot_odom
        def get_poses():
            raw = snapshot_odom()
            xy = {}
            speed = {}
            for r in ROBOTS:
                x = raw[r]
                xy[r] = tuple(map(float, x.get('xy', x.get('xy_m'))))
                speed[r] = float(x.get('speed_m_s', x.get('speed', 0.0)))
            return xy, speed
        poses, speeds = get_poses()
        if any(speed > 0.08 for speed in speeds.values()):
            raise RuntimeError(f'Robots not stationary: {speeds}')
        wave2_robot = poses['summit1']
        dx, dy = wave2_robot[0]-dep_xy[0], wave2_robot[1]-dep_xy[1]
        dist = math.hypot(dx, dy)
        if dist < 0.1:
            raise RuntimeError('Wave2 robot currently on deposit')
        standoff = (dep_xy[0] + 0.8 * dx / dist, dep_xy[1] + 0.8 * dy / dist)
        path, goal, snap = plan_cbs(poses['summit0'], {r:poses[r] for r in ROBOTS if r!='summit0'}, dep_xy, standoff)
        report.update({'route_xy_m': path, 'parking_xy_m': goal, 'start_snap_m': snap,
                       'initial_odometry': poses})
        if not a.execute:
            report['status']='CLEARANCE_ROUTE_PLANNED_NO_COMMANDS'
        else:
            from d9920t_timed_aux import execute_route
            report['status']='CLEARANCE_IN_PROGRESS'
            write(a.output,report)
            report['commands_may_have_been_sent']=True
            write(a.output,report)
            details=execute_route(path, output_report=a.output, timeout_s=a.timeout_s)
            report.update(details)
            write(a.output,report)
            final, final_speed=get_poses()
            report['final_odometry']=final
            if final_speed['summit0'] > 0.08 or math.dist(final['summit0'],goal) > .12:
                raise RuntimeError('Parking pose or velocity not verified')
            if math.dist(final['summit0'],dep_xy) < 2.8 or math.dist(final['summit0'],standoff) < 2.8:
                raise RuntimeError('Deposit or wave2 approach not cleared')
            report['status']='DEPOSIT_CLEAR_VERIFIED'
    except Exception as e:
        report['problems'].append(f'{type(e).__name__}: {e}')
        report['status']='REJECT_STOPPED'
    write(a.output,report)
    print(json.dumps({'status': report['status'], 'problems': report['problems'],
                      'parking_xy_m':report.get('parking_xy_m'), 'output':str(a.output)}, ensure_ascii=False))
    return 0 if report['status'] in ('CLEARANCE_ROUTE_PLANNED_NO_COMMANDS','DEPOSIT_CLEAR_VERIFIED') else 2

if __name__ == '__main__':
    sys.exit(main())
