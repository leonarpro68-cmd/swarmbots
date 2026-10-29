#!/usr/bin/env python3
"""D9.8C.4: offline discrete CBS wave preview; NEVER executable trajectories."""
import argparse
import json
import math
import sys
from pathlib import Path

ROBOTS = ('summit0', 'summit1', 'summit2')
CLEARANCE = 1.55


def distance(a, b):
    return math.dist(a, b)


def nearest(graph, point, *, forbidden=(), clearance=CLEARANCE):
    permitted = [v for v in graph.vertices if all(distance(graph.pos(v), p) >= clearance for p in forbidden)]
    if not permitted:
        raise ValueError('NO_GRID_VERTEX_OUTSIDE_STATIONARY_CLEARANCE')
    v = min(permitted, key=lambda x: distance(graph.pos(x), point))
    return v, round(distance(graph.pos(v), point), 5)


def filtered_graph(cbs, points, clearance=CLEARANCE):
    """Remove vertices AND edges that get too close to stationary robot centers."""
    g = cbs.Graph()
    geom = cbs.geom
    g.vertices = {v for v in g.vertices if all(distance(g.pos(v), p) >= clearance for p in points)}
    g.neighbors = {v: [w for w in g.neighbors[v] if w in g.vertices and all(
        geom.segment_point_distance(g.pos(v), g.pos(w), p) >= clearance for p in points)]
                   for v in g.vertices}
    return g


def validate_inputs(snapshot, waves, geometry):
    if snapshot.get('status') != 'LOAD_GEOMETRY_COMPATIBLE_NOT_JOINT_CERTIFIED':
        raise ValueError('BAD_SNAPSHOT')
    if waves.get('status') != 'PROPOSED_WAVES_ONLY_BLOCKED_FOR_EXECUTION':
        raise ValueError('BAD_WAVES')
    if geometry.get('schema') != 'swarm.d98c3.geometry_screen.v1':
        raise ValueError('BAD_GEOMETRY')
    if set(snapshot.get('robots', {})) != set(ROBOTS):
        raise ValueError('BAD_ROBOT_SET')
    sequence = [e['robot'] for w in waves['waves'] for e in w['robots']]
    if sorted(sequence) != sorted(ROBOTS):
        raise ValueError('EACH_ROBOT_MUST_DELIVER_ONCE')
    if [w['number'] for w in waves['waves']] != [1, 2]:
        raise ValueError('BAD_WAVE_ORDER')
    for w in waves['waves']:
        if len({e['deposit_color'] for e in w['robots']}) != len(w['robots']):
            raise ValueError('SAME_DEPOSIT_IN_WAVE')
        for e in w['robots']:
            if snapshot['robots'][e['robot']]['object'] != e['object']:
                raise ValueError('OBJECT_MISMATCH')


def build_preview(snapshot, waves, geometry, cbs, *, clearance=CLEARANCE,
                  max_snap_m=1.0, max_ct=20000):
    validate_inputs(snapshot, waves, geometry)
    if not (clearance >= cbs.MIN_SEPARATION and 0.1 < max_snap_m <= 1.25):
        raise ValueError('INVALID_CLEARANCE_OR_SNAP')
    observed = {r: snapshot['robots'][r]['odom_xy_m'] for r in ROBOTS}
    # Static approximate positions for wave 2 are only placeholders; a fresh post-wave odom is mandatory.
    positions = dict(observed)
    results = []
    failures = []
    for w, geo in zip(waves['waves'], geometry['waves']):
        if w['number'] != geo['number']:
            raise ValueError('WAVE_GEOMETRY_MISMATCH')
        active = [e['robot'] for e in w['robots']]
        stationary = [r for r in ROBOTS if r not in active]
        fixed = [positions[r] for r in stationary]
        g = filtered_graph(cbs, fixed, clearance)
        goals = {e['robot']: e['standoff_goal_xy_m'] for e in geo['nominal_straight_segments']}
        if set(goals) != set(active):
            raise ValueError('GEOMETRY_ACTIVE_SET_MISMATCH')
        try:
            start_nodes, goal_nodes, snapping = [], [], {}
            for r in active:
                sv, se = nearest(g, positions[r], forbidden=fixed, clearance=clearance)
                gv, ge = nearest(g, goals[r], forbidden=fixed, clearance=clearance)
                snapping[r] = {'start_error_m': se, 'goal_error_m': ge}
                if se > max_snap_m or ge > max_snap_m:
                    raise ValueError(f'{r}:SNAP_TOO_FAR')
                start_nodes.append(sv)
                goal_nodes.append(gv)
            solution = cbs.solve(tuple(start_nodes), tuple(goal_nodes), graph=g,
                                 max_ct=max_ct, max_time=120)
            if cbs.first_conflict(solution['paths'], clearance) is not None:
                raise ValueError('POST_CBS_DYNAMIC_CONFLICT')
            path_entries = []
            for r, path in zip(active, solution['paths']):
                xy = [list(g.pos(v)) for v in path]
                for a, b in zip(xy, xy[1:]):
                    if any(cbs.geom.segment_point_distance(a, b, f) < clearance for f in fixed):
                        raise ValueError(f'{r}:STATIC_EDGE_CLEARANCE_FAILURE')
                positions[r] = xy[-1]  # wave 2 proxy: never a substitute for live odom
                path_entries.append({'robot': r, 'grid_nodes': [list(v) for v in path],
                                     'waypoints_xy_m': xy, 'abstract_ticks': len(path)-1,
                                     'snapping': snapping[r]})
            results.append({'number': w['number'], 'status': 'CBS_DISCRETE_PREVIEW_ONLY',
                            'active': active, 'stationary': stationary,
                            'stationary_xy_assumed_m': dict(zip(stationary, fixed)),
                            'ct_expanded': solution['expanded'], 'paths': path_entries})
        except (ValueError, RuntimeError) as err:
            failures.append(f'WAVE_{w["number"]}:{err}')
            results.append({'number': w['number'], 'status': 'NO_CERTIFIED_GRID_PATH',
                            'active': active, 'stationary': stationary, 'error': str(err)})
            # Do not plan later waves based on undefined end positions.
            break
    return {'schema': 'swarm.d98c4.wave_cbs_preview.v1',
            'status': 'CBS_GRID_PREVIEW_READY_NOT_EXECUTABLE' if not failures else 'CBS_GRID_PREVIEW_BLOCKED',
            'issues': failures, 'waves': results,
            'grid_clearance_m': clearance, 'model': 'D8 geometric grid CBS (abstract synchronous ticks)',
            'execution_authorized': False, 'commands_sent': 0,
            'cargo_joint_certified': False, 'live_odom_certified': False,
            'wave2_uses_predicted_wave1_endpoints': True,
            'hybrid_v2_certified': False, 'd92_certified': False,
            'release_authorized': False, 'mission_ledger_updated': False,
            'remaining': ['Restore/recreate three grasped loads in fresh Gazebo',
                          'Verify persistent detachable joints and mission ledger',
                          'Replan each wave from fresh odom with actual post-wave positions',
                          'Adapt Hybrid V2 / D9.2 independently for active robots',
                          'Validate stationary robots, clearance, timing, and controlled release']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('snapshot', 'waves', 'geometry'):
        p.add_argument('--' + name, required=True, type=Path)
    p.add_argument('--output', type=Path)
    a = p.parse_args()
    try:
        from swarm_behavior.swarm_planning import cbs
    except ImportError:
        p.error('Cannot import swarm_behavior.swarm_planning.cbs; source install/setup.bash and set PYTHONPATH=$PWD/src/swarm_behavior:$PYTHONPATH')
    inputs = [json.loads(getattr(a, k).read_text()) for k in ('snapshot','waves','geometry')]
    result = build_preview(*inputs, cbs)
    output = a.output or a.geometry.parent/'delivery1_wave_cbs_preview.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps({'status': result['status'], 'issues': result['issues'], 'output': str(output),
                      'wave_summaries': [{'number': w['number'],'status':w['status']} for w in result['waves']]}, indent=2))
    return 0 if not result['issues'] else 2


if __name__ == '__main__':
    sys.exit(main())
