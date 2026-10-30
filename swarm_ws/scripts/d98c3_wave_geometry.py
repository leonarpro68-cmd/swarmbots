#!/usr/bin/env python3
"""D9.8C.3: READ-ONLY geometric screen for delivery waves; NOT a CBS planner."""
import argparse, json, math
from pathlib import Path

ROBS=('summit0','summit1','summit2')
def dist(a,b):return math.hypot(a[0]-b[0],a[1]-b[1])
def point_segment(p,a,b):
    dx=b[0]-a[0];dy=b[1]-a[1];den=dx*dx+dy*dy
    u=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/den)) if den else 0.
    return dist(p,(a[0]+u*dx,a[1]+u*dy))
def segments(a,b,c,d):
    def cross(p,q,r):return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
    x,y,z,w=cross(a,b,c),cross(a,b,d),cross(c,d,a),cross(c,d,b)
    if x*y<0 and z*w<0:return 0.
    return min(point_segment(a,c,d),point_segment(b,c,d),point_segment(c,a,b),point_segment(d,a,b))
def validate(snapshot,waves,scenario,clearance=1.25,approach=.80):
    if snapshot.get('status')!='LOAD_GEOMETRY_COMPATIBLE_NOT_JOINT_CERTIFIED' or snapshot.get('issues'):
        raise ValueError('SNAPSHOT_NOT_READY')
    if waves.get('status')!='PROPOSED_WAVES_ONLY_BLOCKED_FOR_EXECUTION':raise ValueError('INVALID_WAVE_PROPOSAL')
    robots=snapshot['robots'];
    if set(robots)!=set(ROBS):raise ValueError('ROBOT_SET_MISMATCH')
    elements=scenario.get('elements',[])
    deposits={e['name']:(e['xy_m'],e) for e in elements if e.get('type')=='deposit'}
    colors={}
    for name,(_,e) in deposits.items():
        col=str(e.get('color','')).lower()
        if not col:
            col='verde' if 'verde' in name.lower() else 'azul' if 'azul' in name.lower() else ''
        if col:colors.setdefault(col,[]).append(name)
    starts={r:robots[r]['odom_xy_m'] for r in ROBS}
    results=[]; all_robots=[]; issues=[]
    for w in waves['waves']:
        wave=w['number']; entries=w['robots']; active=[e['robot'] for e in entries]
        if len(active)!=len(set(active)) or any(r not in ROBS for r in active):raise ValueError('BAD_WAVE_ROBOTS')
        if len(set(e['deposit_color'] for e in entries))!=len(entries):raise ValueError('SAME_DEPOSIT_IN_WAVE')
        paths={}; rows=[]
        for e in entries:
            r=e['robot']; color=e['deposit_color']
            if e['object']!=robots[r]['object']:raise ValueError('CARGO_ID_MISMATCH')
            matches=colors.get(color,[])
            if len(matches)!=1:
                issues.append(f'{r}:DEPOSIT_NOT_UNIQUE_{color}');continue
            dest=deposits[matches[0]][0]
            d=dist(starts[r],dest)
            if d<=approach:
                issues.append(f'{r}:ALREADY_WITHIN_APPROACH_OR_TOO_CLOSE');continue
            goal=[dest[0]+(starts[r][0]-dest[0])*approach/d,dest[1]+(starts[r][1]-dest[1])*approach/d]
            paths[r]=(starts[r],goal)
            rows.append(dict(robot=r,object=e['object'],deposit_name=matches[0],start_xy_m=starts[r],deposit_xy_m=dest,standoff_goal_xy_m=goal,straight_line_length_m=round(dist(starts[r],goal),4)))
        for r,(a,b) in paths.items():
            for s in ROBS:
                if s!=r and s not in active and point_segment(starts[s],a,b)<clearance:
                    issues.append(f'WAVE_{wave}:{r}:STATIONARY_{s}_STRAIGHT_SEGMENT_TOO_CLOSE')
            for s,(c,d) in paths.items():
                if r<s and segments(a,b,c,d)<clearance:
                    issues.append(f'WAVE_{wave}:{r}_{s}:STRAIGHT_SEGMENTS_TOO_CLOSE')
        results.append(dict(number=wave,active_robots=active,stationary_robots=[r for r in ROBS if r not in active],nominal_straight_segments=rows))
        all_robots+=active
    if sorted(all_robots)!=sorted(ROBS):issues.append('EACH_ROBOT_MUST_APPEAR_ONCE')
    return dict(schema='swarm.d98c3.geometry_screen.v1',status='BLOCKED_FOR_EXECUTION_GEOMETRY_PREVIEW',issues=issues,waves=results,
        cargo_joint_certified=bool(snapshot.get('gazebo_joint_attachment_verified',False)),
        geometry_clearance_threshold_m=clearance,deposit_standoff_m=approach,
        straight_segment_screen_passed=not issues,cbs_routes_computed=False,hybrid_v2_certified=False,d92_certified=False,
        current_ros_odometry_verified=False,mission_ledger_updated=False,release_authorized=False,
        commands_sent=0,publishers_created=0,
        note='Nominal straight lines are NOT collision-free CBS routes. Source snapshot may be stale. No physical joint verification. Requires D8 ledger, fresh odom, active/stationary CBS and D9.2 before execution.')
def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--snapshot',required=True,type=Path);p.add_argument('--waves',required=True,type=Path)
    p.add_argument('--scenario',required=True,type=Path);p.add_argument('--output',type=Path)
    a=p.parse_args(); out=a.output or a.waves.parent/'delivery1_wave_geometry.json'
    report=validate(*[json.loads(path.read_text()) for path in (a.snapshot,a.waves,a.scenario)])
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(report,indent=2,ensure_ascii=False))
if __name__=='__main__':main()
