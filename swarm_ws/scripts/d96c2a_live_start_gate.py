#!/usr/bin/env python3
"""D9.6C.2A: read-only live-start eligibility gate for archived Bézier plans.

Intentionally NEVER creates ROS publishers or TF; does not approve execution.
"""
import argparse
import json
import math
import time
from pathlib import Path

from d96a_readonly_follower import load_stage, sample

ROBOTS = ('summit0', 'summit1', 'summit2')


def position_distance(a, b):
    return math.hypot(float(a[0])-float(b[0]), float(a[1])-float(b[1]))


def evaluate_starts(scenario, ref_starts, captured, live, pos_tol=.05, plan_tol=1e-5):
    """Compare three distinct origins: nominal scenario, D8 capture, live Gazebo.

    Plan must reproduce captured D8 positions; archived plan is not reusable
    unless current robot position matches each original trajectory origin.
    This DOES NOT prove that odometry frame is globally aligned.
    """
    if set(ref_starts)!=set(ROBOTS) or set(live)!=set(ROBOTS) or set(captured)!=set(ROBOTS):
        raise ValueError('Need exactly three known robots in all observations')
    if not math.isfinite(pos_tol) or pos_tol <= 0 or not math.isfinite(plan_tol) or plan_tol <= 0:
        raise ValueError('Invalid tolerances')
    results={}
    for name in ROBOTS:
        r=ref_starts[name]
        ref=(r['x_m'],r['y_m'])
        now=live[name]
        if not all(math.isfinite(float(v)) for v in (*ref, now['x_m'], now['y_m'], *captured[name], *scenario[name])):
            raise ValueError(f'{name}: nonfinite coordinates')
        frame_good = now['frame_id']==f'{name}/odom' and now['child_frame_id']==f'{name}/base_footprint'
        normal = abs(float(now['quaternion_norm'])-1.) <= 0.02
        age_good = 0 <= float(now['age_wall_s']) <= 0.5
        d_cap=position_distance(ref,captured[name])
        d_live=position_distance(ref,(now['x_m'],now['y_m']))
        d_nom=position_distance(ref,scenario[name])
        results[name]={'plan_vs_captured_m':d_cap,'plan_vs_live_m':d_live,
                       'plan_vs_nominal_m':d_nom,'frame_labels_expected':frame_good,
                       'quaternion_valid':normal,'odom_fresh':age_good,
                       'plan_from_capture':d_cap<=plan_tol,
                       'archived_start_matches_live':d_live<=pos_tol,
                       'current_xy_m':[now['x_m'],now['y_m']],
                       'archived_start_xy_m':list(ref),
                       'yaw_rad':now['yaw_rad']}
    all_matched=all(r['plan_from_capture'] and r['archived_start_matches_live'] and
                    r['frame_labels_expected'] and r['quaternion_valid'] and r['odom_fresh'] for r in results.values())
    return {'status':'ARCHIVED_START_MATCHES_LIVE_READ_ONLY' if all_matched else 'REQUIRES_LIVE_REPLAN',
            'archived_start_reuse_gate':all_matched,'robots':results,
            'coordinate_frame_equivalence_proven':False,'execution_authorized':False,
            'commands_sent':0,'publisher_count':0}


def read_captured(run,stage):
    matches=list((run/'mission'/stage).rglob(f'{stage}_live_report.json'))
    if len(matches)!=1:raise ValueError(f'Expected one {stage}_live_report.json; got {len(matches)}')
    report=json.loads(matches[0].read_text())
    arr=report['stages'][stage]['hybrid_v2']['odom_xy']
    if len(arr)!=3:raise ValueError('Invalid captured odometry')
    return {name: tuple(map(float, arr[i])) for i,name in enumerate(ROBOTS)}


def ros_snapshot(seconds):
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    if not 0<seconds<=60:raise ValueError('seconds must be in (0,60]')
    rclpy.init(args=None)
    class Observer(Node):
        def __init__(self):
            super().__init__('d96c2a_live_start_readonly')
            self.data={}
            self.subs=[self.create_subscription(Odometry, f'/{name}/odom',
                  lambda msg,n=name:self.callback(n,msg),qos_profile_sensor_data) for name in ROBOTS]
            self.get_logger().info('D9.6C.2A READ ONLY; no publishers or robot control')
        def callback(self,name,msg):
            q=msg.pose.pose.orientation
            norm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            self.data[name]={'x_m':float(msg.pose.pose.position.x),
                             'y_m':float(msg.pose.pose.position.y),
                             'frame_id':msg.header.frame_id,
                             'child_frame_id':msg.child_frame_id,
                             'quaternion_norm':norm,'yaw_rad':yaw,
                             'received_wall':time.monotonic()}
    node=Observer()
    try:
        deadline=time.monotonic()+seconds
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if len(node.data)==3:break
        now=time.monotonic()
        return {n:{**x,'age_wall_s':now-x['received_wall']} for n,x in node.data.items()}
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run',required=True,type=Path)
    ap.add_argument('--stage',default='pickup1',choices=['pickup1','delivery1','pickup2','delivery2'])
    ap.add_argument('--odom-timeout',type=float,default=10.)
    ap.add_argument('--start-tolerance',type=float,default=.05)
    ap.add_argument('--output',type=Path)
    a=ap.parse_args()
    manifest, trajectories=load_stage(a.run,a.stage)
    refs=sample(trajectories,0.)
    captured=read_captured(a.run,a.stage)
    data=json.loads((a.run/'scenario.json').read_text())
    scenario={r['name']:r['xy_m'] for r in data['robots']}
    actual=ros_snapshot(a.odom_timeout)
    if set(actual)!=set(ROBOTS):
        result={'status':'REJECT_MISSING_ODOMETRY','received_robots':sorted(actual),
                'execution_authorized':False,'commands_sent':0,'publisher_count':0}
    else:
        result=evaluate_starts(scenario,refs,captured,actual,pos_tol=a.start_tolerance)
    result.update({'stage':a.stage,'plan_source':str(a.run),'manifest_schema':manifest['schema'],
                   'note':'Read-only snapshot. Matching numeric coordinates is NOT a TF calibration or authorization to move. Live replan must regenerate and recertify converted D9.2 trajectories.'})
    path=a.output or a.run/'d96c2a_live_start'/f'{a.stage}_live_gate.json'
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    print('Report:',path)
    if result['status']=='REJECT_MISSING_ODOMETRY':return 2
    return 0

if __name__=='__main__':raise SystemExit(main())
