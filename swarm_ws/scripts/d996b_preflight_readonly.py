#!/usr/bin/env python3
"""D9.9.6B: verify a FRESH collective path before any follower exists.

Does not publish anything. ROS only imported if --live. PASS means only a
stationary, fresh-start observation, NOT a movement authorization or TF/map
certification. Works with manifest produced by d996b_plan_from_live_odom.py.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time


def sha256(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def static_check(manifest):
    failures=[]
    if manifest.get('schema')!='swarm.d996b.fresh_start.offline.v1': failures.append('WRONG_SCHEMA')
    if manifest.get('mode')!='live' or manifest.get('status')!='PASS_OFFLINE_FRESH_START_NOT_EXECUTABLE':
        failures.append('NO_FRESH_LIVE_OFFLINE_PLAN')
    if manifest.get('offline_preflight_status')!='PASS_OFFLINE_READ_ONLY': failures.append('OFFLINE_PREFLIGHT_NOT_APPROVED')
    arts=manifest.get('artifacts',{})
    for name,digest in manifest.get('sha256',{}).items():
        f=arts.get(name)
        if not f or not Path(f).is_file(): failures.append('MISSING_'+name.upper())
        elif sha256(f)!=digest: failures.append('FILE_CHANGED_'+name.upper())
    starts=manifest.get('converted_start_xy_m',{})
    if set(starts)!=set(f'summit{i}' for i in range(3)): failures.append('INVALID_ROBOT_STARTS')
    else:
        for name,xy in starts.items():
            if len(xy)!=2 or not all(math.isfinite(float(v)) for v in xy):failures.append('INVALID_START_'+name)
    # The raw preflight itself is integrity-protected by the manifest.
    pf=arts.get('offline_preflight')
    if pf and Path(pf).is_file():
        p=json.loads(Path(pf).read_text(encoding='utf8'))
        if p.get('status')!='PASS_OFFLINE_READ_ONLY' or p.get('failures'):
            failures.append('PREFLIGHT_REPORT_REJECTED')
    return sorted(set(failures))


def evaluate_live(positions, timestamps, speeds, frames, clock_values, reference,
                  max_error, max_speed, max_stale):
    """Pure evaluator of sampled ROS topics; testable without ROS/Gazebo."""
    fails=[];results={}
    if len(clock_values)<2 or clock_values[-1]-clock_values[0]<0.05:
        fails.append('CLOCK_NOT_ADVANCING')
    if any(y<x-1e-6 for x,y in zip(clock_values,clock_values[1:])):
        fails.append('CLOCK_RESET')
    last=clock_values[-1] if clock_values else None
    for i in range(3):
        robot=f'summit{i}'
        if robot not in positions or robot not in reference:
            fails.append('MISSING_ODOM_'+robot);continue
        xy=positions[robot];err=math.dist(xy,reference[robot]);v=speeds[robot]
        age=abs(last-timestamps[robot]) if last is not None else float('inf')
        frame,child=frames[robot]
        results[robot]={'xy':xy,'reference_xy':reference[robot],'error_m':err,
                        'speed_m_s':v,'stamp_delta_s':age,'frame':frame,'child':child}
        if err>max_error:fails.append('START_POSE_MISMATCH_'+robot)
        if v>max_speed:fails.append('ROBOT_NOT_STOPPED_'+robot)
        if age>max_stale:fails.append('ODOM_STALE_'+robot)
        if frame!=f'{robot}/odom' or child!=f'{robot}/base_footprint':fails.append('ODOM_FRAME_MISMATCH_'+robot)
    return sorted(set(fails)),results


def live_observe(reference,seconds,max_error,max_speed,max_stale):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    rclpy.init()
    node=Node('d996b_new_start_readonly')
    qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,
                   durability=DurabilityPolicy.VOLATILE)
    clock=[];odoms={};subscriptions=[]
    def cb_clock(m):clock.append(float(m.clock.sec)+float(m.clock.nanosec)*1e-9)
    subscriptions.append(node.create_subscription(Clock,'/clock',cb_clock,qos))
    for robot in reference:
        subscriptions.append(node.create_subscription(Odometry,f'/{robot}/odom',
                                                  lambda m,n=robot:odoms.__setitem__(n,m),qos))
    try:
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=min(.2,max(0,end-time.monotonic())))
    finally:
        node.destroy_node();rclpy.shutdown()
    pos={};speed={};stamps={};frames={}
    for name,msg in odoms.items():
        p=msg.pose.pose.position;v=msg.twist.twist.linear
        pos[name]=[float(p.x),float(p.y)]
        speed[name]=math.hypot(v.x,v.y)
        stamps[name]=float(msg.header.stamp.sec)+float(msg.header.stamp.nanosec)*1e-9
        frames[name]=(msg.header.frame_id,msg.child_frame_id)
    problems,robots=evaluate_live(pos,stamps,speed,frames,clock,reference,
                                  max_error,max_speed,max_stale)
    return problems,{'robots':robots,'clock_count':len(clock),
                     'clock_start_s':clock[0] if clock else None,
                     'clock_last_s':clock[-1] if clock else None}


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--manifest',type=Path,required=True)
    ap.add_argument('--live',action='store_true')
    ap.add_argument('--observe-seconds',type=float,default=8.)
    ap.add_argument('--max-start-error',type=float,default=.05)
    ap.add_argument('--max-stationary-speed',type=float,default=.03)
    ap.add_argument('--max-stamp-delta',type=float,default=.25)
    ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args()
    if min(a.observe_seconds,a.max_start_error,a.max_stationary_speed,a.max_stamp_delta)<=0:
        ap.error('Parameters must be positive')
    manifest=json.loads(a.manifest.read_text(encoding='utf8'))
    failures=static_check(manifest)
    live={}
    if a.live and not failures:
        try:
            live_errors,live=live_observe(manifest['converted_start_xy_m'],a.observe_seconds,
                                          a.max_start_error,a.max_stationary_speed,a.max_stamp_delta)
            failures+=live_errors
        except Exception as exc:
            failures.append('LIVE_OBSERVER_ERROR')
            live={'error':str(exc)}
    status=('REJECT' if failures else ('PASS_STATIC_AND_LIVE_INERT_NOT_EXECUTABLE'
                                       if a.live else 'PASS_STATIC_ONLY_NOT_EXECUTABLE'))
    output={'schema':'swarm.d996b.start_gate.readonly.v1',
            'status':status,'problems':sorted(set(failures)),
            'fresh_start_manifest':str(a.manifest.resolve()),
            'live_observation':live,'tf_certified':False,
            'gazebo_world_bytes_loaded_certified':False,
            'execution_authorized':False,'publishers_created':0,'commands_sent':0}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(output,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
    print(json.dumps({'status':status,'problems':output['problems'],
                      'report':str(a.output)},indent=2,ensure_ascii=False))
    return 0 if not failures else 2

if __name__=='__main__':raise SystemExit(main())
