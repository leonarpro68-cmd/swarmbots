#!/usr/bin/env python3
"""D9.9.7A: diagnostic-only alignment gate for archived D9.9 trajectories.

NEVER creates a publisher or issues commands. Positional alignment alone does
not certify orientation, collision-free approach, TF, or safe execution.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time
import numpy as np

ROBOTS = tuple(f'summit{i}' for i in range(3))


def sha256(path):
    h=hashlib.sha256()
    with open(path,'rb') as fh:
        for block in iter(lambda:fh.read(1024*1024),b''):
            h.update(block)
    return h.hexdigest()


def read_reference(npz):
    with np.load(npz,allow_pickle=False) as data:
        starts={}; ends={}; durations={}
        for i,name in enumerate(ROBOTS):
            c=data[f'R{i}_controls']; dur=data[f'R{i}_durations_s']
            if c.ndim!=3 or c.shape[1:]!=(6,2) or len(c)!=len(dur) or not len(c):
                raise ValueError(f'Invalid trajectory shape: {name}')
            if not np.isfinite(c).all() or not np.isfinite(dur).all() or np.any(dur<=0):
                raise ValueError(f'Nonfinite trajectory or nonpositive duration: {name}')
            starts[name]=c[0,0,:].tolist()
            ends[name]=c[-1,-1,:].tolist()
            durations[name]=float(dur.sum())
    return starts,ends,durations


def evaluate(reference,observed,speeds,stamps,frames,clock,tolerance,stationary,stamp_tolerance):
    problems=[];robots={}
    if len(clock)<2 or clock[-1]-clock[0]<=.05:problems.append('CLOCK_NOT_ADVANCING')
    if any(b<a-1e-6 for a,b in zip(clock,clock[1:])):problems.append('CLOCK_RESET')
    for name in ROBOTS:
        if name not in observed:
            problems.append('MISSING_ODOMETRY_'+name);continue
        pos=observed[name];target=reference[name]; delta=[target[0]-pos[0],target[1]-pos[1]]
        error=math.hypot(*delta);v=speeds[name];stamp_age=abs(clock[-1]-stamps[name]) if clock else float('inf')
        frame,child=frames[name]
        robots[name]={'observed_xy_m':pos,'reference_xy_m':target,'delta_xy_m':delta,
                      'distance_m':error,'speed_m_s':v,'stamp_delta_s':stamp_age,
                      'frame':frame,'child':child,'position_within_tolerance':error<=tolerance}
        if error>tolerance:problems.append('ALIGNMENT_REQUIRED_'+name)
        if v>stationary:problems.append('NOT_STATIONARY_'+name)
        if stamp_age>stamp_tolerance:problems.append('STALE_ODOMETRY_'+name)
        if frame!=f'{name}/odom' or child!=f'{name}/base_footprint':
            problems.append('FRAME_NOT_EXPECTED_'+name)
    return sorted(set(problems)),robots


def observe_ros(seconds):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    rclpy.init()
    node=Node('d997a_alignment_diagnostic_readonly')
    qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,
                   durability=DurabilityPolicy.VOLATILE)
    clock=[];odom={};subscriptions=[]
    subscriptions.append(node.create_subscription(Clock,'/clock',lambda msg:clock.append(msg.clock.sec+msg.clock.nanosec*1e-9),qos))
    for name in ROBOTS:
        subscriptions.append(node.create_subscription(Odometry,f'/{name}/odom',lambda msg,n=name:odom.__setitem__(n,msg),qos))
    try:
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=min(.15,max(0.,until-time.monotonic())))
    finally:
        node.destroy_node();rclpy.shutdown()
    pos={};speeds={};stamps={};frames={}
    for name,msg in odom.items():
        p=msg.pose.pose.position;v=msg.twist.twist.linear
        pos[name]=[float(p.x),float(p.y)]
        speeds[name]=math.hypot(v.x,v.y)
        stamps[name]=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        frames[name]=[msg.header.frame_id,msg.child_frame_id]
    return pos,speeds,stamps,frames,clock


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--trajectory',type=Path,required=True,help='Historical converted D9.9.4 NPZ')
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--live',action='store_true',help='Observe ROS /odom and /clock, read-only')
    ap.add_argument('--sample-json',type=Path,help='Offline fixture: xy, speeds, stamps, frames, clock')
    ap.add_argument('--observe-seconds',type=float,default=8)
    ap.add_argument('--position-tolerance',type=float,default=.05)
    ap.add_argument('--max-stationary-speed',type=float,default=.03)
    ap.add_argument('--max-stamp-delta',type=float,default=.25)
    args=ap.parse_args()
    if args.live==bool(args.sample_json):ap.error('Specify exactly one of --live or --sample-json')
    if min(args.observe_seconds,args.position_tolerance,args.max_stationary_speed,args.max_stamp_delta)<=0:
        ap.error('All tolerances/timeouts must be positive')
    starts,ends,durs=read_reference(args.trajectory)
    if args.live:
        try: observation=observe_ros(args.observe_seconds)
        except Exception as exc:
            observation=({}, {}, {}, {}, [])
            observer_error=str(exc)
        else:observer_error=None
    else:
        fixture=json.loads(args.sample_json.read_text())
        observation=tuple(fixture[k] for k in ('positions','speeds','stamps','frames','clock'))
        observer_error=None
    problems,robots=evaluate(starts,*observation,args.position_tolerance,args.max_stationary_speed,args.max_stamp_delta)
    if observer_error: problems.append('ROS_OBSERVER_FAILED')
    result={'schema':'swarm.d997a.alignment.diagnostic.readonly.v1',
            'status':('REJECT_OBSERVATION' if any(x not in [f'ALIGNMENT_REQUIRED_{r}' for r in ROBOTS] for x in problems)
                      else 'ALREADY_POSITIONALLY_ALIGNED' if not problems else 'ALIGNMENT_REQUIRED_NOT_EXECUTABLE'),
            'problems':sorted(set(problems)),'trajectory':str(args.trajectory.resolve()),
            'trajectory_sha256':sha256(args.trajectory),'robots':robots,
            'trajectory_end_xy_m':ends,'trajectory_duration_s':durs,
            'orientation_checked':False,'static_obstacle_clearance_checked':False,
            'interrobot_alignment_motion_checked':False,'tf_certified':False,
            'execution_authorized':False,'publishers_created':0,'commands_sent':0}
    if observer_error:result['observer_error']=observer_error
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({'status':result['status'],'robots':{k:{'distance_m':v['distance_m'],'delta_xy_m':v['delta_xy_m']} for k,v in robots.items()},'problems':result['problems'],'report':str(args.output)},indent=2,ensure_ascii=False))
    return 2 if result['status']=='REJECT_OBSERVATION' else 0

if __name__=='__main__':raise SystemExit(main())
