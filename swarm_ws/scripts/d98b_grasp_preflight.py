#!/usr/bin/env python3
"""D9.8B: read-only live preflight of grasp candidates.

No ROS publishers; no gripper actions; no mission ledger changes.
Checks physical poses near each Summit gripper tip against D8 object set.
"""
import argparse
import itertools
import json
import math
import re
import subprocess
import time
from pathlib import Path

ROBOTS = ('summit0','summit1','summit2')


def parse_gz_pose_info(text, prefix='trash_'):
    """Same pose/info parsing convention as existing grasp_manager.py."""
    objects={}
    lines=text.splitlines(); name=None
    for i,raw in enumerate(lines):
        m=re.match(r'\s*name:\s*"([^"]+)"',raw)
        if m:
            name=m.group(1); continue
        if name and name.startswith(prefix) and raw.strip().startswith('position'):
            x=y=None
            for line in lines[i+1:i+6]:
                mx=re.match(r'\s*x:\s*([-+\d.eE]+)',line)
                my=re.match(r'\s*y:\s*([-+\d.eE]+)',line)
                if mx:x=float(mx.group(1))
                if my:y=float(my.group(1))
                if x is not None and y is not None:break
            if x is not None and y is not None:objects[name]=[x,y]
            name=None
    return objects


def evaluate(names, poses, objects, *, forward=.45, radius=.45, margin=.07, max_speed=.03):
    reasons=[]
    if not isinstance(names,list) or len(names)!=3 or len(set(names))!=3:
        return {'status':'BLOCKED','reasons':['INVALID_STAGE_OBJECTS']}
    if set(poses)!=set(ROBOTS):reasons.append('MISSING_ODOMETRY')
    for name in names:
        if name not in objects:reasons.append('OBJECT_NOT_IN_GAZEBO:'+name)
    if reasons:return {'status':'BLOCKED','reasons':reasons}
    tips={}
    for robot in ROBOTS:
        x,y,yaw,speed=poses[robot]
        if not all(math.isfinite(v) for v in (x,y,yaw,speed)):reasons.append('INVALID_ODOMETRY:'+robot)
        if speed>max_speed:reasons.append('ROBOT_NOT_STOPPED:'+robot)
        tips[robot]=(x+forward*math.cos(yaw),y+forward*math.sin(yaw))
    if reasons:return {'status':'BLOCKED','reasons':reasons}
    distances={robot:{name:math.dist(tips[robot],objects[name]) for name in objects} for robot in ROBOTS}
    # Must match grasp_manager's actual selection: nearest trash object, not nearest assigned object.
    nearest={robot:min(distances[robot],key=distances[robot].get) for robot in ROBOTS}
    proposals=[]
    for perm in itertools.permutations(names):
        if all(nearest[robot]==name for robot,name in zip(ROBOTS,perm)):
            proposals.append({robot:name for robot,name in zip(ROBOTS,perm)})
    if len(proposals)!=1:reasons.append('NEAREST_OBJECT_NOT_A_UNIQUE_D8_ASSIGNMENT')
    assignment=proposals[0] if len(proposals)==1 else {}
    for robot,name in assignment.items():
        d=distances[robot][name]
        ordered=sorted(distances[robot].items(),key=lambda z:z[1])
        if d>radius:reasons.append(robot+':OUTSIDE_GRASP_RADIUS')
        if len(ordered)>1 and ordered[1][1]-ordered[0][1]<margin:
            reasons.append(robot+':AMBIGUOUS_CLOSE_OBJECT')
    return {'status':'READY_FOR_MANUAL_GRASP_TEST_ONLY' if not reasons else 'BLOCKED',
            'reasons':reasons,'objects_planned':names,'nearest_candidates':nearest,
            'suggested_robot_object_mapping':assignment,
            'tip_xy_world':{r:list(t) for r,t in tips.items()},
            'closest_distances_m':{r:round(distances[r][nearest[r]],5) for r in ROBOTS},
            'grasp_radius_m':radius,'ambiguity_margin_m':margin,
            'commands_sent':0,'mission_ledger_updated':False,
            'note':'Preflight only; Int8 grasp result has no object ID. ACK and joint-state verification required before committing a pickup.'}


def read_ros_odom(timeout=8.):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init()
    node=rclpy.create_node('d98b_readonly_grasp_inspection')
    poses={}; rec={}
    def callback(robot):
        def cb(m):
            q=m.pose.pose.orientation;p=m.pose.pose.position;v=m.twist.twist.linear
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            poses[robot]=[p.x,p.y,yaw,math.hypot(v.x,v.y)]
            rec[robot]=time.monotonic()
        return cb
    # Sensor-data QoS subscribes to both common sensor and reliable odom pub profiles.
    sub=[node.create_subscription(Odometry,f'/{r}/odom',callback(r),qos_profile_sensor_data) for r in ROBOTS]
    try:
        t0=time.monotonic()
        while time.monotonic()-t0<timeout:
            rclpy.spin_once(node,timeout_sec=.1)
            if len(poses)==3 and all(time.monotonic()-rec[r]<.5 for r in ROBOTS):break
        return {r:v for r,v in poses.items() if time.monotonic()-rec[r]<.5}
    finally:
        node.destroy_node();rclpy.shutdown()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--handoff',type=Path,required=True)
    p.add_argument('--world',required=True)
    p.add_argument('--output',type=Path)
    p.add_argument('--palm-forward',type=float,default=.45)
    p.add_argument('--grasp-radius',type=float,default=.45)
    p.add_argument('--ambiguity-margin',type=float,default=.07)
    args=p.parse_args()
    handoff=json.loads(args.handoff.read_text())
    if handoff.get('status')!='READY_FOR_PHYSICAL_CONFIRMATION_ONLY' or handoff.get('stage')!='pickup1':
        raise SystemExit('BLOCKED: need READY_FOR_PHYSICAL_CONFIRMATION_ONLY pickup1 handoff')
    gz=subprocess.run(['gz','topic','-e','-t',f'/world/{args.world}/pose/info','-n','1'],capture_output=True,text=True,timeout=10,check=True)
    objects=parse_gz_pose_info(gz.stdout)
    poses=read_ros_odom()
    report=evaluate(handoff['stage_object_names_from_D8_plan'],poses,objects,
                    forward=args.palm_forward,radius=args.grasp_radius,margin=args.ambiguity_margin)
    report['schema']='swarm.d98b.live_grasp_preflight.v1';report['world']=args.world
    out=args.output or args.handoff.parent.parent/'d98b'/'pickup1_grasp_preflight.json'
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(report,indent=2,ensure_ascii=False))
    return 0 if report['status']=='READY_FOR_MANUAL_GRASP_TEST_ONLY' else 2

if __name__=='__main__':raise SystemExit(main())
