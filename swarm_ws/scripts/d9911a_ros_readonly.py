#!/usr/bin/env python3
"""D9.9.11A: observador ROS de prearmado. No crea publicadores ni mueve robots."""
import argparse
import hashlib
import importlib.util
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.node import Node

ROBOTS = [f'summit{i}' for i in range(3)]

def digest(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def protocol_from_file(path):
    spec=importlib.util.spec_from_file_location('d9911_proto', str(Path(path).resolve()))
    mod=importlib.util.module_from_spec(spec)
    import sys
    sys.modules[spec.name]=mod
    spec.loader.exec_module(mod)
    return mod

class Observer(Node):
    def __init__(self):
        super().__init__('d9911a_readonly_observer')
        self.odom={}
        self.clocks=[]
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        for name in ROBOTS:
            self.create_subscription(Odometry, f'/{name}/odom', lambda m, n=name: self.on_odom(n,m), 10)
    def on_clock(self, msg):
        t=float(msg.clock.sec)+msg.clock.nanosec*1e-9
        self.clocks.append(t)
        if len(self.clocks)>200: self.clocks=self.clocks[-200:]
    def on_odom(self, name, msg):
        p=msg.pose.pose.position
        v=msg.twist.twist.linear
        self.odom[name]={'xy':[float(p.x),float(p.y)],'speed_m_s':math.hypot(v.x,v.y),
                         'frame':msg.header.frame_id, 'child':msg.child_frame_id,
                         'stamp_s':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9}

def main():
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument('--trajectory',required=True)
    a.add_argument('--protocol-file',default='src/swarm_behavior/swarm_behavior/swarm_planning/hybrid_refinement/timed_protocol.py')
    a.add_argument('--expected-sha256',default='3235223232eb1b47e36c46cc107086c933e5020917aa7b78758b9b183985e2dd')
    a.add_argument('--observe-seconds',type=float,default=8.)
    a.add_argument('--position-tolerance',type=float,default=.05)
    a.add_argument('--max-speed',type=float,default=.01)
    a.add_argument('--output',required=True)
    args=a.parse_args()
    r={'schema':'swarm.d9911a.readonly.v1','status':'REJECT','problems':[],'execution_authorized':False,
       'publishers_created':0,'commands_sent':0,'robots':{}}
    node=None
    try:
        path=Path(args.trajectory).resolve()
        h=digest(path); r['trajectory_sha256']=h
        if h!=args.expected_sha256.lower(): raise ValueError('Hash histórico distinto del esperado')
        proto=protocol_from_file(args.protocol_file)
        if not hasattr(proto,'SCHEMA_V2'): raise ValueError('Protocolo V2 no presente en fuente')
        starts={}
        with np.load(path,allow_pickle=False) as n:
            for i,name in enumerate(ROBOTS):
                control=np.asarray(n[f'R{i}_controls'],dtype=float)
                dur=np.asarray(n[f'R{i}_durations_s'],dtype=float)
                if control.ndim!=3 or control.shape[1:]!=(6,2) or len(dur)!=len(control) or not np.isfinite(control).all() or not np.isfinite(dur).all() or np.any(dur<=0):
                    raise ValueError(f'{name}: datos inválidos')
                payload={'schema':proto.SCHEMA_V2,'action':'start','session_id':'READONLY_NOT_FOR_PUBLISHING','robot':name,
                         'frame_id':f'{name}/odom','start_time_s':100.,'control_points_m':control.tolist(),
                         'segment_durations_s':dur.tolist(),'speed_limit_m_s':1.2,'acceleration_limit_m_s2':1.,
                         'max_tracking_error_m':.15,'goal_tolerance_m':.05,'goal_settle_s':.5,'finish_grace_s':4.}
                parsed=proto.decode_command(json.dumps(payload))
                starts[name]=control[0,0].tolist()
                r['robots'][name]={'start_xy':starts[name],'duration_s':parsed.duration_s,'segments':len(control)}
        rclpy.init()
        node=Observer()
        end=time.monotonic()+args.observe_seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.1)
        if len(node.clocks)<2 or node.clocks[-1]<=node.clocks[0]:
            r['problems'].append('CLOCK_MISSING_OR_NOT_ADVANCING')
        for name in ROBOTS:
            pubs=node.get_publishers_info_by_topic(f'/{name}/cmd_vel')
            named=[p for p in pubs if p.node_name=='path_follower' and p.node_namespace.rstrip('/')==f'/{name}']
            if len(pubs)!=1 or len(named)!=1: r['problems'].append(f'{name}: CMD_VEL_PUBLISHERS_{len(pubs)}_FOLLOWERS_{len(named)}')
            o=node.odom.get(name)
            if o is None: r['problems'].append(f'{name}: ODOM_MISSING');continue
            err=math.dist(o['xy'],starts[name]);fresh=node.clocks and 0<=node.clocks[-1]-o['stamp_s']<=.5
            r['robots'][name].update(observed_xy=o['xy'],error_m=err,speed_m_s=o['speed_m_s'],
                                     odom_frame=o['frame'],odom_fresh=bool(fresh))
            if o['frame']!=f'{name}/odom':r['problems'].append(f'{name}: ODOM_FRAME_MISMATCH')
            if err>args.position_tolerance:r['problems'].append(f'{name}: NOT_ALIGNED_{err:.3f}m')
            if o['speed_m_s']>args.max_speed:r['problems'].append(f'{name}: NOT_STATIONARY')
            if not fresh:r['problems'].append(f'{name}: ODOM_STALE')
        r['clock_samples']=len(node.clocks)
        r['status']='PASS_READONLY_READY_NOT_EXECUTABLE' if not r['problems'] else 'REJECT'
    except Exception as exc:
        r['problems'].append(str(exc))
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        p=Path(args.output);p.parent.mkdir(parents=True,exist_ok=True)
        p.write_text(json.dumps(r,indent=2,ensure_ascii=False)+'\n')
        print(json.dumps(r,indent=2,ensure_ascii=False))
    return 0 if not r['problems'] else 2
if __name__=='__main__':raise SystemExit(main())
