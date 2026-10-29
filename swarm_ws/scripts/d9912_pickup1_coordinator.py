#!/usr/bin/env python3
"""D9.9.12: guarded pickup1 V2 coordinator; dry-run by default.

Uses existing /summitN/path_follower and its timed_trajectory ROS interface.
Never publishes cmd_vel. Historical SHA256 is required.
"""
import argparse
import hashlib
import importlib.util
import json
import math
import sys
import time
import uuid
from pathlib import Path

import numpy as np

ROBOTS = [f'summit{i}' for i in range(3)]
HISTORICAL_SHA = '3235223232eb1b47e36c46cc107086c933e5020917aa7b78758b9b183985e2dd'


def load_proto(path):
    spec=importlib.util.spec_from_file_location('d9912_timed_protocol',str(path.resolve()))
    m=importlib.util.module_from_spec(spec);sys.modules[spec.name]=m;spec.loader.exec_module(m);return m


def prepare(a, report):
    traj=Path(a.trajectory).resolve()
    digest=hashlib.sha256(traj.read_bytes()).hexdigest()
    report['trajectory_sha256']=digest
    if digest!=a.expected_sha256.lower():raise ValueError('HASH_MISMATCH_CERTIFIED_TRAJECTORY')
    proto=load_proto(Path(a.protocol_file))
    if not hasattr(proto,'SCHEMA_V2'):raise ValueError('V2_PROTOCOL_MISSING_IN_SOURCE')
    data={}
    with np.load(traj,allow_pickle=False) as z:
        for i,name in enumerate(ROBOTS):
            ctrl=np.asarray(z[f'R{i}_controls'],float)
            dur=np.asarray(z[f'R{i}_durations_s'],float)
            if ctrl.ndim!=3 or ctrl.shape[1:]!=(6,2) or dur.shape!=(len(ctrl),):
                raise ValueError(f'{name}: INVALID_SHAPE')
            if not np.isfinite(ctrl).all() or not np.isfinite(dur).all() or (dur<=0).any():
                raise ValueError(f'{name}: INVALID_VALUES')
            data[name]=(ctrl,dur)
            report['robots'][name]={'segments':len(ctrl),'duration_s':float(sum(dur)),'start_xy':ctrl[0,0].tolist()}
    return proto,data


def run_ros(a,proto,data,report):
    import rclpy
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    class Coordinator(Node):
        def __init__(self):
            super().__init__('d9912_pickup1_coordinator')
            self.clock_times=[];self.odoms={};self.statuses={n:[] for n in ROBOTS}
            self.create_subscription(Clock,'/clock',self.clock_cb,10)
            for n in ROBOTS:
                self.create_subscription(Odometry,f'/{n}/odom',lambda m,n=n:self.odom_cb(n,m),10)
                self.create_subscription(String,f'/{n}/hybrid_trajectory/status',lambda m,n=n:self.status_cb(n,m),10)
            self.pubs={}
        def clock_cb(self,msg):
            t=msg.clock.sec+msg.clock.nanosec*1e-9;self.clock_times.append(t)
        def odom_cb(self,n,msg):
            p=msg.pose.pose.position;v=msg.twist.twist.linear
            self.odoms[n]={'xy':(p.x,p.y),'speed':math.hypot(v.x,v.y),'frame':msg.header.frame_id,
                            'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9}
        def status_cb(self,n,msg):
            try:
                robot,session,state=proto.decode_status(msg.data)
                if robot==n and session==report['session_id']:
                    self.statuses[n].append((state,time.monotonic()))
            except Exception:pass
    rclpy.init();node=Coordinator();published=False
    def spin_for(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    def publish_cancel():
        if not published:return
        for n in ROBOTS:
            raw=json.dumps({'schema':proto.SCHEMA_V2,'action':'cancel','session_id':report['session_id'],'robot':n})
            node.pubs[n].publish(String(data=raw))
        spin_for(.5)
    try:
        spin_for(a.observe_seconds)
        if len(node.clock_times)<3 or node.clock_times[-1]-node.clock_times[0]<1.:
            raise RuntimeError('CLOCK_NOT_ADVANCING')
        latest=node.clock_times[-1]
        for n in ROBOTS:
            pubs=node.get_publishers_info_by_topic(f'/{n}/cmd_vel')
            expected=[p for p in pubs if p.node_name=='path_follower' and p.node_namespace.rstrip('/')==f'/{n}']
            if len(pubs)!=1 or len(expected)!=1:raise RuntimeError(f'{n}: CMD_VEL_OWNER_MISMATCH')
            od=node.odoms.get(n)
            if od is None:raise RuntimeError(f'{n}: ODOM_MISSING')
            err=math.dist(od['xy'],data[n][0][0,0])
            report['robots'][n].update(error_m=err,speed_m_s=od['speed'],odom_frame=od['frame'])
            if err>a.start_tolerance or od['speed']>a.max_stationary_speed or od['frame']!=f'{n}/odom':
                raise RuntimeError(f'{n}: ALIGNMENT_OR_STATIONARY_CHECK_FAILED')
            if not 0<=latest-od['stamp']<=.5:raise RuntimeError(f'{n}: ODOM_STALE')
        report['clock_s']=latest
        if not a.execute:
            report['status']='PASS_GUARDED_DRY_RUN_NO_COMMANDS';return
        # Must be rebuilt and the live Gazebo followers restarted before this point.
        # The pre-arm test is intentionally required by explicit operator confirmation.
        if not a.confirm_runtime_v2:
            raise RuntimeError('REQUIRE_EXPLICIT_RUNTIME_V2_CONFIRMATION')
        start_time=latest+a.start_delay
        report['start_time_s']=start_time
        for n in ROBOTS:
            node.pubs[n]=node.create_publisher(String,f'/{n}/hybrid_trajectory/command',10)
        spin_for(.5)
        if any(node.pubs[n].get_subscription_count()!=1 for n in ROBOTS):
            raise RuntimeError('TIMED_COMMAND_SUBSCRIBER_MISSING_OR_DUPLICATED')
        report['commands_sent']=0
        for n in ROBOTS:
            c,d=data[n]
            cmd=dict(schema=proto.SCHEMA_V2,action='start',session_id=report['session_id'],robot=n,
                     frame_id=f'{n}/odom',start_time_s=start_time,control_points_m=c.tolist(),
                     segment_durations_s=d.tolist(),speed_limit_m_s=1.2,acceleration_limit_m_s2=1.,
                     max_tracking_error_m=.15,goal_tolerance_m=.05,goal_settle_s=.5,finish_grace_s=4.)
            raw=json.dumps(cmd,separators=(',',':'),allow_nan=False)
            proto.decode_command(raw)
            node.pubs[n].publish(String(data=raw));report['commands_sent']+=1
        published=True
        # Arm barrier. No robot should start until common start_time.
        deadline=time.monotonic()+min(a.start_delay-1.,15.)
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if any('failed' in [s for s,_ in node.statuses[n]] for n in ROBOTS):
                raise RuntimeError('FOLLOWER_REJECTED_START')
            if all('armed' in [s for s,_ in node.statuses[n]] for n in ROBOTS):break
        else:raise RuntimeError('ARM_BARRIER_TIMEOUT')
        report['status']='ARMED_ALL'
        deadline=time.monotonic()+max(report['robots'][n]['duration_s'] for n in ROBOTS)+a.start_delay+12.
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            states={n:[s for s,_ in node.statuses[n]] for n in ROBOTS}
            if any('failed' in states[n] or 'cancelled' in states[n] for n in ROBOTS):
                raise RuntimeError('FOLLOWER_FAILED_OR_CANCELLED')
            if all('done' in states[n] for n in ROBOTS):
                report['status']='DONE_ALL_PICKUP1';break
        else:raise RuntimeError('FOLLOWER_COMPLETION_TIMEOUT')
    except BaseException:
        publish_cancel()
        raise
    finally:
        report['status_history']={n:[s for s,_ in node.statuses[n]] for n in ROBOTS}
        node.destroy_node();rclpy.shutdown()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trajectory',default='scripts/d996_reference/pickup1_d994_NON_EXECUTABLE.npz')
    p.add_argument('--protocol-file',default='src/swarm_behavior/swarm_behavior/swarm_planning/hybrid_refinement/timed_protocol.py')
    p.add_argument('--expected-sha256',default=HISTORICAL_SHA)
    p.add_argument('--output',default='scripts/resultados/d9912_pickup1.json')
    p.add_argument('--observe-seconds',type=float,default=5.)
    p.add_argument('--start-tolerance',type=float,default=.05)
    p.add_argument('--max-stationary-speed',type=float,default=.01)
    p.add_argument('--start-delay',type=float,default=10.)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--confirm-runtime-v2',action='store_true',help='Explicit assertion: package rebuilt and followers relaunched with V2')
    a=p.parse_args()
    report={'schema':'swarm.d9912.pickup1.coordinator.v1','status':'REJECT','execution_requested':a.execute,
            'commands_sent':0,'problems':[],'robots':{},'session_id':'D9912_'+uuid.uuid4().hex[:12]}
    try:
        if a.start_delay<5 or a.start_delay>30:raise ValueError('START_DELAY_OUT_OF_RANGE')
        proto,data=prepare(a,report)
        run_ros(a,proto,data,report)
    except Exception as exc:report['problems'].append(str(exc));report['status']='REJECT'
    target=Path(a.output);target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(report,indent=2,ensure_ascii=False))
    return 0 if not report['problems'] else 2
if __name__=='__main__':raise SystemExit(main())
