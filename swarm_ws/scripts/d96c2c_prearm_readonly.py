#!/usr/bin/env python3
"""D9.6C.2C: passive prearm freshness / coordinated-clock gate.

No publishers, no /cmd_vel, no TF injection, no execution authorization.
A successful result only means SHADOW_PREARM_PASSED (never executable).
"""
import argparse
import hashlib
import json
import math
import time
from pathlib import Path

ROBOTS = ("summit0", "summit1", "summit2")

def position_distance(a,b):
    return math.hypot(float(a[0])-float(b[0]),float(a[1])-float(b[1]))


def hash_file(path):
    with open(path,'rb') as stream:
        h=hashlib.sha256()
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
        return h.hexdigest()


def validate_static(run,stage):
    report_path=run/'d96c2b_report.json'
    report=json.loads(report_path.read_text())
    if report.get('status')!='PASS_READ_ONLY_LIVE_REPLAN' or report.get('stage')!=stage:
        raise ValueError('D9.6C.2B must have passed for the same stage')
    if report.get('execution_authorized') is not False or report.get('commands_sent')!=0:
        raise ValueError('Unexpected executable/source report')
    scenario=Path(report['scenario_path'])
    if not scenario.is_file() or hash_file(scenario)!=report['scenario_sha256']:
        raise ValueError('Scenario checksum mismatch')
    raw=run/'live'/f'{stage}_live_controls.npz'
    converted=run/'d94_conversion'/f'{stage}_converted_NON_EXECUTABLE.npz'
    if hash_file(raw)!=report['source_controls_sha256'] or hash_file(converted)!=report['converted_controls_sha256']:
        raise ValueError('Controls checksum mismatch')
    from d96a_readonly_follower import load_stage, sample
    manifest, trajs=load_stage(run,stage)
    if manifest['status']!='PASS_READ_ONLY':
        raise ValueError('Manifest not preflighted')
    starts=sample(trajs,0.)
    saved=report.get('odometry_snapshot')
    if not isinstance(saved,list) or len(saved)!=3:
        raise ValueError('Missing original odometry snapshot')
    for i,name in enumerate(ROBOTS):
        if position_distance((starts[name]['x_m'],starts[name]['y_m']),saved[i]) > 1e-5:
            raise ValueError(f'{name}: converted curve origin differs from live planning capture')
    return report,manifest,trajs


def decide(report,starts,live,clock,position_tol=.05,max_capture_age_s=120.,max_speed_m_s=.03):
    if not (0<position_tol<=.25 and 0<max_capture_age_s<=3600 and 0<=max_speed_m_s<=.3):
        raise ValueError('Invalid tolerances')
    if set(live)!=set(ROBOTS):
        return 'REJECT_MISSING_ODOMETRY',{}
    details={}; ok=True
    for i,name in enumerate(ROBOTS):
        d=live[name]
        pos=position_distance((starts[name]['x_m'],starts[name]['y_m']),(d['x_m'],d['y_m']))
        speed=math.hypot(d['vx_m_s'],d['vy_m_s'])
        labels=(d['frame_id']==f'{name}/odom' and d['child_frame_id']==f'{name}/base_footprint')
        good=(labels and math.isfinite(pos) and math.isfinite(speed) and pos<=position_tol and
              speed<=max_speed_m_s and 0<=d['age_wall_s']<=.5 and abs(d['quaternion_norm']-1.)<=.02)
        details[name]={'start_error_m':pos,'speed_m_s':speed,'odom_fresh':0<=d['age_wall_s']<=.5,
                       'frame_labels_expected':labels,'quaternion_valid':abs(d['quaternion_norm']-1.)<=.02,'passed':bool(good)}
        ok &= good
    snapshot=report.get('snapshot_time_s')
    now=clock.get('current_sim_s')
    progression=bool(clock.get('progressing',False))
    age=None
    if isinstance(snapshot,(float,int)) and isinstance(now,(float,int)):
        age=float(now)-float(snapshot)
    age_ok=age is not None and math.isfinite(age) and 0<=age<=max_capture_age_s
    details['clock']={'progressing':progression,'capture_age_sim_s':age,'capture_recent':age_ok}
    if not progression:return 'REJECT_CLOCK_NOT_PROGRESSING',details
    if not age_ok:return 'REJECT_STALE_OR_RESET_CAPTURE',details
    if not ok:return 'REJECT_START_OR_ODOMETRY',details
    return 'PASS_SHADOW_PREARM_ONLY',details


def capture_ros(wait_s):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    if not 2<=wait_s<=30:raise ValueError('Wait time outside [2,30] seconds')
    rclpy.init(args=None)
    class Observer(Node):
        def __init__(self):
            super().__init__('d96c2c_shadow_prearm_no_publishers')
            self.odoms={};self.clocks=[]
            self.subs=[self.create_subscription(Odometry,f'/{n}/odom',lambda m,n=n:self.odom(n,m),qos_profile_sensor_data) for n in ROBOTS]
            qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
            self.subs.append(self.create_subscription(Clock,'/clock',self.clock,qos))
            self.get_logger().info('D9.6C.2C: READ ONLY. No velocity or control publishers.')
        def odom(self,n,msg):
            q=msg.pose.pose.orientation
            norm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
            # Odometry twist is in the child coordinate frame; magnitude is invariant under rotation.
            self.odoms[n]={'x_m':float(msg.pose.pose.position.x),'y_m':float(msg.pose.pose.position.y),
                          'vx_m_s':float(msg.twist.twist.linear.x),'vy_m_s':float(msg.twist.twist.linear.y),
                          'frame_id':msg.header.frame_id,'child_frame_id':msg.child_frame_id,
                          'quaternion_norm':norm,'received':time.monotonic()}
        def clock(self,msg):
            value=msg.clock.sec+msg.clock.nanosec*1e-9
            self.clocks.append((time.monotonic(),value))
            if len(self.clocks)>200:self.clocks=self.clocks[-200:]
    node=Observer()
    try:
        deadline=time.monotonic()+wait_s
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if len(node.odoms)==3 and len(node.clocks)>5 and node.clocks[-1][1]-node.clocks[0][1]>=.2:
                break
        now=time.monotonic()
        return ({n:{**d,'age_wall_s':now-d['received']} for n,d in node.odoms.items()},
                {'current_sim_s':node.clocks[-1][1] if node.clocks else None,
                 'progressing':len(node.clocks)>1 and node.clocks[-1][1]-node.clocks[0][1]>=.2,
                 'messages':len(node.clocks)})
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run',required=True,type=Path)
    ap.add_argument('--stage',default='pickup1',choices=('pickup1','delivery1','pickup2','delivery2'))
    ap.add_argument('--position-tolerance',type=float,default=.05)
    ap.add_argument('--max-capture-age-s',type=float,default=120.)
    ap.add_argument('--max-stationary-speed',type=float,default=.03)
    ap.add_argument('--observe-seconds',type=float,default=8.)
    a=ap.parse_args(argv)
    out=a.run/'d96c2c_prearm'/f'{a.stage}_prearm_report.json'
    result={'stage':a.stage,'schema':'swarm.d96c2c.shadow_prearm.v1',
            'status':'REJECTED','execution_authorized':False,'ros_executable':False,
            'publishers_created':0,'commands_sent':0,'coordinate_frame_equivalence_proven':False}
    try:
        report,manifest,trajs=validate_static(a.run,a.stage)
        live,clock=capture_ros(a.observe_seconds)
        from d96a_readonly_follower import sample
        start=sample(trajs,0.)
        status,details=decide(report,start,live,clock,a.position_tolerance,a.max_capture_age_s,a.max_stationary_speed)
        result.update(status=status,details=details,clock=clock,
                      planning_capture_sim_s=report['snapshot_time_s'],
                      planned_horizon_s=max(sum(v[1]) for v in trajs.values()))
    except Exception as e:
        result['status']='REJECTED';result['reason']=str(e)
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(result,indent=2,ensure_ascii=False))
    print('Report:',out)
    return 0 if result['status']=='PASS_SHADOW_PREARM_ONLY' else 2

if __name__=='__main__':raise SystemExit(main())
