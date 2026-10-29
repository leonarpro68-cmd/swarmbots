#!/usr/bin/env python3
"""D9.6C.2F: passive comparison of Gazebo world model poses to ROS odometry.
Never advertises publishers or changes Gazebo/TF. Static observations are NOT calibration proof.
"""
import argparse
import json
import math
import re
import subprocess
import threading
import time
from pathlib import Path

ROBOTS = tuple(f'summit{i}' for i in range(3))


def blocks(text, keyword):
    """Extract nested protobuf text-format blocks with a given field name."""
    pattern = re.compile(r'(?<![\w])' + re.escape(keyword) + r'\s*\{')
    out = []
    for match in pattern.finditer(text):
        start = match.end()
        depth = 1
        quoted = False
        escaped = False
        for pos in range(start, len(text)):
            ch = text[pos]
            if quoted:
                if escaped: escaped = False
                elif ch == '\\': escaped = True
                elif ch == '"': quoted = False
            elif ch == '"': quoted = True
            elif ch == '{': depth += 1
            elif ch == '}':
                depth -= 1
                if depth == 0:
                    out.append(text[start:pos])
                    break
    return out


def scalar(block, field, default=None):
    match = re.search(r'(?m)^\s*' + re.escape(field) + r'\s*:\s*([-+\d.eE]+)\s*$', block)
    return float(match.group(1)) if match else default


def xyzq(pose):
    pb = blocks(pose, 'position')
    qb = blocks(pose, 'orientation')
    if not pb or not qb: return None
    p = [scalar(pb[0], a) for a in ('x','y','z')]
    q = [scalar(qb[0], a, 0.0 if a != 'w' else 1.0) for a in ('x','y','z','w')]
    if None in p or not all(math.isfinite(x) for x in p+q): return None
    if not 0.9 < sum(x*x for x in q) < 1.1: return None
    return {'xyz':p,'quaternion_xyzw':q,'yaw_rad':yaw(q)}


def yaw(q):
    x,y,z,w = q
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))


def angle_diff(a,b):
    return math.atan2(math.sin(a-b),math.cos(a-b))


def parse_pose_messages(text):
    """Returns world model samples; poses with names summitN only (not links)."""
    samples = {name:[] for name in ROBOTS}
    for pose in blocks(text, 'pose'):
        name = re.search(r'(?m)^\s*name\s*:\s*"(summit[012])"\s*$', pose)
        if not name: continue
        data = xyzq(pose)
        if data: samples[name.group(1)].append(data)
    return samples


def record_odom(msg):
    p=msg.pose.pose.position; q=msg.pose.pose.orientation
    Q=[q.x,q.y,q.z,q.w]
    return {'xyz':[p.x,p.y,p.z], 'quaternion_xyzw':Q, 'yaw_rad':yaw(Q),
            'stamp_s':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
            'frame_id':msg.header.frame_id,'child_frame_id':msg.child_frame_id,
            'recv_wall_s':time.monotonic()}


def capture(seconds, topic):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    rclpy.init(args=None)
    class Observer(Node):
        def __init__(self):
            super().__init__('d96c2f_passive_frame_comparison')
            self.odom={r:[] for r in ROBOTS}; self.clock=[]
            self.subs = [self.create_subscription(Odometry,f'/{r}/odom',lambda m,r=r:self.on_odom(r,m),qos_profile_sensor_data) for r in ROBOTS]
            self.subs.append(self.create_subscription(Clock,'/clock',self.on_clock,QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)))
        def on_odom(self,r,m):
            v=self.odom[r]
            if len(v)<500: v.append(record_odom(m))
        def on_clock(self,m):
            self.clock.append(m.clock.sec+m.clock.nanosec*1e-9)
    obs=Observer()
    gz_output=[]
    proc=None
    try:
        # Discover before starting capture to avoid silent 'gz topic -e' indefinite wait.
        listed=subprocess.run(['gz','topic','-l'],capture_output=True,text=True,timeout=8)
        if listed.returncode or topic not in listed.stdout.splitlines():
            raise RuntimeError('Gazebo pose topic not found: '+topic)
        proc=subprocess.Popen(['gz','topic','-e','-t',topic],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,start_new_session=True)
        def readout():
            try:
                for line in proc.stdout:
                    gz_output.append(line)
                    if sum(map(len,gz_output))>5_000_000: break
            except Exception: pass
        thread=threading.Thread(target=readout,daemon=True)
        thread.start()
        deadline=time.monotonic()+seconds
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(obs,timeout_sec=0.05)
    finally:
        if proc is not None:
            proc.terminate()
            try:proc.wait(timeout=2)
            except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=2)
        if proc is not None: thread.join(timeout=0.5)
        raw=''.join(gz_output)
        odom=obs.odom
        clock=obs.clock
        obs.destroy_node();rclpy.shutdown()
    return raw,odom,clock


def analyze(raw,odom,clock, position_tol=.05, yaw_tol_deg=3):
    world=parse_pose_messages(raw)
    details={}
    ok=True
    for robot in ROBOTS:
        gz_samples=world[robot]; ros_samples=odom[robot]
        result={'gazebo_samples':len(gz_samples),'ros_samples':len(ros_samples)}
        if not gz_samples or not ros_samples:
            result['status']='MISSING_WORLD_OR_ODOM'; ok=False
        else:
            # Select final world sample and final odometry sample. Require stationary data,
            # because there is no cross-transport timestamp matching in this passive probe.
            g=gz_samples[-1]; o=ros_samples[-1]
            positions=[math.dist(p['xyz'][:2],g['xyz'][:2]) for p in gz_samples]
            odom_drift=math.dist(ros_samples[0]['xyz'][:2],o['xyz'][:2])
            gz_drift=math.dist(gz_samples[0]['xyz'][:2],g['xyz'][:2])
            yaw_delta=abs(math.degrees(angle_diff(g['yaw_rad'],o['yaw_rad'])))
            pos_delta=math.dist(g['xyz'][:2],o['xyz'][:2])
            stationary=odom_drift <= .01 and gz_drift <= .01
            labels=o['frame_id']==f'{robot}/odom' and o['child_frame_id']==f'{robot}/base_footprint'
            passed=stationary and labels and pos_delta<=position_tol and yaw_delta<=yaw_tol_deg
            result.update({'status':'STATIC_CONSISTENT' if passed else 'STATIC_MISMATCH_OR_MOTION',
                'gazebo_world_xyz_m':g['xyz'], 'ros_odom_xyz_m':o['xyz'],
                'position_delta_xy_m':pos_delta,'yaw_delta_deg':yaw_delta,
                'gazebo_yaw_rad':g['yaw_rad'],'odom_yaw_rad':o['yaw_rad'],
                'gazebo_drift_xy_m':gz_drift,'ros_drift_xy_m':odom_drift,
                'labels_expected':labels,'stationary':stationary,'passed':passed})
            ok = ok and passed
        details[robot]=result
    clock_ok=len(clock)>=2 and clock[-1]>clock[0]
    status='STATIC_FRAME_CONSISTENCY_OBSERVED_NOT_PROVEN' if ok and clock_ok else 'REJECT_FRAME_COMPARISON'
    return {'schema':'swarm.d96c2f.static_compare.v1','status':status,'robots':details,
            'clock_messages':len(clock),'clock_progressing':clock_ok,
            'world_pose_samples_total':sum(len(v) for v in world.values()),
            'coordinate_frame_equivalence_proven':False,'execution_authorized':False,
            'publishers_created':0,'commands_sent':0,
            'note':'Gazebo model pose may differ from ROS base_footprint pose. Samples are NOT time synchronized. Static agreement does not certify frames under motion or authorize cmd_vel.'}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--pose-topic',default='/world/world_demo/pose/info')
    p.add_argument('--observe-seconds',type=float,default=5)
    a=p.parse_args()
    if not 2<=a.observe_seconds<=15:p.error('observe-seconds must be between 2 and 15')
    try:
        raw,odom,clock=capture(a.observe_seconds,a.pose_topic)
        result=analyze(raw,odom,clock)
    except Exception as e:
        result={'schema':'swarm.d96c2f.static_compare.v1','status':'REJECT_CAPTURE_ERROR','error':repr(e),
                'coordinate_frame_equivalence_proven':False,'execution_authorized':False,'publishers_created':0,'commands_sent':0}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    if result['status'].startswith('REJECT'):raise SystemExit(2)

if __name__=='__main__':main()
