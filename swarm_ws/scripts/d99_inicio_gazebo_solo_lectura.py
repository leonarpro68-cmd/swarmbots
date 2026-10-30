#!/usr/bin/env python3
"""Comparación del inicio de D9.9 vs ROS/Gazebo en vivo. SOLO LECTURA.
Requiere ROS 2 Jazzy + rclpy, nav_msgs, rosgraph_msgs; no publica.
No certifica TF ni habilita navegación. Ejecutar con 3 Summit quietos.
"""
import argparse, hashlib, json, math, subprocess, time
from pathlib import Path

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def stamp(x):return float(x.sec)+float(x.nanosec)*1e-9

def main():
    pa=argparse.ArgumentParser(description=__doc__)
    pa.add_argument('--reference-report',type=Path,required=True)
    pa.add_argument('--world-sdf',type=Path,required=True)
    pa.add_argument('--scenario',type=Path,required=True)
    pa.add_argument('--observe-seconds',type=float,default=5)
    pa.add_argument('--max-start-error',type=float,default=.05)
    pa.add_argument('--max-stationary-speed',type=float,default=.03)
    pa.add_argument('--max-stamp-delta',type=float,default=.25)
    pa.add_argument('--output',type=Path,default=Path('scripts/resultados/d99_live_start_readonly.json'))
    a=pa.parse_args()
    ref=json.loads(a.reference_report.read_text())
    problems=[]
    if sha(a.world_sdf)!=ref['world']['run007_sdf_sha256']:problems.append('WORLD_SDF_HASH_DIFFERENT_FROM_RUN007')
    if sha(a.scenario)!=ref['world']['run007_scenario_sha256']:problems.append('SCENARIO_HASH_DIFFERENT_FROM_RUN007')
    try:
        cp=subprocess.run(['gz','model','--list'],capture_output=True,text=True,timeout=7)
        listing=(cp.stdout or '')+'\n'+(cp.stderr or '')
        if cp.returncode!=0:problems.append('GAZEBO_MODEL_LIST_NOT_AVAILABLE')
        for robot in ref['robots']:
            if robot not in listing:problems.append('MISSING_GAZEBO_MODEL_'+robot)
    except (OSError,subprocess.TimeoutExpired):problems.append('GAZEBO_MODEL_LIST_NOT_AVAILABLE')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    rclpy.init()
    node=Node('d99_start_gate_readonly')
    qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
    odoms={};clock_stamps=[]
    subs=[]
    def callback_odom(msg,name):odoms[name]=msg
    def callback_clock(msg):clock_stamps.append(stamp(msg.clock))
    subs.append(node.create_subscription(Clock,'/clock',callback_clock,qos))
    for name in ref['robots']:
        subs.append(node.create_subscription(Odometry,f'/{name}/odom',lambda msg, n=name:callback_odom(msg,n),qos))
    until=time.monotonic()+max(0.5,a.observe_seconds)
    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=min(.2,max(0,until-time.monotonic())))
    actual={};last_clock=clock_stamps[-1] if clock_stamps else None
    if not clock_stamps or max(clock_stamps)-min(clock_stamps)<.05:problems.append('CLOCK_NOT_ADVANCING')
    if any(y<x-1e-6 for x,y in zip(clock_stamps,clock_stamps[1:])):problems.append('CLOCK_RESET_OBSERVED')
    for name,reference in ref['robots'].items():
        if name not in odoms:problems.append('MISSING_ODOM_'+name);continue
        msg=odoms[name]
        xy=[float(msg.pose.pose.position.x),float(msg.pose.pose.position.y)]
        err=math.dist(xy,reference['d99_converted_start_xy_m'])
        vel=msg.twist.twist.linear
        speed=math.hypot(vel.x,vel.y)
        stamp_delta=abs(last_clock-stamp(msg.header.stamp)) if last_clock is not None else None
        frame_ok=(msg.header.frame_id==f'{name}/odom' and msg.child_frame_id==f'{name}/base_footprint')
        actual[name]={'observed_xy':xy,'reference_xy':reference['d99_converted_start_xy_m'],
            'error_m':err,'speed_m_s':speed,'frame_id':msg.header.frame_id,
            'child_frame_id':msg.child_frame_id,'timestamp_delta_s':stamp_delta}
        if err>a.max_start_error:problems.append('START_POSE_MISMATCH_'+name)
        if speed>a.max_stationary_speed:problems.append('ROBOT_NOT_STOPPED_'+name)
        if not frame_ok:problems.append('ODOM_FRAME_MISMATCH_'+name)
        if stamp_delta is None or stamp_delta>a.max_stamp_delta:problems.append('ODOM_STALE_'+name)
    node.destroy_node();rclpy.shutdown()
    result={'schema':'swarm.d99.runtime_start.readonly.v1',
        'status':'PASS_INERT_OBSERVATION_NOT_EXECUTION_AUTHORIZATION' if not problems else 'REJECT_READONLY_RUNTIME_START',
        'problems':problems,'robots':actual,'clock_last_s':last_clock,'clock_count':len(clock_stamps),
        'world_sdf_file_hash_matches_reference':sha(a.world_sdf)==ref['world']['run007_sdf_sha256'],
        'run_scenario_file_hash_matches_reference':sha(a.scenario)==ref['world']['run007_scenario_sha256'],
        'tf_certified':False,'gazebo_world_bytes_loaded_certified':False,
        'execution_authorized':False,'commands_sent':0,'publishers_created':0}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({'status':result['status'],'problems':problems,'robots':actual,'output':str(a.output)},indent=2,ensure_ascii=False))
    return 0 if not problems else 2

if __name__=='__main__':raise SystemExit(main())
