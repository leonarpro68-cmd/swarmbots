#!/usr/bin/env python3
"""D9.9.7: align historical Bézier starts using EXISTING go_to_goal nodes.

Fail-closed: dry-run by default; --execute launches go_to_goal only for robots
outside tolerance. Never launches the historical trajectory or alters D8.
"""
import argparse, json, math, os, signal, subprocess, sys, time
from pathlib import Path
import numpy as np

ROBOTS=tuple(f'summit{i}' for i in range(3))

def read_starts(path):
    with np.load(path,allow_pickle=False) as z:
        out={}
        for i,r in enumerate(ROBOTS):
            c=z[f'R{i}_controls']
            if c.ndim!=3 or c.shape[1:]!=(6,2) or len(c)==0 or not np.isfinite(c).all():
                raise ValueError(f'Invalid controls {r}')
            out[r]=tuple(map(float,c[0,0]))
        return out

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trajectory',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--execute',action='store_true',help='Explicitly permit physical/simulated movement')
    p.add_argument('--tolerance',type=float,default=.025)
    p.add_argument('--max-speed',type=float,default=.12)
    p.add_argument('--max-seconds',type=float,default=40.)
    p.add_argument('--max-offset',type=float,default=.30)
    p.add_argument('--stationary-speed',type=float,default=.03)
    p.add_argument('--stationary-seconds',type=float,default=1.0)
    a=p.parse_args()
    if not (0<a.tolerance<=.05 and 0<a.max_speed<=.2 and a.max_seconds>1 and .05<=a.max_offset<=.5):p.error('Unsafe parameters')
    targets=read_starts(a.trajectory)
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init()
    n=Node('d997_initial_alignment_supervisor',parameter_overrides=[Parameter('use_sim_time',Parameter.Type.BOOL,True)])
    seen={};clock=[];subs=[]; processes={}
    for r in ROBOTS:
        subs.append(n.create_subscription(Odometry,f'/{r}/odom',lambda m,r=r:seen.__setitem__(r,(m,time.monotonic())),qos_profile_sensor_data))
    subs.append(n.create_subscription(Clock,'/clock',lambda m:clock.append((m.clock.sec+m.clock.nanosec*1e-9,time.monotonic())),qos_profile_sensor_data))
    problems=[];state='REJECT';start=time.monotonic(); stable_since=None; initial={}
    def snapshot():
        d={}
        for r in ROBOTS:
            if r not in seen or time.monotonic()-seen[r][1]>.6:return None
            msg=seen[r][0];pos=msg.pose.pose.position;vel=msg.twist.twist.linear
            if msg.header.frame_id!=f'{r}/odom' or msg.child_frame_id!=f'{r}/base_footprint':return None
            d[r]={'xy':[float(pos.x),float(pos.y)],'speed':float(math.hypot(vel.x,vel.y)),
                  'error':float(math.dist((pos.x,pos.y),targets[r]))}
        if len(clock)<2 or time.monotonic()-clock[-1][1]>.6 or clock[-1][0]-clock[0][0]<.05:return None
        return d
    try:
        while time.monotonic()-start<10:
            rclpy.spin_once(n,timeout_sec=.1)
            initial=snapshot()
            if initial:break
        if not initial:raise RuntimeError('Missing fresh odometry/clock or incorrect frames')
        if any(x['speed']>a.stationary_speed for x in initial.values()):raise RuntimeError('Robots are not stationary')
        if any(x['error']>a.max_offset for x in initial.values()):raise RuntimeError('Initial offset exceeds max-offset; do not align blindly')
        movable=[r for r in ROBOTS if initial[r]['error']>a.tolerance]
        # Fail-closed: ensure no existing Twist publishers (Hybrid follower or go_to_goal).
        for r in ROBOTS:
            publishers=n.get_publishers_info_by_topic(f'/{r}/cmd_vel')
            if publishers:raise RuntimeError(f'{r}: cmd_vel publishers active ({len(publishers)}); stop existing controllers first')
        if not a.execute:
            state='DRY_RUN_READY' if movable else 'ALREADY_ALIGNED'
        elif not movable:
            state='ALIGNED';
        else:
            for r in movable:
                x,y=targets[r]
                cmd=['ros2','run','swarm_behavior','go_to_goal','--ros-args','-r',f'__ns:=/{r}',
                     '-p',f'goal_x:={x}','-p',f'goal_y:={y}', '-p','n_robots:=1',
                     '-p','use_goal_topic:=false',
                     '-p',f'goal_tol:={a.tolerance/2}', '-p',f'max_lin_vel:={a.max_speed}',
                     '-p','max_ang_vel:=0.3','-p','slow_radius:=0.15']
                processes[r]=subprocess.Popen(cmd,start_new_session=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            end=time.monotonic()+a.max_seconds
            while time.monotonic()<end:
                rclpy.spin_once(n,timeout_sec=.1)
                if any(proc.poll() is not None for proc in processes.values()):raise RuntimeError('go_to_goal exited during alignment')
                d=snapshot()
                if not d:raise RuntimeError('Odometry or clock stale during alignment')
                if any(d[r]['error']>a.max_offset+.15 for r in movable):raise RuntimeError('Alignment deviated too far')
                if all(d[r]['error']<=a.tolerance and d[r]['speed']<=a.stationary_speed for r in ROBOTS):
                    if stable_since is None:stable_since=time.monotonic()
                    elif time.monotonic()-stable_since>=a.stationary_seconds:
                        state='ALIGNED';break
                else:stable_since=None
            if state!='ALIGNED':raise RuntimeError('Alignment timeout or did not settle')
    except Exception as ex:problems.append(str(ex))
    finally:
        # Hand off cmd_vel ownership: stop EVERY alignment controller before future follower.
        for proc in processes.values():
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
        for proc in processes.values():
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGTERM)
                try:proc.wait(timeout=2)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        final=snapshot()
        if a.execute and state=='ALIGNED':
            # Re-observe without the controller; if drifted, deny handoff.
            for _ in range(10):rclpy.spin_once(n,timeout_sec=.1)
            final=snapshot()
            if not final or any(q['error']>a.tolerance or q['speed']>a.stationary_speed for q in final.values()):
                state='REJECT';problems.append('Alignment not retained after go_to_goal shutdown')
            else:
                for r in ROBOTS:
                    if n.get_publishers_info_by_topic(f'/{r}/cmd_vel'):
                        state='REJECT';problems.append('cmd_vel still has a publisher after handoff');break
        report={'schema':'swarm.d997.alignment.v1','status':state,'execution_requested':a.execute,
                'main_trajectory_execution_authorized':False,'trajectory':str(a.trajectory.resolve()),
                'targets':targets,'initial':initial,'final':final,'launched_alignment_robots':list(processes),
                'problems':problems}
        a.output.parent.mkdir(parents=True,exist_ok=True)
        a.output.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
        print(json.dumps({'status':state,'problems':problems,'report':str(a.output)},ensure_ascii=False,indent=2))
        n.destroy_node();rclpy.shutdown()
    return 0 if state in ('DRY_RUN_READY','ALREADY_ALIGNED','ALIGNED') and not problems else 2

if __name__=='__main__':sys.exit(main())
