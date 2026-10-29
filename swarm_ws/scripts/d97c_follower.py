#!/usr/bin/env python3
"""D97C Gazebo-only live Bézier follower using separate three-robot watchdog relay.
Default preview is inert. Active requires explicit --execute-in-gazebo flag.
"""
import argparse,csv,json,math,time
from pathlib import Path
from d97c_common import ROBOTS,gate
from d97_executor_core import calculate_command

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--run',required=True,type=Path);p.add_argument('--stage',default='pickup1')
    p.add_argument('--execute-in-gazebo',action='store_true')
    p.add_argument('--full-pickup1',action='store_true',help='Explicitly opt in to full trajectory; default is 0.6 sim-second checkout')
    p.add_argument('--max-run-seconds',type=float,default=15.)
    p.add_argument('--kp',type=float,default=.8)
    a=p.parse_args()
    if a.stage!='pickup1':p.error('First actuation permits pickup1 only')
    if not 0<=a.kp<=1.2 or not 9<=a.max_run_seconds<=20:p.error('Controller limits exceeded')
    from d96a_readonly_follower import load_stage,sample
    manifest,traj=load_stage(a.run,a.stage)
    pipeline=json.loads((a.run/'d96c2b_report.json').read_text())
    if pipeline.get('status')!='PASS_READ_ONLY_LIVE_REPLAN':p.error('Source live pipeline not certified')
    horizon=max(float(x[1].sum()) for x in traj.values())
    if horizon>a.max_run_seconds-1:p.error('Horizon exceeds bounded run duration')
    reportpath=a.run/'d97c'/f'{a.stage}_d97c_report.json';reportpath.parent.mkdir(parents=True,exist_ok=True)
    if not a.execute_in_gazebo:
        print(json.dumps({'status':'INERT_PREVIEW','horizon_s':horizon,'command_publishers':0},indent=2));return
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    class Follower(Node):
        def __init__(self):
            super().__init__('d97c_follower')
            self.obs={};self.clock=None;self.clock_wall=0;self.clock_count=0;self.faults=[]
            self.previous={r:(0.,0.) for r in ROBOTS};self.last_step=None;self.last_req_wall=0
            self.pub={};self.end_pub=None;self.n_sent=0;self.min_sep=float('inf');self.peak_err={r:0. for r in ROBOTS}
            for r in ROBOTS:self.create_subscription(Odometry,f'/{r}/odom',lambda m,n=r:self.odom_cb(n,m),10)
            qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(Clock,'/clock',self.clock_cb,qos)
        def odom_cb(self,r,m):
            q=m.pose.pose.orientation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            v=m.twist.twist.linear
            self.obs[r]={'x':m.pose.pose.position.x,'y':m.pose.pose.position.y,'yaw':yaw,
              'speed':math.hypot(v.x,v.y),'frame_id':m.header.frame_id,'child_frame_id':m.child_frame_id,
              'wall':time.monotonic()}
        def clock_cb(self,m):
            stamp=m.clock.sec+m.clock.nanosec*1e-9
            if self.clock is not None and stamp<self.clock-.002:self.faults.append('CLOCK_REWIND')
            if self.clock is None or stamp>self.clock+1e-6:self.clock_wall=time.monotonic()
            self.clock=stamp;self.clock_count+=1
        def observed(self):
            now=time.monotonic()
            return {r:{**{k:v for k,v in x.items() if k!='wall'},'age':now-x['wall']} for r,x in self.obs.items()}
        def send(self,cmd):
            for r in ROBOTS:
                m=Twist();m.linear.x=cmd[r][0];m.linear.y=cmd[r][1];self.pub[r].publish(m);self.n_sent+=1
        def zeros(self):
            if self.pub:self.send({r:(0.,0.) for r in ROBOTS})
    rclpy.init();n=Follower();state='REJECTED';rows=[];origin=None
    try:
        # Verify independent watchdog is the only writer, and subscriptions exist on request topics.
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.02)
        zero=sample(traj,0.)
        if n.clock is None or time.monotonic()-n.clock_wall>.3:n.faults.append('CLOCK_UNAVAILABLE')
        issues=gate(n.observed(),zero,start=True);n.faults.extend(issues)
        if len(n.obs)!=3:n.faults.append('ODOMETRY_INCOMPLETE')
        for r in ROBOTS:
            writers=n.get_publishers_info_by_topic(f'/{r}/cmd_vel')
            if len(writers)!=1 or writers[0].node_name!='d97c_three_robot_relay':n.faults.append(r+':RELAY_NOT_EXCLUSIVE')
            if n.count_subscribers(f'/d97c/{r}/request')!=1:n.faults.append(r+':REQUEST_SUBSCRIBER_MISSING')
        if n.clock is not None and (n.clock-float(pipeline.get('snapshot_time_s',-1e9))>120 or n.clock<float(pipeline.get('snapshot_time_s',1e9))):
            n.faults.append('PLAN_STALE_OR_CLOCK_RESET')
        if not n.faults:
            n.pub={r:n.create_publisher(Twist,f'/d97c/{r}/request',10) for r in ROBOTS}
            n.end_pub=n.create_publisher(String,'/d97c/session_end',10)
            # Discovery delay while still inert. Requires current odometry check after delay.
            ready=time.monotonic()+.6
            while time.monotonic()<ready:rclpy.spin_once(n,timeout_sec=.02)
            n.faults.extend(gate(n.observed(),zero,start=True))
        if not n.faults:
            origin=n.clock;state='RUNNING';last_wall=time.monotonic();run_wall_start=last_wall
            while True:
                rclpy.spin_once(n,timeout_sec=.01)
                now=time.monotonic()
                if n.clock is None or now-n.clock_wall>.3:n.faults.append('CLOCK_STALE')
                t=0 if n.clock is None else n.clock-origin
                if t<-.002:n.faults.append('CLOCK_REWIND')
                if t>(horizon+.03 if a.full_pickup1 else .60):break
                if now-run_wall_start>a.max_run_seconds:n.faults.append('WALLTIME_LIMIT')
                if now-last_wall<.02:continue
                if now-last_wall>.20:n.faults.append('CONTROL_LOOP_STALLED')
                if n.faults:break
                refs=sample(traj,max(0,min(horizon,t)))
                obs=n.observed()
                n.faults.extend(gate(obs,refs))
                if n.faults:break
                sep=min(math.dist((obs[x]['x'],obs[x]['y']),(obs[y]['x'],obs[y]['y']))
                        for i,x in enumerate(ROBOTS) for y in ROBOTS[i+1:]);n.min_sep=min(sep,n.min_sep)
                dt=min(.10,max(.005,now-last_wall));last_wall=now
                commands={}
                for r in ROBOTS:
                    o=obs[r]
                    cmd=calculate_command(refs[r],{'x_m':o['x'],'y_m':o['y'],'yaw_rad':o['yaw']},n.previous[r],dt,kp=a.kp)
                    n.previous[r]=(cmd['vx_body_m_s'],cmd['vy_body_m_s'])
                    commands[r]=n.previous[r];n.peak_err[r]=max(n.peak_err[r],cmd['position_error_m'])
                n.send(commands)
                rows.append([f'{t:.6f}',*([f'{n.peak_err[r]:.6f}' for r in ROBOTS]),f'{sep:.6f}'])
                if time.monotonic()-last_wall>.2:n.faults.append('LOOP_TIMING')
                
            if not n.faults and a.full_pickup1:
                final_refs=sample(traj,horizon)
                final_obs=n.observed()
                n.faults.extend(gate(final_obs,final_refs,track=.12))
                if n.faults:n.faults.append('FINAL_GOAL_NOT_REACHED')
            if not n.faults:state=('FINISHED_TRAJECTORY_NOT_MISSION' if a.full_pickup1 else 'SHORT_CHECKOUT_COMPLETED')
            else:state='FAULT_STOP_REQUESTED'
        else:state='REJECTED_PRECHECK'
    except KeyboardInterrupt:
        state='INTERRUPTED_STOP_REQUESTED'
    except Exception as exc:
        n.faults.append(type(exc).__name__+':'+str(exc));state='EXCEPTION_STOP_REQUESTED'
    finally:
        # Keep requesting zeros until all measured speeds are low. Only then
        # announce normal end to the independent relay. A fault NEVER sends END.
        stop_confirmed=False;stop_samples=0
        for _ in range(100):
            n.zeros();rclpy.spin_once(n,timeout_sec=.02)
            obs=n.observed()
            if len(obs)==3 and all(obs[r]['age']<.30 and obs[r]['speed']<.03 for r in ROBOTS):
                stop_samples+=1
            else:stop_samples=0
            if stop_samples>=8:stop_confirmed=True;break
        if state in ('SHORT_CHECKOUT_COMPLETED','FINISHED_TRAJECTORY_NOT_MISSION') and not stop_confirmed:
            n.faults.append('STOP_NOT_CONFIRMED');state='FAULT_STOP_NOT_CONFIRMED'
        if stop_confirmed and state in ('SHORT_CHECKOUT_COMPLETED','FINISHED_TRAJECTORY_NOT_MISSION') and n.end_pub is not None:
            # Repeated stop-intent messages give DDS discovery time to settle.
            for _ in range(15):
                msg=String();msg.data='D97C_STOP_ALL';n.end_pub.publish(msg)
                n.zeros();rclpy.spin_once(n,timeout_sec=.02)
        csvpath=reportpath.with_suffix('.csv')
        with csvpath.open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['t_s','peak_err_0_m','peak_err_1_m','peak_err_2_m','separation_m']);w.writerows(rows)
        result={'schema':'swarm.d97c.first_movement.v1','status':state,'faults':n.faults,
          'horizon_s':horizon,'samples':len(rows),'requests_sent_including_zeros':n.n_sent,
          'peak_tracking_error_m':n.peak_err,'min_measured_separation_m':n.min_sep if math.isfinite(n.min_sep) else None,
          'clock_messages':n.clock_count,'final_stop_confirmed':stop_confirmed,'end_signal_sent':bool(stop_confirmed and state in ('SHORT_CHECKOUT_COMPLETED','FINISHED_TRAJECTORY_NOT_MISSION')),'final_observed_speed_m_s':{r:n.obs.get(r,{}).get('speed') for r in ROBOTS},'output_csv':str(csvpath),
          'note':'SIMULATION ONLY. Actual completion must be confirmed with odometry and relay report; robot stopping on simultaneous relay/process crash NOT certified.'}
        reportpath.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
        n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
