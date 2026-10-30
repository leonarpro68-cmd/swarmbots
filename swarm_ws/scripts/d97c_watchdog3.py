#!/usr/bin/env python3
"""Simulation-only three-robot command relay. Runs independently of trajectory producer.
Inputs /d97c/summitN/request; outputs /summitN/cmd_vel. Hard fault latches to zeros.
No hard physical failsafe if THIS relay process also crashes.
"""
import argparse, json, math, time
from pathlib import Path
from d97c_common import ROBOTS, clamp_xy

def main():
    p=argparse.ArgumentParser();p.add_argument('--enable-sim-relay',action='store_true')
    p.add_argument('--max-seconds',type=float,default=35.0)
    p.add_argument('--output',type=Path,default=Path('scripts/resultados/d9_live/d97c_relay.json'))
    a=p.parse_args()
    if not a.enable_sim_relay:raise SystemExit('INERT: --enable-sim-relay required')
    if not 8<=a.max_seconds<=60:raise SystemExit('max-seconds must be 8..60')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    class Relay(Node):
        def __init__(self):
            super().__init__('d97c_three_robot_relay')
            self.pub={};self.req={};self.msgstamp={};self.odomstamp={};self.vel={};self.count={r:0 for r in ROBOTS}
            self.fault=set();self.latched=False;self.started=False;self.start_wall=time.monotonic()
            self.clocklast=0;self.clockstamp=None;self.clockcount=0;self.publisher_conflicts=[]
            self.end_requested=False;self.end_wall=None;self.normal_end=False;self.stopped_samples=0
            for r in ROBOTS:
                self.req[r]=(0.,0.);self.msgstamp[r]=0.;self.odomstamp[r]=0.;self.vel[r]=0.
                self.create_subscription(Twist,f'/d97c/{r}/request',lambda m,n=r:self.on_request(n,m),10)
                self.create_subscription(Odometry,f'/{r}/odom',lambda m,n=r:self.on_odom(n,m),10)
            qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(Clock,'/clock',self.on_clock,qos)
            self.end_sub=self.create_subscription(String,'/d97c/session_end',self.on_end,10)
            self.timer=self.create_timer(.05,self.tick)
        def on_end(self,m):
            # A normal completion requires receiving the literal stop intent AND
            # confirming stationary odometry for every robot. Never accepts
            # another motion request after completion was requested.
            if m.data!='D97C_STOP_ALL':
                self.fault.add('BAD_END_MESSAGE');self.latched=True;return
            if not self.started:
                self.fault.add('END_BEFORE_MOVEMENT');self.latched=True;return
            self.end_requested=True;self.end_wall=time.monotonic()
            self.req={r:(0.,0.) for r in ROBOTS}
        def on_request(self,r,m):
            if self.end_requested:return
            vals=(m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z)
            if not all(map(math.isfinite,vals)) or any(abs(v)>1e-6 for v in (m.linear.z,m.angular.x,m.angular.y,m.angular.z)):
                self.fault.add('INVALID_REQUEST');self.latched=True;return
            # Preserve provenance of the actual request; rejects out-of-range commands instead of silently accepting them.
            if math.hypot(m.linear.x,m.linear.y)>1.20+1e-6:
                self.fault.add('OVERSPEED_REQUEST');self.latched=True;return
            self.req[r]=(m.linear.x,m.linear.y);self.msgstamp[r]=time.monotonic()
            if math.hypot(*self.req[r])>1e-4:self.started=True
        def on_odom(self,r,m):
            self.odomstamp[r]=time.monotonic();v=m.twist.twist.linear;self.vel[r]=math.hypot(v.x,v.y)
            if m.header.frame_id != r+'/odom' or m.child_frame_id != r+'/base_footprint':
                self.fault.add(r+':FRAME');self.latched=True
        def on_clock(self,m):
            t=m.clock.sec+m.clock.nanosec*1e-9
            if self.clockstamp is not None and t<self.clockstamp-.002:self.fault.add('CLOCK_REWIND');self.latched=True
            if self.clockstamp is None or t>self.clockstamp+1e-6:self.clocklast=time.monotonic()
            self.clockstamp=t;self.clockcount+=1
        def graph(self):
            for r in ROBOTS:
                names=[(x.node_namespace,x.node_name) for x in self.get_publishers_info_by_topic(f'/{r}/cmd_vel') if x.node_name!=self.get_name()]
                if names:self.fault.add(r+':COMPETING_PUBLISHER');self.publisher_conflicts.extend(names);self.latched=True
        def tick(self):
            if len(self.pub)!=3:return
            now=time.monotonic();self.graph()
            if self.started and not self.end_requested:
                if now-self.clocklast>.35:self.fault.add('CLOCK_STALE');self.latched=True
                for r in ROBOTS:
                    if now-self.odomstamp[r]>.35:self.fault.add(r+':ODOM_STALE');self.latched=True
                    if now-self.msgstamp[r]>.30:self.fault.add(r+':INPUT_TIMEOUT');self.latched=True
                    if self.vel[r]>1.35:self.fault.add(r+':MEASURED_OVERSPEED');self.latched=True
            if self.end_requested and not self.latched:
                if all(now-self.odomstamp[r]<.30 and self.vel[r]<.03 for r in ROBOTS):
                    self.stopped_samples+=1
                else:self.stopped_samples=0
                if self.stopped_samples>=8:self.normal_end=True
                if now-self.end_wall>2.0 and not self.normal_end:
                    self.fault.add('END_STOP_NOT_CONFIRMED');self.latched=True
            for r in ROBOTS:
                m=Twist()
                if not self.end_requested and not self.latched and self.started and all(now-self.msgstamp[x]<=.30 for x in ROBOTS):
                    m.linear.x,m.linear.y=clamp_xy(*self.req[r],1.20)
                self.pub[r].publish(m);self.count[r]+=1
    rclpy.init();n=Relay();state='REJECTED'
    try:
        deadline=time.monotonic()+3.
        while time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.025)
        n.graph()
        if n.fault or n.clockcount<2 or time.monotonic()-n.clocklast>.3 or any(time.monotonic()-n.odomstamp[r]>.3 for r in ROBOTS):
            n.fault.add('PRECHECK_FAILED')
        else:
            n.pub={r:n.create_publisher(Twist,f'/{r}/cmd_vel',10) for r in ROBOTS}
            state='RELAY_FINISHED'
            end=time.monotonic()+a.max_seconds
            while time.monotonic()<end:
                rclpy.spin_once(n,timeout_sec=.025)
                if n.normal_end:
                    state='NORMAL_END_STOP_CONFIRMED';break
    except KeyboardInterrupt:state='INTERRUPTED'
    finally:
        for _ in range(25):
            for publisher in n.pub.values():publisher.publish(Twist())
            time.sleep(.02)
        out={'status':state if not n.fault else 'FAULT_LATCHED','faults':sorted(n.fault),'latched':n.latched,
             'publisher_count':len(n.pub),'published':n.count,'clock_messages':n.clockcount,
             'saw_nonzero_requests':n.started,'end_requested':n.end_requested,'normal_end':n.normal_end,'stopped_samples':n.stopped_samples,'note':'Relay self-crash is NOT protected by Gazebo failsafe; simulation only.'}
        a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(out,indent=2)+'\n')
        print(json.dumps(out,indent=2));n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
