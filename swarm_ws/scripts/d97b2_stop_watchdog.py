#!/usr/bin/env python3
"""D9.7B.2 simulator-only isolated command relay; not a physical emergency stop."""
import argparse, json, math, time
from pathlib import Path


def sanitize(vx, vy=0.0, wz=0.0, limit=0.12):
    if not all(math.isfinite(x) for x in (vx,vy,wz)):
        return 0.0, False
    if abs(vy)>1e-6 or abs(wz)>1e-6 or vx < 0 or vx>limit:
        return 0.0, False
    return float(vx), True


def cmd_value(requested, age, elapsed, odom_age, clock_age, clock_advancing,
              conflict, displaced, *, timeout=.30, max_drive_s=1.0):
    reasons=[]
    if conflict: reasons.append('COMPETING_CMD_PUBLISHER')
    if odom_age>.35: reasons.append('ODOMETRY_STALE')
    if clock_age>.35 or not clock_advancing: reasons.append('CLOCK_STALE')
    if displaced>.25: reasons.append('TRAVEL_LIMIT')
    if elapsed>max_drive_s: reasons.append('MAX_DRIVE_TIME')
    if age>timeout: reasons.append('INPUT_TIMEOUT')
    vx, valid=sanitize(requested)
    if not valid: reasons.append('INVALID_INPUT')
    return (0.0 if reasons else vx), reasons


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--active-sim-only',action='store_true',help='Required explicit opt-in: writes /summit0/cmd_vel')
    ap.add_argument('--output',default='scripts/resultados/d9_live/d97b2_stop_test.json')
    ap.add_argument('--observe-seconds',type=float,default=7.0)
    args=ap.parse_args()
    if not args.active_sim_only:
        raise SystemExit('No publishers created. Pass --active-sim-only after checking Gazebo and no D8 controller.')
    if not (5<=args.observe_seconds<=15):
        raise SystemExit('observe-seconds must be 5..15')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    class Relay(Node):
        def __init__(self):
            super().__init__('d97b2_independent_zero_watchdog')
            self.input_vx=0.; self.input_time=-1e9; self.odom_time=-1e9; self.clock_time=-1e9
            self.clock_stamp=None; self.last_clock_change=-1e9; self.initial_xy=None; self.xy=None
            self.samples=0; self.published=0; self.nonzero_sent=0; self.max_displacement=0.
            self.max_measured_speed=0.; self.last_speed=0.; self.last_nonzero_wall=None; self.last_zero_wall=None
            self.reasons=set(); self.latch=False; self.arm_time=None; self.first_nonzero=None
            self.graph_checks=0; self.conflict=False; self.requests_received=0; self.nonzero_requests_received=0
            self.create_subscription(Twist,'/d97b2/summit0/request',self.request_cb,10)
            self.create_subscription(Odometry,'/summit0/odom',self.odom_cb,10)
            qos=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(Clock,'/clock',self.clock_cb,qos)
            self.pub=None
            self.timer=self.create_timer(.05,self.tick)
        def request_cb(self,msg):
            self.requests_received+=1
            if msg.linear.x>0: self.nonzero_requests_received+=1
            v,valid=sanitize(msg.linear.x,msg.linear.y,msg.angular.z)
            if not valid:
                self.latch=True; self.reasons.add('INVALID_INPUT'); return
            self.input_vx=v;self.input_time=time.monotonic()
            if v>0 and self.first_nonzero is None: self.first_nonzero=time.monotonic()
        def odom_cb(self,msg):
            self.odom_time=time.monotonic();self.samples+=1
            p=(msg.pose.pose.position.x,msg.pose.pose.position.y)
            self.xy=p
            if self.initial_xy is None:self.initial_xy=p
            d=math.dist(self.initial_xy,p);self.max_displacement=max(self.max_displacement,d)
            v=msg.twist.twist.linear
            self.last_speed=math.hypot(v.x,v.y)
            self.max_measured_speed=max(self.max_measured_speed,self.last_speed)
        def clock_cb(self,msg):
            self.clock_time=time.monotonic()
            stamp=msg.clock.sec+msg.clock.nanosec*1e-9
            if self.clock_stamp is not None and stamp < self.clock_stamp - .002:
                self.latch=True;self.reasons.add('CLOCK_REWIND')
            if self.clock_stamp is None or stamp>self.clock_stamp+1e-6:
                self.last_clock_change=time.monotonic()
            self.clock_stamp=stamp
        def inspect_graph(self):
            endpoints=self.get_publishers_info_by_topic('/summit0/cmd_vel')
            self.graph_checks+=1
            others=[p.node_name for p in endpoints if p.node_name!=self.get_name()]
            if others:
                self.conflict=True;self.latch=True;self.reasons.add('COMPETING_CMD_PUBLISHER:'+','.join(others))
        def tick(self):
            now=time.monotonic()
            if self.pub is None: return
            self.inspect_graph()
            start=self.first_nonzero
            elapsed=0 if start is None else now-start
            v, reasons=cmd_value(self.input_vx,now-self.input_time,elapsed,now-self.odom_time,
                 now-self.last_clock_change,now-self.last_clock_change<.35,self.conflict,
                 self.max_displacement)
            # Timeout while idle is normal; capture faults only after a nonzero command.
            critical=[r for r in reasons if r!='INPUT_TIMEOUT']
            if start is not None and (critical or now-self.input_time>.30):
                self.latch=True;self.reasons.update(reasons)
            if self.latch:v=0.0
            msg=Twist();msg.linear.x=v;self.pub.publish(msg);self.published+=1
            if v>0:self.nonzero_sent+=1;self.last_nonzero_wall=now
            else:self.last_zero_wall=now
    rclpy.init(); n=Relay(); begin=time.monotonic(); state='REJECT_PRECHECK'
    try:
        # Warmup must establish clock, odometry, no competitor before publisher created.
        deadline=begin+3.
        while time.monotonic()<deadline:
            rclpy.spin_once(n,timeout_sec=.02)
        n.inspect_graph()
        ready=(not n.conflict and n.samples>10 and n.clock_stamp is not None
               and time.monotonic()-n.last_clock_change<.3 and time.monotonic()-n.odom_time<.3)
        if ready:
            n.pub=n.create_publisher(Twist,'/summit0/cmd_vel',10)
            state='RELAY_STOP_TEST_COMPLETED'
            deadline=time.monotonic()+args.observe_seconds
            while time.monotonic()<deadline:
                rclpy.spin_once(n,timeout_sec=.02)
            # publish zero at shutdown, with grace time for Gazebo transit
            for _ in range(20):
                n.pub.publish(Twist());rclpy.spin_once(n,timeout_sec=.025)
        else:
            n.reasons.add('PRECHECK_NOT_READY')
    except KeyboardInterrupt:
        state='INTERRUPTED_ZERO_REQUESTED'
    finally:
        if n.pub:
            for _ in range(10): n.pub.publish(Twist());time.sleep(.02)
        if state=='RELAY_STOP_TEST_COMPLETED':
            if n.nonzero_sent==0: state='INCONCLUSIVE_NO_NONZERO_COMMAND'
            elif not n.latch: state='INCONCLUSIVE_NO_TIMEOUT_TRIGGER'
            elif n.max_displacement < .005: state='INCONCLUSIVE_NO_MEASURABLE_MOTION'
            elif n.last_speed > .03: state='REJECT_ROBOT_NOT_STOPPED'
            elif n.max_displacement > .25: state='REJECT_TRAVEL_LIMIT'
            elif n.conflict: state='REJECT_COMPETING_PUBLISHER'
            else: state='PASS_RELAY_TIMEOUT_STOP_OBSERVED_SIM_ONLY'
        result={'schema':'swarm.d97b2.stop_probe.v1','status':state,'publisher_created':n.pub is not None,
          'zero_or_nonzero_messages_sent':n.published,'nonzero_messages_sent':n.nonzero_sent,
          'requests_received':n.requests_received,'nonzero_requests_received':n.nonzero_requests_received,
          'odom_samples':n.samples,'max_displacement_m':n.max_displacement,
          'max_reported_speed_m_s':n.max_measured_speed,'final_reported_speed_m_s':n.last_speed,'clock_seen':n.clock_stamp is not None,
          'graph_checks':n.graph_checks,'faults':sorted(n.reasons),'input_timeout_s':.30,
          'command_speed_cap_m_s':.12,'max_drive_s':1.0,
          'stop_actuator_certified':False,'independent_process_watchdog_observed':bool(n.first_nonzero is not None and n.latch),
          'note':'Only Gazebo simulator. Zero after missing input is a software relay, not Gazebo/plugin dead-process failsafe.'}
        path=Path(args.output);path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2));n.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
