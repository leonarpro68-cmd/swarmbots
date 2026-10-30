#!/usr/bin/env python3
"""D9.7B.1: controlled zero-Twist ROS transport test. NEVER sends nonzero motion.

No external watchdog or fail-safe is provided by this script. Use only in Gazebo.
"""
import argparse, json, math, time
from pathlib import Path

TOPICS = [f'/summit{i}/cmd_vel' for i in range(3)]
WATCH_TOPICS = ['/cmd_vel'] + TOPICS + [f'/model/summit{i}/cmd_vel' for i in range(3)] + [f'/summit{i}/cmd_vel_stamped' for i in range(3)]

def decide(observations, graph, clock_n, drift_limit=0.025):
    reasons = []
    for topic, pubs in graph.items():
        if pubs: reasons.append('EXTERNAL_PUBLISHER:'+topic)
    if clock_n < 2: reasons.append('CLOCK_MISSING')
    if set(observations) != {f'summit{i}' for i in range(3)}:
        reasons.append('MISSING_ODOMETRY')
    for name, samples in observations.items():
        if len(samples) < 2:
            reasons.append('INSUFFICIENT_ODOM:'+name); continue
        x0,y0=samples[0]
        if max(math.hypot(x-x0,y-y0) for x,y in samples) > drift_limit:
            reasons.append('DRIFT:'+name)
    return reasons

def run(args):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    class Test(Node):
        def __init__(self):
            super().__init__('d97b1_zero_transport')
            self.pos={f'summit{i}':[] for i in range(3)}
            self.clock=[]
            self.subs=[]
            for i in range(3):
                name=f'summit{i}'
                self.subs.append(self.create_subscription(Odometry, f'/{name}/odom',
                    lambda m,n=name:self.odometry(n,m), 10))
            clock_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                                   reliability=ReliabilityPolicy.BEST_EFFORT,
                                   durability=DurabilityPolicy.VOLATILE)
            self.subs.append(self.create_subscription(Clock, '/clock', self.clock_cb, clock_qos))
        def odometry(self,name,msg):
            self.pos[name].append((msg.pose.pose.position.x,msg.pose.pose.position.y))
        def clock_cb(self,msg):
            self.clock.append(msg.clock.sec+msg.clock.nanosec*1e-9)
    rclpy.init()
    node=Test()
    pubs=[]
    try:
        # Allow DDS discovery without bypassing the clock safety gate.
        end=time.monotonic()+args.warmup_seconds
        while time.monotonic()<end or (len(set(node.clock)) < 2 and time.monotonic()<end+args.clock_grace_seconds):
            rclpy.spin_once(node,timeout_sec=.05)
        clock_publishers = [
            {'node': p.node_name, 'namespace': p.node_namespace,
             'reliability': str(p.qos_profile.reliability),
             'durability': str(p.qos_profile.durability)}
            for p in node.get_publishers_info_by_topic('/clock')
        ]
        graph={t:[p.node_name for p in node.get_publishers_info_by_topic(t)] for t in WATCH_TOPICS}
        reasons=decide(node.pos,graph,len(set(node.clock)))
        # A graph check is only a snapshot. Never claim guaranteed exclusivity.
        if not reasons and args.send_zero:
            pubs=[node.create_publisher(Twist,t,10) for t in TOPICS]
            deadline=time.monotonic()+args.observe_seconds
            next_pub=time.monotonic(); zeros=0
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.025)
                now=time.monotonic()
                if now>=next_pub:
                    current={t:[p.node_name for p in node.get_publishers_info_by_topic(t)
                                if p.node_name!=node.get_name()] for t in WATCH_TOPICS}
                    # Dynamic conflict; publish only zeros and stop promptly.
                    if any(current.values()):
                        reasons.append('EXTERNAL_PUBLISHER_DURING_TEST');break
                    for p in pubs:p.publish(Twist());zeros+=3
                    next_pub=now+.1
        else:
            zeros=0
        if not node.clock or node.clock[-1] <= node.clock[0]:reasons.append('CLOCK_NOT_ADVANCING')
        reasons+=decide(node.pos,{},len(set(node.clock)))
        report={'schema':'swarm.d97b1.zero_transport.v1',
          'status':('PASS_ZERO_TRANSPORT_OBSERVED_NOT_FAILSAFE' if not reasons and args.send_zero
                    else 'PASS_PREFLIGHT_NO_PUBLISHERS' if not reasons else 'REJECT_ZERO_TEST'),
          'zero_publishers_created':len(pubs),'zero_messages_sent':zeros,
          'nonzero_commands_sent':0,'execution_authorized':False,
          'independent_stop_watchdog_verified':False,
          'clock_messages':len(node.clock),
          'clock_first_s':node.clock[0] if node.clock else None,
          'clock_last_s':node.clock[-1] if node.clock else None,
          'clock_publishers_discovered':clock_publishers,
          'clock_subscription_qos':'BEST_EFFORT/VOLATILE/KEEP_LAST(10)',
          'odom_samples':{k:len(v) for k,v in node.pos.items()},
          'reasons':list(dict.fromkeys(reasons)),
          'note':'Zero Twists only; does not demonstrate actual stop under failure or Gazebo bridge delivery.'}
        out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True)
        out.write_text(json.dumps(report,indent=2)+'\n')
        return report
    finally:
        # Best effort final zero on normal Python shutdown only.
        for p in pubs:
            try:p.publish(Twist())
            except Exception:pass
        node.destroy_node();rclpy.shutdown()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True)
    p.add_argument('--warmup-seconds',type=float,default=2.)
    p.add_argument('--observe-seconds',type=float,default=4.)
    p.add_argument('--clock-grace-seconds',type=float,default=5.0, help='Maximum extra DDS discovery wait if /clock has fewer than two unique samples')
    p.add_argument('--send-zero',action='store_true',help='Explicitly opt-in to publish ZERO velocities only')
    args=p.parse_args()
    if not(.5<=args.warmup_seconds<=10 and 1<=args.observe_seconds<=10 and 0<=args.clock_grace_seconds<=15):p.error('Out of bounds seconds')
    print(json.dumps(run(args),indent=2))
if __name__=='__main__':main()
