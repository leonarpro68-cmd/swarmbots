#!/usr/bin/env python3
"""Simulator-only short request; wait for relay subscription discovery before issuing pulse."""
import argparse, time

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--send-short-pulse',action='store_true')
    ap.add_argument('--discovery-timeout',type=float,default=8.0)
    args=ap.parse_args()
    if not args.send_short_pulse: raise SystemExit('Specify --send-short-pulse explicitly')
    if not 1 <= args.discovery_timeout <= 20: raise SystemExit('discovery-timeout must be 1..20')
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    rclpy.init(); node=Node('d97b2_short_pulse_source')
    pub=node.create_publisher(Twist,'/d97b2/summit0/request',10)
    try:
        until=time.monotonic()+args.discovery_timeout
        while pub.get_subscription_count()<1 and time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.05)
        n=pub.get_subscription_count()
        print(f'Request subscribers discovered: {n}',flush=True)
        if n!=1:
            raise SystemExit('ABORT: expected exactly 1 relay subscriber; no nonzero request published')
        # Keep subscribed match stable briefly before first request
        stable_until=time.monotonic()+.3
        while time.monotonic()<stable_until:
            rclpy.spin_once(node,timeout_sec=.05)
            if pub.get_subscription_count()!=1:
                raise SystemExit('ABORT: relay subscriber changed; no request published')
        sent=0
        until=time.monotonic()+.6
        while time.monotonic()<until:
            if pub.get_subscription_count()!=1:
                print('ABORT: relay subscription vanished',flush=True);break
            m=Twist(); m.linear.x=.08; pub.publish(m); sent+=1
            rclpy.spin_once(node,timeout_sec=.04)
        print(f'Requests published: {sent}; source ending, relay should time out in 0.30 s.',flush=True)
    finally:
        node.destroy_node(); rclpy.shutdown()
if __name__=='__main__':main()
