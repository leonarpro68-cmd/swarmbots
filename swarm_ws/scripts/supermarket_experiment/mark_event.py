#!/usr/bin/env python3
"""Append an operator-confirmed event with a ROS /clock timestamp; never controls robots."""
import argparse,csv,datetime,pathlib,sys
p=argparse.ArgumentParser();p.add_argument('--run-dir',required=True);p.add_argument('--event',required=True,choices=['mission_start','mission_end','pickup1_nav_start','pickup1_nav_end','pickup1_physical_done','delivery1_nav_start','delivery1_nav_end','delivery1_physical_done','pickup2_nav_start','pickup2_nav_end','pickup2_physical_done','delivery2_nav_start','delivery2_nav_end','delivery2_physical_done','pickup1_physical_start','delivery1_physical_start','pickup2_physical_start','delivery2_physical_start']);p.add_argument('--robot',choices=['summit0','summit1','summit2','all'],default='all');p.add_argument('--note',default='');p.add_argument('--clock-timeout',type=float,default=8.0);a=p.parse_args()
try:
 import rclpy
 from rclpy.node import Node
 from rosgraph_msgs.msg import Clock
except ImportError as e:sys.exit('ERROR: requires ROS 2 sourced environment: '+str(e))
class Reader(Node):
 def __init__(self):
  super().__init__('swarm_experiment_event_clock');self.now=None;self.create_subscription(Clock,'/clock',self.cb,10)
 def cb(self,msg):self.now=msg.clock.sec+msg.clock.nanosec*1e-9
rclpy.init();n=Reader()
try:
 import time
 deadline=time.monotonic()+a.clock_timeout
 while time.monotonic()<deadline and n.now is None:rclpy.spin_once(n,timeout_sec=.1)
 if n.now is None:sys.exit('ERROR: /clock not received; event NOT written')
 sim=n.now
finally:n.destroy_node();rclpy.shutdown()
path=pathlib.Path(a.run_dir).expanduser().resolve();path.mkdir(parents=True,exist_ok=True);f=path/'events.csv';exist=f.exists()
with f.open('a',newline='') as h:
 w=csv.writer(h)
 if not exist:w.writerow(['event','robot','sim_time_s','utc_time','note'])
 w.writerow([a.event,a.robot,f'{sim:.9f}',datetime.datetime.now(datetime.timezone.utc).isoformat(),a.note])
print('RECORDED',a.event,a.robot,'sim_s=',sim,'file=',f)
