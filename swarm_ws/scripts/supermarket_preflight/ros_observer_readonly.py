#!/usr/bin/env python3
"""Read-only ROS2 preflight: subscribes; never publishes."""
import argparse, json, time, math
from pathlib import Path

def main():
 p=argparse.ArgumentParser()
 p.add_argument('--scenario',type=Path,required=True)
 p.add_argument('--output',type=Path,default=Path('/tmp/supermarket_pickup1_ros_observation.json'))
 p.add_argument('--seconds',type=float,default=8.0)
 p.add_argument('--start-tolerance',type=float,default=0.05)
 p.add_argument('--max-stationary-speed',type=float,default=0.01)
 args=p.parse_args()
 import rclpy
 from rclpy.node import Node
 from rosgraph_msgs.msg import Clock
 from nav_msgs.msg import Odometry
 from tf2_ros import Buffer, TransformListener
 scenario=json.loads(args.scenario.read_text())
 class Observer(Node):
  def __init__(self):
   super().__init__('supermarket_preflight_readonly')
   self.times=[];self.odom={}
   self.sub_clock=self.create_subscription(Clock,'/clock',self.clock,10)
   self.subs=[]
   self.tf_buffer=Buffer();self.tf_listener=TransformListener(self.tf_buffer,self)
   for i in range(3):
    n=f'summit{i}'
    self.subs.append(self.create_subscription(Odometry,f'/{n}/odom',lambda m,n=n:self.odom_cb(n,m),10))
  def clock(self,msg):self.times.append(msg.clock.sec+msg.clock.nanosec*1e-9)
  def odom_cb(self,n,msg):
   po=msg.pose.pose.position;vel=msg.twist.twist.linear
   self.odom[n]={'xy':[po.x,po.y],'speed':math.hypot(vel.x,vel.y), 'frame':msg.header.frame_id,
                'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9}
 rclpy.init();node=Observer()
 try:
  end=time.monotonic()+args.seconds
  while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
  check={}
  check['clock_advancing']=len(node.times)>=3 and node.times[-1]-node.times[0]>=1.0
  report={'schema':'swarm.supermarket.preflight.runtime.v1','status':'BLOCKED','checks':check,'robots':{},
          'note':'Solo lectura; odom y TF deben estar en un marco que permita comparar con las posiciones globales del escenario.'}
  for i,sc in enumerate(scenario['robots']):
   n=f'summit{i}';od=node.odom.get(n)
   pubs=node.get_publishers_info_by_topic(f'/{n}/cmd_vel')
   owners=[{'name':pub.node_name,'namespace':pub.node_namespace} for pub in pubs]
   owner_ok=(len(owners)==1 and owners[0]['name']=='path_follower' and owners[0]['namespace'].rstrip('/')==f'/{n}')
   info={'expected_xy_m':sc['xy_m'],'odom':od,'cmd_vel_publishers':owners}
   check[f'{n}_single_cmd_vel_owner']=owner_ok
   check[f'{n}_odom_present']=od is not None
   if od:
    age=node.times[-1]-od['stamp'] if node.times else float('inf')
    # if odom frame is local, do NOT assert pose match; mark unresolved.
    comparable=(od['frame']==f'{n}/odom')
    diff=math.dist(od['xy'],sc['xy_m']) if comparable else None
    info['start_error_m_if_coordinates_comparable']=diff
    check[f'{n}_odom_fresh']=0<=age<=0.5
    check[f'{n}_stationary']=od['speed']<=args.max_stationary_speed
    check[f'{n}_initial_position_matching_offline_scenario']=bool(comparable and diff<=args.start_tolerance)
   # Require available TF to at least check local tree; requires dynamic tree in running Gazebo.
   try:
    tr=node.tf_buffer.lookup_transform(f'{n}/odom',f'{n}/base_link',rclpy.time.Time())
    info['tf_odom_base_link']='available';check[f'{n}_tf_odom_to_base_link']=True
   except Exception as exc:
    info['tf_odom_base_link_error']=str(exc);check[f'{n}_tf_odom_to_base_link']=False
   report['robots'][n]=info
  report['status']='PASS_READ_ONLY_NOT_EXECUTION_AUTHORIZATION' if all(check.values()) else 'BLOCKED'
  args.output.parent.mkdir(parents=True,exist_ok=True)
  args.output.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
  print(json.dumps(report,indent=2,ensure_ascii=False))
  return 0 if all(check.values()) else 2
 finally:
  node.destroy_node();rclpy.shutdown()
if __name__=='__main__':raise SystemExit(main())
