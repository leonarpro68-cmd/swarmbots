#!/usr/bin/env python3
"""D9.6C.2E: discover Gazebo world pose endpoints and ROS odom safely.
Never publishes to ROS/Gazebo or calls modifying Gazebo services.
"""
import argparse, json, re, subprocess, time
from pathlib import Path

ROBOTS = ('summit0','summit1','summit2')

def call(argv, timeout=5):
    try:
        p=subprocess.run(argv, capture_output=True, text=True, timeout=timeout, check=False)
        return {'returncode':p.returncode, 'stdout':p.stdout[:30000], 'stderr':p.stderr[:2000]}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {'returncode':None,'stdout':'','stderr':str(exc)}

def discover(topic_list, service_list):
    topics=[s.strip() for s in topic_list.splitlines() if s.strip().startswith('/')]
    services=[s.strip() for s in service_list.splitlines() if s.strip().startswith('/')]
    worlds=sorted(set(re.findall(r'^/world/([^/]+)/', '\n'.join(topics+services), re.M)))
    poses=[s for s in topics if re.search(r'/(?:dynamic_pose|pose)/info$',s)]
    states=[s for s in services if re.search(r'/state$',s)]
    return {'worlds':worlds,'pose_topics':poses,'state_services':states,
            'robot_odometry_gz_topics':[s for s in topics if re.search(r'/model/summit[012]/odometry$',s)],
            'gazebo_topic_count':len(topics),'gazebo_service_count':len(services)}

def capture_ros(seconds):
    import rclpy
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
    rclpy.init(args=None)
    class Observer(Node):
        def __init__(self):
            super().__init__('d96c2e_frame_probe_readonly')
            self.odom={}; self.clock=[]
            self.subs=[self.create_subscription(Odometry,f'/{r}/odom',lambda m,r=r:self.on_odom(r,m),qos_profile_sensor_data) for r in ROBOTS]
            self.subs.append(self.create_subscription(Clock,'/clock',self.clock_cb,QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)))
        def on_odom(self,r,m):
            p=m.pose.pose.position; q=m.pose.pose.orientation; v=m.twist.twist.linear
            self.odom[r]={'header_frame_id':m.header.frame_id,'child_frame_id':m.child_frame_id,
                'stamp_s':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                'position':{'x':p.x,'y':p.y,'z':p.z},
                'orientation':{'x':q.x,'y':q.y,'z':q.z,'w':q.w},
                'velocity':{'x':v.x,'y':v.y,'z':v.z},'received_monotonic_s':time.monotonic()}
        def clock_cb(self,m):self.clock.append(m.clock.sec+m.clock.nanosec*1e-9)
    n=Observer()
    try:
        deadline=time.monotonic()+seconds
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(n,timeout_sec=.06)
        for v in n.odom.values(): v['age_wall_s']=time.monotonic()-v.pop('received_monotonic_s')
        return {'robots':n.odom,'clock_messages':len(n.clock),'clock_first_s':n.clock[0] if n.clock else None,
                'clock_last_s':n.clock[-1] if n.clock else None}
    finally:
        n.destroy_node();rclpy.shutdown()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--observe-seconds',type=float,default=3.)
    a=p.parse_args()
    if not 1<=a.observe_seconds<=15:p.error('observe-seconds must be 1..15')
    topics=call(['gz','topic','-l']); services=call(['gz','service','-l'])
    endpoints=discover(topics['stdout'],services['stdout'])
    try: ros=capture_ros(a.observe_seconds)
    except Exception as exc: ros={'error':repr(exc)}
    complete=all(r in ros.get('robots',{}) for r in ROBOTS)
    result={'schema':'swarm.d96c2e.discovery.readonly.v1','status':'POSE_SOURCE_DISCOVERED_CALIBRATION_PENDING' if endpoints['pose_topics'] and complete else 'NEEDS_GAZEBO_POSE_SOURCE_OR_ROS_ODOM',
            'gazebo':endpoints,'ros':ros,'gz_topic_query_error':topics['stderr'],'gz_service_query_error':services['stderr'],
            'coordinate_frame_equivalence_proven':False,'execution_authorized':False,
            'publishers_created':0,'commands_sent':0,
            'note':'Discovery only. A topic name and static odometry agreement cannot establish frame equivalence. Calibrate using matched world poses and robot motion before permitting execution.'}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    print('Report:',a.output)
if __name__=='__main__':main()
