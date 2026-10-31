#!/usr/bin/env python3
"""Read-only ROS2 recorder: /clock and /summit{0,1,2}/odom. No cmd_vel publisher."""
import argparse,csv,os,signal,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

class Recorder(Node):
    def __init__(self,path):
        super().__init__('supermarket_pickup1_odom_recorder')
        self.path=path
        self.f=open(path,'w',newline='',buffering=1)
        self.writer=csv.writer(self.f)
        self.writer.writerow(['robot','stamp_s','recv_wall_s','x_m','y_m','z_m','qx','qy','qz','qw','vx_local_m_s','vy_local_m_s','wz_rad_s','odom_frame','child_frame'])
        self.n=[0,0,0];self.sim_s=None
        self.create_subscription(Clock,'/clock',self.clock_cb,10)
        for i in range(3):
            self.create_subscription(Odometry,f'/summit{i}/odom',lambda msg,i=i:self.odom_cb(msg,i),50)
    def clock_cb(self,msg): self.sim_s=msg.clock.sec+msg.clock.nanosec*1e-9
    def odom_cb(self,msg,i):
        t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        p=msg.pose.pose.position;q=msg.pose.pose.orientation;v=msg.twist.twist
        self.writer.writerow([f'summit{i}',f'{t:.9f}',f'{time.time():.9f}',p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.linear.x,v.linear.y,v.angular.z,msg.header.frame_id,msg.child_frame_id]);self.n[i]+=1
    def close(self):
        self.f.flush();self.f.close();print('Saved:',self.path,'samples:',self.n,'last_sim_s:',self.sim_s,flush=True)

def main():
    a=argparse.ArgumentParser();a.add_argument('--output',required=True);args=a.parse_args()
    p=Path(args.output).expanduser().resolve();p.parent.mkdir(parents=True,exist_ok=True)
    rclpy.init(); node=Recorder(p)
    print('Recording odometry from 3 summits; Ctrl+C to finish. NO movement commands.',flush=True)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
