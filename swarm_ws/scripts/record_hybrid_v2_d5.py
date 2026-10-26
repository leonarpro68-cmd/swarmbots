#!/usr/bin/env python3
"""D.5: observador no intrusivo de Hybrid V2; iniciar ANTES del ejecutor.
No publica cmd_vel ni comandos híbridos. Guarda odometría, referencias y estados.
"""
import argparse, csv, json, math, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from swarm_behavior.swarm_planning.hybrid_refinement.timed_protocol import (
    decode_command, decode_status, reference, TimedProtocolError)
ROBOTS = ('summit0','summit1','summit2')
FIELDS=('t_sim_s','robot','session','frame','x_m','y_m','ref_x_m','ref_y_m','error_m','odom_v_m_s','ref_v_m_s','phase','status')

class Observer(Node):
    def __init__(self,args):
        super().__init__('hybrid_v2_d5_observer')
        self.args=args; self.clock_s=None; self.commands={}; self.latest={}; self.states={}; self.session=None
        self.wall_started=time.monotonic(); self.finish_wall=None; self.samples=0
        self.out=Path(args.output_dir);self.out.mkdir(parents=True,exist_ok=True)
        self.csv_file=(self.out/'odom_tracking.csv').open('w',newline='',encoding='utf-8')
        self.writer=csv.DictWriter(self.csv_file,fieldnames=FIELDS);self.writer.writeheader()
        self.events=(self.out/'events.jsonl').open('w',encoding='utf-8')
        self.subs=[]
        self.subs.append(self.create_subscription(Clock,'/clock',self.clock_cb,10))
        for robot in ROBOTS:
            self.subs.append(self.create_subscription(Odometry,f'/{robot}/odom',lambda m,r=robot:self.odom_cb(r,m),10))
            self.subs.append(self.create_subscription(String,f'/{robot}/hybrid_trajectory/command',lambda m,r=robot:self.command_cb(r,m),10))
            self.subs.append(self.create_subscription(String,f'/{robot}/hybrid_trajectory/status',lambda m,r=robot:self.status_cb(r,m),10))
        self.create_timer(1/max(1,args.sample_hz),self.sample)
        self.create_timer(.25,self.lifecycle)
        self.get_logger().info('D.5 observador pasivo activo; iniciar ahora ejecutor en otra terminal')
    def log_event(self,kind,**kwargs):
        self.events.write(json.dumps(dict(kind=kind,t_sim_s=self.clock_s,**kwargs),ensure_ascii=False)+'\n');self.events.flush()
    def clock_cb(self,msg):self.clock_s=float(msg.clock.sec)+float(msg.clock.nanosec)*1e-9
    def odom_cb(self,r,m):
        v=m.twist.twist.linear;p=m.pose.pose.position
        self.latest[r]=(float(p.x),float(p.y),math.hypot(v.x,v.y),str(m.header.frame_id),time.monotonic())
    def command_cb(self,r,msg):
        try:c=decode_command(msg.data)
        except TimedProtocolError:return
        if c.robot!=r:return
        if c.action=='start':
            if self.session is None:self.session=c.session_id;self.log_event('session',session=self.session)
            if c.session_id!=self.session:return
            self.commands[r]=c;self.log_event('start',robot=r,session=c.session_id,frame=c.frame_id,start_time_s=c.start_time_s,duration_s=c.duration_s,dt_s=c.dt_s)
            if len(self.commands)==3:self.log_event('all_commands_received',session=self.session)
        elif c.action=='cancel' and c.session_id==self.session:self.log_event('cancel_command',robot=r)
    def status_cb(self,r,msg):
        try:robot,sid,state=decode_status(msg.data)
        except TimedProtocolError:return
        if robot!=r or sid!=self.session:return
        if self.states.get(r)!=state:
            self.states[r]=state;self.log_event('status',robot=r,status=state)
    def sample(self):
        if self.clock_s is None or len(self.commands)!=3:return
        for r in ROBOTS:
            odom=self.latest.get(r)
            if not odom or time.monotonic()-odom[4]>2:return
        for r in ROBOTS:
            x,y,v,frame,_=self.latest[r];cmd=self.commands[r]
            ref,vel,phase=reference(cmd,self.clock_s)
            if frame!=cmd.frame_id:self.log_event('frame_mismatch',robot=r,actual=frame,expected=cmd.frame_id)
            self.writer.writerow(dict(t_sim_s=f'{self.clock_s:.6f}',robot=r,session=self.session,frame=frame,
                x_m=f'{x:.9f}',y_m=f'{y:.9f}',ref_x_m=f'{ref[0]:.9f}',ref_y_m=f'{ref[1]:.9f}',
                error_m=f'{math.dist((x,y),ref):.9f}',odom_v_m_s=f'{v:.9f}',ref_v_m_s=f'{math.hypot(*vel):.9f}',phase=phase,status=self.states.get(r,'')))
            self.samples+=1
        if self.samples%90==0:self.csv_file.flush()
    def lifecycle(self):
        if time.monotonic()-self.wall_started>self.args.timeout_s:
            self.get_logger().info('Fin por timeout del observador');rclpy.shutdown();return
        if len(self.states)==3 and all(s in ('done','failed','cancelled') for s in self.states.values()):
            if self.finish_wall is None:self.finish_wall=time.monotonic()
            if time.monotonic()-self.finish_wall>self.args.post_seconds:
                self.get_logger().info('Observación completada');rclpy.shutdown()
    def close(self):
        self.log_event('observer_end',samples=self.samples,statuses=self.states)
        self.csv_file.flush();self.csv_file.close();self.events.close()

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output-dir',required=True);ap.add_argument('--sample-hz',type=float,default=10)
    ap.add_argument('--timeout-s',type=float,default=240,help='Tiempo real máximo del observador')
    ap.add_argument('--post-seconds',type=float,default=2)
    args=ap.parse_args();rclpy.init();node=Observer(args)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    print('CSV:',Path(args.output_dir)/'odom_tracking.csv','EVENTS:',Path(args.output_dir)/'events.jsonl')
if __name__=='__main__':main()
