#!/usr/bin/env python3
"""Registro pasivo de odometría ROS 2 por etapa Hybrid V2 (no publica cmd_vel).

La velocidad twist.linear está expresada en el frame child_frame_id (habitualmente base_link).
La rapidez es invariante ante rotación; para comparaciones vectoriales hace falta TF.
"""
import argparse
import csv
import math
import signal
import time
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stage', required=True, choices=['pickup1','delivery1','pickup2','delivery2'])
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--ready-file', type=Path, required=True)
    args=p.parse_args()
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.node import Node

    args.output_dir.mkdir(parents=True, exist_ok=True)
    args.ready_file.unlink(missing_ok=True)
    rclpy.init()
    node=Node('d8_odom_recorder')
    names=('summit0','summit1','summit2')
    fields=('stage','robot','stamp_s','receipt_mono_s','frame_id','child_frame_id',
            'x_m','y_m','z_m','vx_body_m_s','vy_body_m_s','vz_body_m_s',
            'vx_world_m_s','vy_world_m_s','speed_xy_m_s','yaw_rate_rad_s')
    streams={};writers={};subscriptions=[]; counts={n:0 for n in names}
    try:
        for n in names:
            f=(args.output_dir/f'{n}_odom.csv').open('w',newline='',encoding='utf-8',buffering=1)
            streams[n]=f
            writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writers[n]=writer
        def callback(n,msg):
            v=msg.twist.twist.linear
            stamp=msg.header.stamp.sec+1e-9*msg.header.stamp.nanosec
            xyz=msg.pose.pose.position
            q=msg.pose.pose.orientation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            c,s=math.cos(yaw),math.sin(yaw)
            vx_world=c*v.x-s*v.y
            vy_world=s*v.x+c*v.y
            writers[n].writerow(dict(stage=args.stage,robot=n,stamp_s=f'{stamp:.9f}',
                receipt_mono_s=f'{time.monotonic():.9f}',frame_id=msg.header.frame_id,
                child_frame_id=msg.child_frame_id,x_m=xyz.x,y_m=xyz.y,z_m=xyz.z,
                vx_body_m_s=v.x,vy_body_m_s=v.y,vz_body_m_s=v.z,
                vx_world_m_s=vx_world,vy_world_m_s=vy_world,
                speed_xy_m_s=math.hypot(v.x,v.y),yaw_rate_rad_s=msg.twist.twist.angular.z))
            counts[n]+=1
        for n in names:
            subscriptions.append(node.create_subscription(Odometry,f'/{n}/odom',
                lambda msg,name=n: callback(name,msg),qos_profile_sensor_data))
        # A ready file signals creation of subscriptions. First odometry sample
        # is checked by the coordinator before navigation is authorized.
        args.ready_file.write_text('subscriptions_created\n',encoding='utf-8')
        print(f'[D8.11] Grabando odometría {args.stage} -> {args.output_dir}',flush=True)
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.2)
    except KeyboardInterrupt:
        pass
    finally:
        for f in streams.values():f.close()
        print('[D8.11] Muestras: '+str(counts),flush=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
