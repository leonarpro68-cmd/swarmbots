#!/usr/bin/env python3
"""Prueba segura PREORIENTAR después de 6B, SIN aproximar ni agarrar basura."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Int8


def angle(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def delta(a):
    return math.atan2(math.sin(a), math.cos(a))


class Probe(Node):
    def __init__(self, robot):
        super().__init__('prueba_preorientacion_cbs')
        self.pose = None
        self.ack = None
        self.create_subscription(Odometry, f'/{robot}/odom', self.odom, 10)
        self.create_subscription(Int8, f'/{robot}/local_turn/result', self.result, 10)
        self.pub = self.create_publisher(Float64, f'/{robot}/local_turn/target_yaw', 10)

    def odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = p.x, p.y, angle(q)

    def result(self, msg):
        self.ack = msg.data

    def wait_for(self, predicate, timeout):
        until = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        return False


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scenario', required=True)
    p.add_argument('--robot', default='summit0', choices=['summit0','summit1','summit2'])
    p.add_argument('--trash', default='trash_V1_verde')
    args = p.parse_args()
    info = json.loads(Path(args.scenario).read_text())
    matches = [e for e in info['elements'] if e['type']=='trash' and e['name']==args.trash]
    if len(matches)!=1:
        p.error('No existe esa basura en el escenario')
    tx,ty = matches[0]['xy_m']
    rclpy.init()
    n = Probe(args.robot)
    try:
        if not n.wait_for(lambda: n.pose is not None, 10):
            print('ERROR: sin odometría'); return 2
        if not n.wait_for(lambda: n.pub.get_subscription_count()>0, 5):
            print('ERROR: el seguidor nuevo no está suscrito a local_turn/target_yaw'); return 3
        x,y,yaw = n.pose
        goal = math.atan2(ty-y,tx-x)
        print(f'Robot {args.robot}: pose=({x:.3f}, {y:.3f}), yaw={math.degrees(yaw):.1f}°')
        print(f'Basura {args.trash}: ({tx:.3f}, {ty:.3f})')
        print(f'Giro previsto={math.degrees(delta(goal-yaw)):.1f}°')
        n.pub.publish(Float64(data=goal))
        if not n.wait_for(lambda: n.ack is not None, 18):
            print('ERROR: sin confirmación del giro'); return 4
        if n.ack != 1:
            print('PREORIENTAR FALLIDO: ACK=',n.ack); return 5
        n.wait_for(lambda: False, .3)
        x1,y1,yaw1 = n.pose
        print(f'PREORIENTAR CONFIRMADO: yaw={math.degrees(yaw1):.1f}°, error={math.degrees(delta(goal-yaw1)):.1f}°')
        print(f'Desplazamiento del centro={math.hypot(x1-x,y1-y):.3f} m')
        return 0
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':
    raise SystemExit(main())
