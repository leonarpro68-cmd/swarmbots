#!/usr/bin/env python3
"""Prueba aislada tras 6B: ALINEAR -> GRASP ACK -> regreso al vertice.

No ejecuta CBS ni borra objetos. No tocar otros robots.
Debe haber finalizado 6B y estar corriendo el bridge /clock.
"""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.node import Node
from std_msgs.msg import Empty, Int8


def yaw_of(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


class Trial(Node):
    def __init__(self, robot):
        super().__init__('prueba_agarre_cbs')
        self.robot = robot
        self.pose = None
        self.frame = None
        self.align_ack = None
        self.grasp_ack = None
        self.create_subscription(Odometry, f'/{robot}/odom', self.on_odom, 10)
        self.create_subscription(Int8, f'/{robot}/visual_grasp/result', self.on_align, 10)
        self.create_subscription(Int8, f'/{robot}/gripper/result', self.on_grasp, 10)
        self.target_pub = self.create_publisher(PoseStamped, f'/{robot}/visual_grasp/target', 10)
        self.grasp_pub = self.create_publisher(Empty, f'/{robot}/gripper/grasp', 10)
        self.path_pub = self.create_publisher(RosPath, f'/{robot}/planned_path', 10)

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, yaw_of(q))
        self.frame = msg.header.frame_id

    def on_align(self, msg):
        self.align_ack = int(msg.data)

    def on_grasp(self, msg):
        self.grasp_ack = int(msg.data)

    def until(self, condition, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if condition():
                return True
        return False

    def send_target(self, x, y):
        msg = PoseStamped()
        msg.header.frame_id = self.frame or 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.w = 1.0
        self.target_pub.publish(msg)

    def send_return(self, x, y):
        path = RosPath()
        path.header.frame_id = self.frame or 'odom'
        path.header.stamp = self.get_clock().now().to_msg()
        point = PoseStamped()
        point.header = path.header
        point.pose.position.x = x
        point.pose.position.y = y
        point.pose.orientation.w = 1.0
        path.poses.append(point)
        self.path_pub.publish(path)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--robot', default='summit0', choices=['summit0', 'summit1', 'summit2'])
    ap.add_argument('--trash', required=True)
    ap.add_argument('--max-distance', type=float, default=1.3)
    ap.add_argument('--return-tolerance', type=float, default=0.12)
    args = ap.parse_args()
    data = json.loads(Path(args.scenario).read_text())
    options = {e['name']: e for e in data['elements'] if e['type'] == 'trash'}
    if args.trash not in options:
        ap.error(f'Basura no encontrada: {args.trash}; disponibles: {list(options)}')
    tx, ty = options[args.trash]['xy_m']
    rclpy.init()
    node = Trial(args.robot)
    try:
        if not node.until(lambda: node.pose is not None, 10):
            print('ERROR: sin odometria; no se ordeno movimiento')
            return 2
        x0, y0, yaw = node.pose
        dx, dy = tx - x0, ty - y0
        distance = math.hypot(dx, dy)
        forward = math.cos(yaw)*dx + math.sin(yaw)*dy - 0.35
        lateral = -math.sin(yaw)*dx + math.cos(yaw)*dy
        bearing = math.atan2(-lateral, max(forward, 1e-3))
        print(f'Robot={args.robot}; basura={args.trash}')
        print(f'Pose referencia: ({x0:.3f}, {y0:.3f}), yaw={math.degrees(yaw):.1f}°')
        print(f'Basura=({tx:.3f}, {ty:.3f}); distancia={distance:.3f} m; angulo camara={math.degrees(bearing):.1f}°')
        if forward <= 0 or abs(bearing) > 0.48*1.047 or distance > args.max_distance:
            print('PREFLIGHT NO APTO: basura fuera de alcance/campo visual estimado.')
            print('No se ha enviado ninguna orden. Se necesita preorientacion o aproximacion local.')
            return 3
        if node.target_pub.get_subscription_count() == 0:
            print('ERROR: visual_grasp/target sin suscriptor')
            return 4
        node.align_ack = None
        print('ALINEAR: enviando objetivo RGB-D...')
        node.send_target(tx, ty)
        if not node.until(lambda: node.align_ack is not None, 22):
            print('ERROR: sin respuesta de alineacion; no se ordena grasp')
            return 5
        if node.align_ack != 1:
            print('ALINEAR FALLIDO: resultado', node.align_ack)
            return 6
        print('ALINEAR CONFIRMADO')
        node.grasp_ack = None
        print('CERRANDO: solicitando grasp_manager...')
        node.grasp_pub.publish(Empty())
        if not node.until(lambda: node.grasp_ack is not None, 12):
            print('ERROR: sin ACK de grasp_manager; NO regresar')
            return 7
        if node.grasp_ack != 1:
            print('GRASP NO CONFIRMADO:', node.grasp_ack, 'NO regresar')
            return 8
        print('JOINT CONFIRMADO (ACK ROS); regresando al vertice de referencia')
        node.send_return(x0, y0)
        arrived = node.until(lambda: node.pose is not None and math.hypot(node.pose[0]-x0, node.pose[1]-y0) < args.return_tolerance, 40)
        print('REGRESO CONFIRMADO' if arrived else 'RETORNO NO CONFIRMADO, revisar Gazebo y odometria')
        return 0 if arrived else 9
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
