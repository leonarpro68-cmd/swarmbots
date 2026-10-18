#!/usr/bin/env python3
"""Prueba LOCAL de entrega: desde un vertice CBS cercano a su deposito.

No planifica el transporte global. Requiere basura sujetada previamente,
robot en un vertice de entrega (a <= max_start_distance del deposito),
ningun otro ejecutor de movimiento y deposito despejado.
No ejecuta ninguna accion sin --execute.
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
from std_msgs.msg import Empty, Float64, Int8


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


class DeliveryTrial(Node):
    def __init__(self, robot):
        super().__init__('prueba_entrega_local_cbs')
        self.pose = None
        self.frame = None
        self.odom_at = None
        self.turn_ack = None
        self.release_ack = None
        self.create_subscription(Odometry, f'/{robot}/odom', self.odom, 10)
        self.create_subscription(Int8, f'/{robot}/local_turn/result',
                                 lambda m: setattr(self, 'turn_ack', m.data), 10)
        self.create_subscription(Int8, f'/{robot}/gripper/release_result',
                                 lambda m: setattr(self, 'release_ack', m.data), 10)
        self.turn_pub = self.create_publisher(Float64, f'/{robot}/local_turn/target_yaw', 10)
        self.release_pub = self.create_publisher(Empty, f'/{robot}/gripper/release', 10)
        self.path_pub = self.create_publisher(RosPath, f'/{robot}/planned_path', 10)

    def odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw(msg.pose.pose.orientation))
        self.frame = msg.header.frame_id
        self.odom_at = time.monotonic()

    def wait(self, test, timeout):
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.1)
            if test():
                return True
        return False

    def route(self, x, y):
        msg = RosPath()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        msg.poses = [pose]
        self.path_pub.publish(msg)

    def halt(self):
        msg = RosPath()
        msg.header.frame_id = self.frame or ''
        self.path_pub.publish(msg)

    def at(self, x, y, tol):
        return self.pose is not None and self.odom_at is not None and \
            time.monotonic() - self.odom_at < 1.0 and \
            math.hypot(self.pose[0] - x, self.pose[1] - y) <= tol


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--robot', default='summit0', choices=['summit0','summit1','summit2'])
    parser.add_argument('--trash', default='trash_V1_verde')
    parser.add_argument('--deposit', default='deposit_verde')
    parser.add_argument('--execute', action='store_true', help='Autorizar movimientos y liberacion')
    parser.add_argument('--stop-distance', type=float, default=.80,
                        help='Distancia del centro del Summit al centro del deposito (m)')
    parser.add_argument('--max-start-distance', type=float, default=1.80)
    parser.add_argument('--arrival-tolerance', type=float, default=.12)
    args = parser.parse_args()
    data = json.loads(Path(args.scenario).read_text(encoding='utf-8'))
    items = {e['name']: e for e in data['elements']}
    trash = items.get(args.trash)
    depot = items.get(args.deposit)
    if not trash or trash['type'] != 'trash' or not depot or depot['type'] != 'deposit':
        parser.error('La basura o deposito no aparecen con el tipo correcto en el escenario')
    if trash['color'] != depot['color']:
        parser.error('La basura y el deposito no tienen el mismo color')
    if not .65 <= args.stop_distance <= 1.0:
        parser.error('--stop-distance debe estar entre 0.65 y 1.0 m')
    dx, dy = map(float, depot['xy_m'])
    rclpy.init()
    node = DeliveryTrial(args.robot)
    try:
        if not node.wait(lambda: node.odom_at and time.monotonic()-node.odom_at < 1, 12):
            print('ERROR: sin odometria reciente'); return 2
        x0, y0, angle0 = node.pose
        d0 = math.hypot(dx-x0, dy-y0)
        print(f'Robot={args.robot}; basura={args.trash}; deposito={args.deposit}')
        print(f'Vertice de regreso (pose real)=({x0:.3f}, {y0:.3f}) m')
        print(f'Distancia robot-deposito={d0:.3f} m')
        if not args.stop_distance+.10 <= d0 <= args.max_start_distance:
            print('PREFLIGHT NO APTO: robot no esta en un vertice CBS cercano al deposito. Sin ordenes.')
            return 3
        # El objeto esta aproximadamente 0.47 m por delante del centro del robot.
        # A .80 m del centro del deposito, el objeto queda ~.33 m del centro.
        heading = math.atan2(dy-y0, dx-x0)
        ux, uy = math.cos(heading), math.sin(heading)
        gx, gy = dx-args.stop_distance*ux, dy-args.stop_distance*uy
        print(f'Objetivo local=({gx:.3f}, {gy:.3f}) m; '
              f'giro={math.degrees(wrap(heading-angle0)):.1f} grados')
        if not args.execute:
            print('DRY RUN: no se ha publicado ninguna orden.')
            return 0
        for label, pub in [('local_turn',node.turn_pub),('planned_path',node.path_pub),
                           ('gripper/release',node.release_pub)]:
            if not node.wait(lambda p=pub: p.get_subscription_count() > 0, 5):
                print('ERROR: no hay suscriptor:',label); return 4
        # Esperar que la accion local de giro acabe antes de avanzar.
        node.turn_ack = None
        node.turn_pub.publish(Float64(data=heading))
        if not node.wait(lambda: node.turn_ack is not None, 20) or node.turn_ack != 1:
            print('PREORIENTAR FALLIDO, no se mueve a deposito:',node.turn_ack);return 5
        if abs(wrap(heading-node.pose[2])) > math.radians(8):
            print('PREORIENTAR fuera de tolerancia; no se aproxima.');return 6
        print('PREORIENTAR CONFIRMADO; aproximacion local...')
        node.route(gx,gy)
        if not node.wait(lambda: node.at(gx,gy,args.arrival_tolerance), 25):
            node.halt()
            print('ERROR: aproximacion no confirmada; NO se suelta'); return 7
        # Evitar soltar mientras el cuerpo esta aun desplazandose.
        node.halt()
        node.wait(lambda: False, .5)
        if not node.at(gx,gy,args.arrival_tolerance+.05):
            print('ERROR: perdio posicion de entrega, NO se suelta'); return 8
        node.release_ack = None
        print('ABRIENDO: esperando confirmacion de detach...')
        node.release_pub.publish(Empty())
        if not node.wait(lambda: node.release_ack is not None, 12):
            print('ERROR: sin ACK de release; permanecer en deposito y revisar Gazebo');return 9
        if node.release_ack != 1:
            print('ERROR: release no confirmado:',node.release_ack);return 10
        # grasp_manager puede publicar 1 aun sin carga: uso correcto requiere
        # verificar visualmente que habia una basura realmente sujeta.
        print('DETACH CONFIRMADO (ACK ROS). Regresando al vertice CBS...')
        node.route(x0,y0)
        if not node.wait(lambda: node.at(x0,y0,args.arrival_tolerance), 30):
            node.halt()
            print('ERROR: regreso no confirmado');return 11
        node.halt()
        print('REGRESO CONFIRMADO. Verificar que basura quedo en el deposito en Gazebo.')
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
