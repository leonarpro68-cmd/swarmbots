#!/usr/bin/env python3
"""Prueba AISLADA: vértice tras 6B -> preorientar -> RGB-D -> joint -> regreso.

Requiere Gazebo con clock bridge, visual_grasp, grasp_manager y
swarm_path_follower_grasp (con local_turn). NO modifica CBS ni borra basuras.
No iniciar junto con otros ejecutores de misión/control de ese robot.
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


def yaw_of(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def wrap(v):
    return math.atan2(math.sin(v), math.cos(v))


class Trial(Node):
    def __init__(self, robot):
        super().__init__('prueba_alinear_agarrar_cbs')
        self.pose = None
        self.frame = None
        self.last_odom = 0.0
        self.turn_ack = None
        self.align_ack = None
        self.grasp_ack = None
        self.create_subscription(Odometry, f'/{robot}/odom', self.on_odom, 10)
        self.create_subscription(Int8, f'/{robot}/local_turn/result', lambda m: setattr(self, 'turn_ack', int(m.data)), 10)
        self.create_subscription(Int8, f'/{robot}/visual_grasp/result', lambda m: setattr(self, 'align_ack', int(m.data)), 10)
        self.create_subscription(Int8, f'/{robot}/gripper/result', lambda m: setattr(self, 'grasp_ack', int(m.data)), 10)
        self.turn_pub = self.create_publisher(Float64, f'/{robot}/local_turn/target_yaw', 10)
        self.target_pub = self.create_publisher(PoseStamped, f'/{robot}/visual_grasp/target', 10)
        self.grasp_pub = self.create_publisher(Empty, f'/{robot}/gripper/grasp', 10)
        self.path_pub = self.create_publisher(RosPath, f'/{robot}/planned_path', 10)

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (float(p.x), float(p.y), yaw_of(q))
        self.frame = msg.header.frame_id
        self.last_odom = time.monotonic()

    def wait(self, fn, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.10)
            if fn():
                return True
        return False

    def target(self, x, y):
        msg = PoseStamped()
        msg.header.frame_id = self.frame or 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        msg.pose.orientation.w = 1.0
        self.target_pub.publish(msg)

    def return_path(self, x, y):
        msg = RosPath()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        p = PoseStamped()
        p.header = msg.header
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.w = 1.0
        msg.poses = [p]
        self.path_pub.publish(msg)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--robot', default='summit0', choices=('summit0','summit1','summit2'))
    ap.add_argument('--trash', default='trash_V1_verde')
    ap.add_argument('--max-distance', type=float, default=2.2,
                    help='Distancia máxima inicial permitida, en metros (default 2.2)')
    ap.add_argument('--return-tolerance', type=float, default=0.12)
    ap.add_argument('--visual-timeout', type=float, default=22.)
    ap.add_argument('--dry-run', action='store_true', help='Solo comprobar pose y geometría; sin enviar órdenes')
    args = ap.parse_args()
    data = json.loads(Path(args.scenario).read_text(encoding='utf-8'))
    targets = [e for e in data['elements'] if e['type'] == 'trash' and e['name'] == args.trash]
    if len(targets) != 1:
        ap.error('Basura ausente del JSON: ' + args.trash)
    tx, ty = map(float, targets[0]['xy_m'])
    rclpy.init()
    n = Trial(args.robot)
    try:
        if not n.wait(lambda: n.pose is not None, 12):
            print('ERROR: no hay odometría'); return 2
        x0, y0, yaw0 = n.pose
        dist = math.hypot(tx-x0, ty-y0)
        target_yaw = math.atan2(ty-y0, tx-x0)
        print(f'Robot: {args.robot}; basura: {args.trash}')
        print(f'Vertice de referencia (pose real): ({x0:.3f}, {y0:.3f}) m')
        print(f'Distancia inicial: {dist:.3f} m; giro requerido: {math.degrees(wrap(target_yaw-yaw0)):.1f} grados')
        if dist > args.max_distance or dist < 0.55:
            print('PREFLIGHT NO APTO: distancia fuera del rango permitido. Sin órdenes.')
            return 3
        if args.dry_run:
            print('DRY RUN correcto; no se enviaron órdenes.'); return 0
        for label, pub in [('local_turn', n.turn_pub), ('visual_grasp/target', n.target_pub),
                           ('gripper/grasp', n.grasp_pub), ('planned_path', n.path_pub)]:
            if not n.wait(lambda p=pub: p.get_subscription_count() > 0, 5):
                print(f'ERROR: no existe suscriptor para {label}; no se inicia'); return 4
        n.turn_ack = None
        print('PREORIENTAR: solicitando giro local...')
        n.turn_pub.publish(Float64(data=target_yaw))
        if not n.wait(lambda: n.turn_ack is not None, 20) or n.turn_ack != 1:
            print('PREORIENTAR FALLIDO; no se alinea ni agarra. ACK=', n.turn_ack); return 5
        if not n.wait(lambda: n.pose is not None and time.monotonic()-n.last_odom < 1, 3):
            print('ERROR: odometria vencida tras giro'); return 6
        x, y, yaw = n.pose
        dx, dy = tx-x, ty-y
        forward = math.cos(yaw)*dx + math.sin(yaw)*dy - 0.35
        lateral = -math.sin(yaw)*dx + math.cos(yaw)*dy
        bearing = math.atan2(-lateral, max(forward, 1e-3))
        print(f'PREORIENTAR CONFIRMADO: error={math.degrees(wrap(target_yaw-yaw)):.1f} grados; bearing camara={math.degrees(bearing):.1f} grados')
        if forward <= 0 or abs(bearing) >= .48*1.047:
            print('PREFLIGHT VISUAL NO APTO: objetivo fuera de cámara. No se inicia agarre.'); return 7
        n.align_ack = None
        print('ALINEAR: activando RGB-D. Observa Gazebo...')
        n.target(tx, ty)
        if not n.wait(lambda: n.align_ack is not None, args.visual_timeout):
            print('ERROR: timeout sin ACK de visual_grasp. NO se envía grasp.'); return 8
        if n.align_ack != 1:
            print('ALINEAR FALLIDO: ACK=', n.align_ack, '; no se agarra'); return 9
        print('ALINEAR CONFIRMADO')
        # Dejar que el seguidor procese visual_active=False antes de un eventual retorno.
        n.wait(lambda: False, .35)
        n.grasp_ack = None
        print('CERRANDO: orden a grasp_manager...')
        n.grasp_pub.publish(Empty())
        if not n.wait(lambda: n.grasp_ack is not None, 15):
            print('ERROR: sin resultado grasp; no se ordena retorno'); return 10
        if n.grasp_ack != 1:
            print('AGARRE NO CONFIRMADO: ACK=', n.grasp_ack, '; sin retorno'); return 11
        print('JOINT CONFIRMADO (ACK ROS). Ordenando regreso al vertice...')
        n.return_path(x0, y0)
        if n.wait(lambda: n.pose is not None and math.hypot(n.pose[0]-x0, n.pose[1]-y0) <= args.return_tolerance, 45):
            print('REGRESO CONFIRMADO: objeto sujeto, verificar en Gazebo')
            return 0
        print('REGRESO NO CONFIRMADO: revisar Gazebo, odometría y logs')
        return 12
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
