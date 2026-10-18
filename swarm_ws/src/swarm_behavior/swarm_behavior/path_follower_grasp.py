#!/usr/bin/env python3
"""Seguidor CBS experimental: navegación, control RGB-D y preorientación local.

Entrada adicional: local_turn/target_yaw (std_msgs/Float64), radianes EN MARCO
DE ODOMETRÍA. Resultado: local_turn/result (std_msgs/Int8): 1 éxito, -1 fallo.
Solo este nodo publica cmd_vel. No modificar el seguidor CBS estable.
"""
from __future__ import annotations
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64, Int8

from .path_follower import PathFollower


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class GraspPathFollower(PathFollower):
    def __init__(self):
        super().__init__()
        self.declare_parameter('visual_cmd_timeout', 0.35)
        self.declare_parameter('turn_kp', 1.8)
        self.declare_parameter('turn_max_speed', 0.35)
        self.declare_parameter('turn_tolerance_rad', 0.055)
        self.declare_parameter('turn_timeout_s', 15.0)
        self.visual_timeout = float(self.get_parameter('visual_cmd_timeout').value)
        self.turn_kp = float(self.get_parameter('turn_kp').value)
        self.turn_max_speed = float(self.get_parameter('turn_max_speed').value)
        self.turn_tolerance = float(self.get_parameter('turn_tolerance_rad').value)
        self.turn_timeout = float(self.get_parameter('turn_timeout_s').value)
        if min(self.visual_timeout, self.turn_kp, self.turn_max_speed,
               self.turn_tolerance, self.turn_timeout) <= 0:
            raise ValueError('Parámetros de control visual y giro deben ser positivos')
        self.visual_active = False
        self.visual_cmd = Twist()
        self.visual_last_time = None
        self.turn_target = None
        self.turn_started = None
        self.turn_hold_started = None
        self.turn_result_pub = self.create_publisher(Int8, 'local_turn/result', 10)
        self.create_subscription(Float64, 'local_turn/target_yaw', self.on_turn_target, 10)
        self.create_subscription(Bool, 'visual_grasp/active', self.on_visual_active, 10)
        self.create_subscription(Twist, 'visual_grasp/cmd_vel', self.on_visual_cmd, 10)
        self.get_logger().info('Seguidor experimental: RGB-D y preorientación local habilitados')

    def on_turn_target(self, msg):
        if self.visual_active or self.turn_target is not None or not math.isfinite(msg.data):
            self.get_logger().warn('Giro rechazado: control ocupado u objetivo inválido')
            self.turn_result_pub.publish(Int8(data=-1))
            return
        if self.pose is None:
            self.get_logger().warn('Giro rechazado: sin odometría')
            self.turn_result_pub.publish(Int8(data=-1))
            return
        self.path = []  # solo giro en posición, nunca reanudar ruta antigua
        self.path_index = 0
        self.turn_target = wrap_angle(float(msg.data))
        self.turn_started = time.monotonic()
        self.turn_hold_started = None
        self.stop()
        self.get_logger().info(f'PREORIENTAR: yaw objetivo={math.degrees(self.turn_target):.1f}°')

    def on_visual_active(self, msg):
        active = bool(msg.data)
        if active and not self.visual_active:
            if self.turn_target is not None:
                self.turn_target = None
                self.turn_result_pub.publish(Int8(data=-1))
            self.path = []
            self.path_index = 0
            self.visual_cmd = Twist()
            self.visual_last_time = None
            self.stop()
            self.get_logger().info('CONTROL RGB-D: ruta suspendida')
        elif not active and self.visual_active:
            self.visual_cmd = Twist()
            self.visual_last_time = None
            self.stop()
            self.get_logger().info('CONTROL RGB-D terminado: esperando nueva ruta')
        self.visual_active = active

    def on_path(self, msg):
        if self.visual_active or self.turn_target is not None:
            self.get_logger().warn('Ruta recibida mientras control local está activo; ignorada')
            return
        super().on_path(msg)

    def on_visual_cmd(self, msg):
        if self.visual_active:
            self.visual_cmd = msg
            self.visual_last_time = self.get_clock().now().nanoseconds / 1e9

    def control_step(self):
        if self.visual_active:
            now = self.get_clock().now().nanoseconds / 1e9
            if (self.visual_last_time is not None
                    and 0 <= now - self.visual_last_time <= self.visual_timeout):
                self.cmd_pub.publish(self.visual_cmd)
            else:
                self.stop()
            return
        if self.turn_target is not None:
            now = time.monotonic()
            current_sim_time = self.get_clock().now().nanoseconds / 1e9
            if (self.pose is None or self.last_odom_time is None
                    or current_sim_time - self.last_odom_time > self.odom_timeout
                    or now - self.turn_started > self.turn_timeout):
                self.stop()
                self.turn_target = None
                self.turn_result_pub.publish(Int8(data=-1))
                self.get_logger().error('PREORIENTAR FALLIDO: odometría vencida o timeout')
                return
            error = wrap_angle(self.turn_target - self.pose[2])
            if abs(error) <= self.turn_tolerance:
                self.stop()
                if self.turn_hold_started is None:
                    self.turn_hold_started = now
                elif now - self.turn_hold_started >= 0.4:
                    self.turn_target = None
                    self.turn_result_pub.publish(Int8(data=1))
                    self.get_logger().info('PREORIENTAR CONFIRMADO')
                return
            self.turn_hold_started = None
            cmd = Twist()
            cmd.angular.z = max(-self.turn_max_speed, min(self.turn_max_speed,
                                                           self.turn_kp * error))
            self.cmd_pub.publish(cmd)
            return
        super().control_step()


def main(args=None):
    rclpy.init(args=args)
    node = GraspPathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
