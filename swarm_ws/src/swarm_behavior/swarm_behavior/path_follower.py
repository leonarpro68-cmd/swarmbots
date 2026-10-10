#!/usr/bin/env python3
"""Seguidor sencillo de waypoints para Summit XLS omnidireccional (ROS 2 Jazzy).

Ejecutar bajo el namespace del robot, p. ej. __ns:=/summit0.
Entradas relativas: odom (nav_msgs/Odometry), planned_path (nav_msgs/Path).
Salida relativa: cmd_vel (geometry_msgs/Twist).

Primera versión experimental: sin evitación de obstáculos, A* ni CBS.
Presupone que la odometría y la ruta están expresadas en el mismo frame global.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.declare_parameter('max_speed', 0.25)
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('waypoint_tolerance', 0.20)
        self.declare_parameter('odom_timeout', 0.5)
        self.declare_parameter('control_rate', 20.0)

        self.max_speed = float(self.get_parameter('max_speed').value)
        self.kp = float(self.get_parameter('kp').value)
        self.tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout').value)
        rate = float(self.get_parameter('control_rate').value)
        if min(self.max_speed, self.kp, self.tolerance, self.odom_timeout, rate) <= 0:
            raise ValueError('Todos los parámetros numéricos deben ser positivos')

        self.pose = None
        self.last_odom_time = None
        self.odom_frame = None
        self.path = []
        self.path_index = 0
        self.path_frame = None
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(Path, 'planned_path', self.on_path, 10)
        self.create_timer(1.0 / rate, self.control_step)
        self.get_logger().info('Esperando odometría y una ruta en planned_path')

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
        self.odom_frame = msg.header.frame_id
        self.last_odom_time = self.get_clock().now().nanoseconds / 1e9

    def on_path(self, msg):
        self.stop()
        # No podemos seguir rutas en otro frame sin TF. Aceptamos un frame
        # omitido únicamente para pruebas cuando el usuario conoce el marco.
        route_frame = msg.header.frame_id
        if route_frame and self.odom_frame and route_frame != self.odom_frame:
            self.path = []
            self.get_logger().error(
                f'Frame distinto: ruta={route_frame!r}, odom={self.odom_frame!r}. '
                'No se transformará la ruta; corrige el frame.')
            return
        if not msg.poses:
            self.path = []
            self.get_logger().info('Ruta vacía: robot detenido')
            return
        if not all(math.isfinite(p.pose.position.x) and math.isfinite(p.pose.position.y)
                   for p in msg.poses):
            self.path = []
            self.get_logger().error('Ruta inválida: coordenadas no finitas')
            return
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.path_index = 0
        self.path_frame = route_frame
        self.get_logger().info(f'Ruta recibida: {len(self.path)} puntos')

    def stop(self):
        self.cmd_pub.publish(Twist())

    def control_step(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if (self.pose is None or self.last_odom_time is None
                or now - self.last_odom_time > self.odom_timeout
                or not self.path):
            self.stop()
            return
        if self.path_frame and self.odom_frame and self.path_frame != self.odom_frame:
            self.path = []
            self.stop()
            self.get_logger().error('Cambió el frame de odometría; cancelando ruta')
            return

        x, y, yaw = self.pose
        while self.path_index < len(self.path):
            gx, gy = self.path[self.path_index]
            if math.hypot(gx - x, gy - y) > self.tolerance:
                break
            self.path_index += 1

        if self.path_index >= len(self.path):
            self.path = []
            self.stop()
            self.get_logger().info('Ruta completada; robot detenido')
            return

        gx, gy = self.path[self.path_index]
        ex, ey = gx - x, gy - y
        # P en coordenadas mundo; limitar módulo de la velocidad lineal.
        vx_world, vy_world = self.kp * ex, self.kp * ey
        speed = math.hypot(vx_world, vy_world)
        if speed > self.max_speed:
            scale = self.max_speed / speed
            vx_world *= scale
            vy_world *= scale

        # Transformación mundo -> cuerpo; mecanum permite vx y vy simultáneos.
        c, s = math.cos(yaw), math.sin(yaw)
        cmd = Twist()
        cmd.linear.x = c * vx_world + s * vy_world
        cmd.linear.y = -s * vx_world + c * vy_world
        cmd.angular.z = 0.0  # mantener el rumbo; no rotar por cada waypoint
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
