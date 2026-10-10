#!/usr/bin/env python3
"""Envia una ruta de prueba corta y relativa a la odometria de summit0.
Ejecutar solo con Gazebo y path_follower activos y en espacio libre.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


def main():
    rclpy.init()
    node = Node('prueba_ruta')
    state = {'odom': None}
    sub = node.create_subscription(Odometry, '/summit0/odom',
                                   lambda msg: state.__setitem__('odom', msg), 10)
    pub = node.create_publisher(Path, '/summit0/planned_path', 10)
    node.get_logger().info('Esperando /summit0/odom...')
    deadline = node.get_clock().now().nanoseconds / 1e9 + 10.0
    while rclpy.ok() and state['odom'] is None:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.get_clock().now().nanoseconds / 1e9 > deadline:
            raise RuntimeError('No se recibe odometria de /summit0/odom')

    odom = state['odom']
    x, y = odom.pose.pose.position.x, odom.pose.pose.position.y
    frame = odom.header.frame_id
    if not frame:
        raise RuntimeError('La odometria no declara frame_id; verificar antes de probar')
    # Polilinea relativa en el MARCO DE ODOMETRIA, no en el marco del robot.
    points = [(x + 0.6, y), (x + 0.6, y + 0.6), (x, y + 0.6), (x, y)]
    msg = Path()
    msg.header.frame_id = frame
    msg.header.stamp = node.get_clock().now().to_msg()
    for px, py in points:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = float(px)
        pose.pose.position.y = float(py)
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)
    node.get_logger().info(f'Frame={frame!r}; inicio=({x:.2f}, {y:.2f}); ruta={points}')
    # Mantener publicador vivo brevemente para el descubrimiento DDS.
    for _ in range(15):
        rclpy.spin_once(node, timeout_sec=0.1)
        if pub.get_subscription_count() > 0:
            break
    if pub.get_subscription_count() == 0:
        raise RuntimeError('No hay suscriptores de /summit0/planned_path')
    pub.publish(msg)
    node.get_logger().info('Ruta enviada. Observa el robot y el terminal del path_follower.')
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_subscription(sub)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
