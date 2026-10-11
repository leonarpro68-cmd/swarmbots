#!/usr/bin/env python3
"""Prueba ROS 2: tres Summit, 2 pasos de vértice sincronizados.

NO usa CBS todavía: verifica la barrera antes de integrar rutas CBS.
NO modifica path_follower.py: le publica un nav_msgs/Path de UN waypoint.
El arranque se alinea primero a nodos del grafo de 1.25 m.

Colocar este archivo al lado de grafo_cbs_125.py en ~/swarm_ws/scripts.
Ejecutar: python3 ~/swarm_ws/scripts/cbs_step_executor_demo.py

Advertencia: sin comprobación de proximidad entre robots. Supervisar en Gazebo.
"""
from __future__ import annotations
import math
import time

import grafo_cbs_125 as geom


def select_alignment(graph_nodes, xy, radius=2.5):
    if not geom.free_position(xy):
        raise ValueError(f'Inicio fuera de zona libre con margen: {xy}')
    for v in sorted(graph_nodes, key=lambda p: (math.dist(xy, geom.node_position(p)), p)):
        dest = geom.node_position(v)
        if math.dist(xy, dest) > radius:
            break
        if geom.segment_clear(xy, dest):
            return v
    raise ValueError(f'No se encontró vértice de alineación seguro para {xy}')


def two_step_route(start, neighbors):
    """Dos aristas del grafo, preferentemente rectas hacia +Y.

    Evita volver al nodo de inicio. No comprueba conflictos multiagente;
    esa es precisamente la siguiente etapa (CBS).
    """
    if start not in neighbors:
        raise ValueError(f'Vértice inicial fuera del grafo: {start}')
    dirs = ((0, 1), (1, 0), (0, -1), (-1, 0),
            (1, 1), (-1, 1), (1, -1), (-1, -1))
    for di, dj in dirs:
        b = (start[0] + di, start[1] + dj)
        c = (start[0] + 2*di, start[1] + 2*dj)
        if b in neighbors[start] and c in neighbors[b]:
            return start, b, c
    raise ValueError(f'No hay dos pasos rectos desde {start}')


class SyncDemo:
    def __init__(self):
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry, Path
        from rclpy.node import Node

        self.rclpy = rclpy
        self.PoseStamped = PoseStamped
        self.Path = Path
        self.node = Node('cbs_step_executor_demo')
        n = self.node
        n.declare_parameter('arrival_tolerance', 0.12)
        n.declare_parameter('arrival_speed', 0.10)
        n.declare_parameter('hold_seconds', 0.6)
        n.declare_parameter('step_timeout', 45.0)
        n.declare_parameter('odom_timeout', 1.0)
        self.tol = float(n.get_parameter('arrival_tolerance').value)
        self.speed_limit = float(n.get_parameter('arrival_speed').value)
        self.hold = float(n.get_parameter('hold_seconds').value)
        self.timeout = float(n.get_parameter('step_timeout').value)
        self.odom_timeout = float(n.get_parameter('odom_timeout').value)
        self.agents = ('summit0','summit1','summit2')
        self.odom = {}
        self.pubs = {a: n.create_publisher(Path, f'/{a}/planned_path', 10) for a in self.agents}
        self.subs = []
        for a in self.agents:
            self.subs.append(n.create_subscription(
                Odometry, f'/{a}/odom', lambda msg, name=a: self.on_odom(name,msg),10))
        self.vertices, edges, _, _ = geom.make_graph()
        self.neighbors = {v:set() for v in self.vertices}
        for a,b in edges:
            self.neighbors[a].add(b)
            self.neighbors[b].add(a)
        self.phase = 'WAIT_ODOM'
        self.target = {}
        self.started = time.monotonic()
        self.holding_since = {a:None for a in self.agents}
        self.routes = {}
        self.step_idx = 0
        n.create_timer(0.1, self.tick)
        n.get_logger().info('Esperando odometría de summit0, summit1, summit2')

    def on_odom(self, agent, msg):
        p=msg.pose.pose.position; v=msg.twist.twist.linear
        self.odom[agent] = (p.x,p.y,math.hypot(v.x,v.y),time.monotonic(),msg.header.frame_id)

    def fresh(self):
        now=time.monotonic()
        return all(a in self.odom and now-self.odom[a][3]<self.odom_timeout
                   for a in self.agents)

    def publish_goal(self, agent, xy):
        path=self.Path()
        path.header.frame_id=self.odom[agent][4]
        path.header.stamp=self.node.get_clock().now().to_msg()
        item=self.PoseStamped()
        item.header=path.header
        item.pose.position.x=float(xy[0]); item.pose.position.y=float(xy[1])
        item.pose.orientation.w=1.0
        path.poses=[item]
        self.pubs[agent].publish(path)

    def stop_all(self):
        for a in self.agents:
            path=self.Path()
            if a in self.odom:
                path.header.frame_id=self.odom[a][4]
            self.pubs[a].publish(path)  # path_follower cancela y publica Twist cero

    def start_phase(self, name, targets):
        self.target=targets
        self.phase=name
        self.started=time.monotonic()
        self.holding_since={a:None for a in self.agents}
        for a in self.agents:
            self.publish_goal(a,targets[a])
            self.node.get_logger().info(f'{name}: {a} -> {targets[a]}')

    def fail(self, reason):
        self.node.get_logger().error(reason)
        self.stop_all()
        self.phase='ABORT'

    def tick(self):
        if self.phase in ('DONE','ABORT'):
            return
        if not self.fresh():
            if self.phase != 'WAIT_ODOM':
                self.fail('Odometría ausente o vencida: cancelando todas las rutas')
            return
        if self.phase == 'WAIT_ODOM':
            try:
                starts={a:select_alignment(self.vertices,self.odom[a][:2]) for a in self.agents}
                self.routes={a:two_step_route(starts[a],self.neighbors) for a in self.agents}
            except ValueError as ex:
                self.fail(str(ex));return
            for a in self.agents:
                self.node.get_logger().info(f'{a} plan de prueba: {self.routes[a]}')
            self.start_phase('ALIGN',{a:geom.node_position(starts[a]) for a in self.agents})
            return
        now=time.monotonic()
        if now-self.started > self.timeout:
            self.fail(f'Tiempo agotado en {self.phase}');return
        for a in self.agents:
            x,y,speed,_,_=self.odom[a]
            reached=math.hypot(x-self.target[a][0], y-self.target[a][1])<=self.tol and speed<=self.speed_limit
            if reached:
                if self.holding_since[a] is None:
                    self.holding_since[a]=now
            else:
                self.holding_since[a]=None
        if not all(self.holding_since[a] is not None and
                   now-self.holding_since[a]>=self.hold for a in self.agents):
            return
        self.node.get_logger().info(f'BARRERA superada: {self.phase}, llegaron los 3')
        if self.phase=='ALIGN':
            self.step_idx=1
        elif self.phase=='STEP_1':
            self.step_idx=2
        else:
            self.stop_all()
            self.phase='DONE'
            self.node.get_logger().info('PRUEBA COMPLETADA: 2 pasos sincronizados')
            return
        self.start_phase(f'STEP_{self.step_idx}', {
            a:geom.node_position(self.routes[a][self.step_idx]) for a in self.agents})


def main():
    import rclpy
    rclpy.init()
    demo=SyncDemo()
    try:
        rclpy.spin(demo.node)
    except KeyboardInterrupt:
        pass
    finally:
        demo.stop_all()
        demo.node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
