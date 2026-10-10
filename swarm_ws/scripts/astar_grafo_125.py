#!/usr/bin/env python3
"""A* para el grafo de VÉRTICES 1.25 m de SwarmBots (ROS 2 Jazzy).

Reutiliza grafo_cbs_125.py, SIN alterar la geometría ya validada.
Todos los waypoints publicados pertenecen al grafo (no se suavizan).
Solo se usa un conector seguro desde la odometría inicial al primer vértice.

Copia este archivo JUNTO A grafo_cbs_125.py en ~/swarm_ws/scripts/.

    python3 ~/swarm_ws/scripts/astar_grafo_125.py \\
      --ros-args -r __ns:=/summit0 -p goal_i:=19 -p goal_j:=12

La meta del ejemplo es el vértice (10.0, -8.75) m.
Alternativa: sin goal_i/goal_j, usar -p goal_x:=9.5 -p goal_y:=-8.5;
se elige y anuncia un vértice LIBRE alcanzable cercano a esa coordenada.

ADVERTENCIA: la ruta espacial no impone temporización CBS ni garantiza que
las desviaciones de control mantengan el margen físico en Gazebo.
"""
from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
import time
from itertools import count

import grafo_cbs_125 as geom

NodeID = geom.Node
Point = geom.Point


@dataclass
class Plan:
    requested_start: Point
    requested_goal: Point
    start_vertex: NodeID
    goal_vertex: NodeID
    route_vertices: list[NodeID]
    waypoints: list[Point]
    graph_cost_m: float
    expanded: int


class AStarGraph:
    """Grafo geométrico 8-conectado y A* sobre distancias euclídeas."""

    def __init__(self):
        self.vertices, self.edges, self.rejected, self.blocked = geom.make_graph()
        self.adjacency: dict[NodeID, list[tuple[NodeID, float]]] = {
            v: [] for v in self.vertices
        }
        for a, b in self.edges:
            cost = math.dist(geom.node_position(a), geom.node_position(b))
            self.adjacency[a].append((b, cost))
            self.adjacency[b].append((a, cost))
        for v in self.vertices:
            self.adjacency[v].sort(key=lambda entry: entry[0])

    def nearby_vertex(self, p: Point, radius_m: float = 2.5) -> NodeID:
        """Elige el vértice más próximo conectado con segmento seguro.

        Ni el inicio ni la meta se fuerzan a atravesar una columna.
        No se desplaza silenciosamente una posición físicamente bloqueada.
        """
        if not geom.free_position(p):
            raise ValueError(f'Punto {p} no transitable según el mapa inflado')
        candidates = sorted(
            self.vertices,
            key=lambda v: (math.dist(p, geom.node_position(v)), v),
        )
        for v in candidates:
            xy = geom.node_position(v)
            if math.dist(p, xy) > radius_m + 1e-9:
                break
            if geom.segment_clear(p, xy):
                return v
        raise ValueError(
            f'Ningún vértice conectado por segmento libre a {p} ' 
            f'en un radio de {radius_m:.2f} m')

    @staticmethod
    def heuristic(a: NodeID, b: NodeID) -> float:
        """Admisible y consistente: distancia euclídea en metros."""
        return math.dist(geom.node_position(a), geom.node_position(b))

    def astar(self, start: NodeID, goal: NodeID):
        if start not in self.vertices or goal not in self.vertices:
            raise ValueError('Inicio o meta no corresponde a vértice transitable')

        serial = count()
        open_heap = [(self.heuristic(start, goal), next(serial), start)]
        g_score: dict[NodeID, float] = {start: 0.0}
        parent: dict[NodeID, NodeID] = {}
        closed: set[NodeID] = set()
        expanded = 0
        while open_heap:
            f, _, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            if f > g_score[current] + self.heuristic(current, goal) + 1e-9:
                continue
            closed.add(current)
            expanded += 1
            if current == goal:
                route = [goal]
                while route[-1] in parent:
                    route.append(parent[route[-1]])
                route.reverse()
                return route, g_score[goal], expanded
            for neighbor, cost in self.adjacency[current]:
                if neighbor in closed:
                    continue
                next_g = g_score[current] + cost
                if next_g + 1e-10 < g_score.get(neighbor, math.inf):
                    g_score[neighbor] = next_g
                    parent[neighbor] = current
                    heapq.heappush(open_heap,
                                   (next_g + self.heuristic(neighbor, goal),
                                    next(serial), neighbor))
        raise RuntimeError('No existe una ruta en el grafo entre los vértices seleccionados')

    def plan_world(self, start: Point, goal: Point | None = None, *,
                   goal_vertex: NodeID | None = None,
                   snap_radius_m: float = 2.5,
                   waypoint_skip_m: float = 0.04) -> Plan:
        if (goal is None) == (goal_vertex is None):
            raise ValueError('Especifica exactamente goal (x,y) o goal_vertex (i,j)')
        start_vertex = self.nearby_vertex(start, snap_radius_m)
        if goal_vertex is not None:
            if goal_vertex not in self.vertices:
                raise ValueError(f'Vértice objetivo {goal_vertex} bloqueado o fuera del grafo')
            goal_real = geom.node_position(goal_vertex)
            target = goal_vertex
        else:
            goal_real = goal
            target = self.nearby_vertex(goal_real, snap_radius_m)
        route, cost, expanded = self.astar(start_vertex, target)
        # No suavizar ni saltar nodos intermedios: importante para futura CBS.
        points = [geom.node_position(v) for v in route]
        if points and math.dist(start, points[0]) <= waypoint_skip_m:
            points.pop(0)  # robot ya está en el vértice inicial
        if not points:
            # Robot ya está sobre el vértice/meta: no necesita desplazarse.
            points = []
        # Comprobar también el tramo físico desde la odom al primer waypoint.
        cursor = start
        for xy in points:
            if not geom.segment_clear(cursor, xy):
                raise RuntimeError(f'Tramo de trayectoria bloqueado: {cursor} -> {xy}')
            cursor = xy
        return Plan(start, goal_real, start_vertex, target, route, points,
                    cost, expanded)


def run_ros():
    """Publicador de ruta de un solo uso (ROS solo si se ejecuta este método)."""
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry, Path
    from rclpy.node import Node

    rclpy.init()
    node = Node('astar_grafo_125')
    node.declare_parameter('goal_i', -1)
    node.declare_parameter('goal_j', -1)
    node.declare_parameter('goal_x', 9.5)
    node.declare_parameter('goal_y', -8.5)
    node.declare_parameter('snap_radius_m', 2.5)
    node.declare_parameter('odom_timeout_s', 12.0)
    node.declare_parameter('follower_timeout_s', 10.0)
    try:
        data = {'odom': None}
        node.create_subscription(
            Odometry, 'odom', lambda msg: data.__setitem__('odom', msg), 10)
        publisher = node.create_publisher(Path, 'planned_path', 10)
        node.get_logger().info('Esperando /summit0/odom (tópico relativo: odom)...')
        deadline = time.monotonic() + float(node.get_parameter('odom_timeout_s').value)
        while rclpy.ok() and data['odom'] is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        odom = data['odom']
        if odom is None:
            raise RuntimeError('No llegó odometría: comprueba /summit0/odom y el namespace')
        frame = odom.header.frame_id
        if not frame:
            raise RuntimeError('odom.header.frame_id vacío; no es seguro publicar ruta')
        start = (odom.pose.pose.position.x, odom.pose.pose.position.y)

        graph = AStarGraph()
        gi, gj = int(node.get_parameter('goal_i').value), int(node.get_parameter('goal_j').value)
        snap_radius = float(node.get_parameter('snap_radius_m').value)
        if snap_radius <= 0 or not math.isfinite(snap_radius):
            raise ValueError('snap_radius_m debe ser positivo y finito')
        if gi >= 0 or gj >= 0:
            if gi < 0 or gj < 0:
                raise ValueError('Para meta por índice debes dar goal_i Y goal_j')
            plan = graph.plan_world(start, goal_vertex=(gi, gj), snap_radius_m=snap_radius)
        else:
            goal = (float(node.get_parameter('goal_x').value),
                    float(node.get_parameter('goal_y').value))
            plan = graph.plan_world(start, goal, snap_radius_m=snap_radius)

        node.get_logger().info(
            f'Grafo: {len(graph.vertices)} nodos transitables, '
            f'{len(graph.edges)} aristas; paso={geom.STEP_M:.2f} m')
        node.get_logger().info(
            f'Inicio odom={plan.requested_start}; vértice inicial={plan.start_vertex} '
            f'-> {geom.node_position(plan.start_vertex)}')
        node.get_logger().info(
            f'Meta solicitada={plan.requested_goal}; VÉRTICE final={plan.goal_vertex} '
            f'-> {geom.node_position(plan.goal_vertex)}')
        node.get_logger().info(
            f'A*: {len(plan.route_vertices)} vértices, costo en grafo='
            f'{plan.graph_cost_m:.2f} m; expandidos={plan.expanded}; '
            f'waypoints a enviar={len(plan.waypoints)}')
        node.get_logger().info('Secuencia (i,j): ' + str(plan.route_vertices))

        if not plan.waypoints:
            node.get_logger().info('El robot ya está en la meta discretizada; no se publica ruta.')
            return

        msg = Path()
        msg.header.frame_id = frame  # exactamente el frame de la odometría
        msg.header.stamp = node.get_clock().now().to_msg()
        for x, y in plan.waypoints:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        deadline = time.monotonic() + float(node.get_parameter('follower_timeout_s').value)
        while (rclpy.ok() and publisher.get_subscription_count() == 0
               and time.monotonic() < deadline):
            rclpy.spin_once(node, timeout_sec=0.1)
        if publisher.get_subscription_count() == 0:
            raise RuntimeError('No hay seguidor de planned_path; inicia path_follower primero')
        publisher.publish(msg)
        node.get_logger().info('Ruta publicada en planned_path. Observa el movimiento en Gazebo.')
        end = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    run_ros()
