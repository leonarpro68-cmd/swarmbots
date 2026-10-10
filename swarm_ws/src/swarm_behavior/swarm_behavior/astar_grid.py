#!/usr/bin/env python3
"""A* 8-conectado para un Summit XLS en el almacén de SwarmBots.

Etapa 1 (UN robot, SIN CBS): mapa estático medido manualmente, inflación de
obstáculos y publicación de nav_msgs/Path hacia `path_follower.py`.

Ejemplo (en el contenedor, con Gazebo y path_follower ya ejecutándose):

    python3 ~/swarm_ws/src/swarm_behavior/swarm_behavior/astar_grid.py \
      --ros-args -r __ns:=/summit0 -p goal_x:=9.5 -p goal_y:=-8.5

No publicar `cmd_vel` desde aquí: el único controlador es path_follower.

IMPORTANTE: el almacén y las columnas se aproximan usando medidas manuales.
El inflado y el planificador NO sustituyen la verificación de colisiones reales
ni constituyen una garantía de seguridad ante errores de seguimiento.
"""

from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass
from itertools import count
from typing import Optional

# --------------------- Modelo geométrico medido -------------------------
X_MIN, X_MAX = -14.0, 14.0
Y_MIN, Y_MAX = -24.0, 24.0
RESOLUTION = 0.25

# Envolvente circular conservadora del chasis + ruedas.
ROBOT_LENGTH = 0.7190
ROBOT_WIDTH = 0.5786
ROBOT_RADIUS = 0.5 * math.hypot(ROBOT_LENGTH, ROBOT_WIDTH)
SAFETY_MARGIN = 0.30
CLEARANCE = ROBOT_RADIUS + SAFETY_MARGIN  # ~0.561 m desde el centro a paredes

COLUMN_DIAMETER = 0.35
COLUMN_RADIUS = COLUMN_DIAMETER / 2.0
COLUMN_CLEARANCE = COLUMN_RADIUS + CLEARANCE  # ~0.736 m desde centro columna

COLUMNS = (
    (7.4, 7.5),
    (-7.4, -7.5),
    (-7.4, 7.5),
    (-7.4, -15.0),
    (7.4, 0.0),
    (7.4, -7.5),
    (-7.4, 15.0),
    (7.4, -15.0),
    (7.4, 15.0),
    (-7.4, 0.0),
)

# Los rectángulos pintados en el suelo NO se marcan como obstáculos.
Cell = tuple[int, int]
Point = tuple[float, float]


@dataclass
class PlanResult:
    # La primera posición es el comienzo de la ruta (en el grid).
    cells: list[Cell]
    cost_m: float
    expanded: int


class WarehouseGrid:
    """Grid de centros de celda y comprobación geométrica continua."""

    def __init__(self):
        self.nx = round((X_MAX - X_MIN) / RESOLUTION)
        self.ny = round((Y_MAX - Y_MIN) / RESOLUTION)
        if (not math.isclose(self.nx * RESOLUTION, X_MAX - X_MIN)
                or not math.isclose(self.ny * RESOLUTION, Y_MAX - Y_MIN)):
            raise ValueError('Las dimensiones del mundo no son múltiplos de la resolución')
        self.free = [
            [self.is_free_point(*self.to_world((ix, iy)))
             for ix in range(self.nx)]
            for iy in range(self.ny)
        ]

    def in_grid(self, c: Cell) -> bool:
        return 0 <= c[0] < self.nx and 0 <= c[1] < self.ny

    def to_cell(self, p: Point) -> Cell:
        x, y = p
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError('Coordenadas no finitas')
        c = (math.floor((x - X_MIN) / RESOLUTION),
             math.floor((y - Y_MIN) / RESOLUTION))
        if not self.in_grid(c):
            raise ValueError(f'Posición {p} fuera del grid')
        return c

    def to_world(self, c: Cell) -> Point:
        return (X_MIN + (c[0] + 0.5) * RESOLUTION,
                Y_MIN + (c[1] + 0.5) * RESOLUTION)

    def is_free_cell(self, c: Cell) -> bool:
        return self.in_grid(c) and self.free[c[1]][c[0]]

    def is_free_point(self, x: float, y: float) -> bool:
        if not all(math.isfinite(v) for v in (x, y)):
            return False
        # El centro del robot debe mantenerse alejado de las paredes.
        if not (X_MIN + CLEARANCE <= x <= X_MAX - CLEARANCE
                and Y_MIN + CLEARANCE <= y <= Y_MAX - CLEARANCE):
            return False
        return all((x-cx)**2 + (y-cy)**2 >= COLUMN_CLEARANCE**2
                   for cx, cy in COLUMNS)

    def is_clear_segment(self, a: Point, b: Point) -> bool:
        """Comprueba todo el segmento de la trayectoria del centro del robot.

        La comprobación por segmentos evita el fallo de mirar sólo si las
        celdas inicial/final son libres (especialmente con diagonales).
        """
        if not (self.is_free_point(*a) and self.is_free_point(*b)):
            return False
        dx, dy = b[0] - a[0], b[1] - a[1]
        denom = dx*dx + dy*dy
        for cx, cy in COLUMNS:
            if denom == 0.0:
                nearest_x, nearest_y = a
            else:
                u = max(0.0, min(1.0, ((cx-a[0])*dx + (cy-a[1])*dy) / denom))
                nearest_x, nearest_y = a[0] + u*dx, a[1] + u*dy
            if (nearest_x-cx)**2 + (nearest_y-cy)**2 < COLUMN_CLEARANCE**2:
                return False
        # El dominio de centros admisible es un rectángulo convexo; si ambos
        # extremos están dentro, todo el segmento también lo está.
        return True

    def neighbors(self, node: Cell):
        ix, iy = node
        a = self.to_world(node)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == dy == 0:
                    continue
                nxt = ix + dx, iy + dy
                if not self.is_free_cell(nxt):
                    continue
                # No atravesar esquinas de celdas ocupadas.
                if dx and dy and (not self.is_free_cell((ix+dx, iy))
                                  or not self.is_free_cell((ix, iy+dy))):
                    continue
                if not self.is_clear_segment(a, self.to_world(nxt)):
                    continue
                yield nxt, RESOLUTION * (math.sqrt(2) if dx and dy else 1.0)

    @staticmethod
    def heuristic(a: Cell, b: Cell) -> float:
        """Distancia octil: heurística admisible para grid 8-conectado."""
        dx, dy = abs(a[0]-b[0]), abs(a[1]-b[1])
        return RESOLUTION * (max(dx, dy) + (math.sqrt(2)-1) * min(dx, dy))

    def astar(self, start: Point, goal: Point) -> PlanResult:
        for label, p in [('Inicio', start), ('Meta', goal)]:
            if not self.is_free_point(*p):
                raise ValueError(f'{label} {p} no está libre, incluyendo el inflado')
        s, t = self.to_cell(start), self.to_cell(goal)
        for label, c in [('Inicio', s), ('Meta', t)]:
            if not self.is_free_cell(c):
                raise ValueError(
                    f'{label} cae en celda bloqueada {c}; cambiar la posición '
                    'en vez de desplazarla automáticamente')
        # Los segmentos entre la pose exacta y el centro de su celda deben
        # ser libres. No usamos el centro como excusa para cruzar colisiones.
        if not self.is_clear_segment(start, self.to_world(s)):
            raise ValueError('Inicio -> centro de su celda no tiene espacio libre')
        if not self.is_clear_segment(self.to_world(t), goal):
            raise ValueError('Centro de celda final -> meta no está libre')

        serial = count()
        opens = [(self.heuristic(s, t), next(serial), s)]
        g = {s: 0.0}
        parent: dict[Cell, Cell] = {}
        closed: set[Cell] = set()
        expanded = 0
        while opens:
            f, _, node = heapq.heappop(opens)
            if node in closed:
                continue
            # Ignorar entradas antiguas del heap por mejorías de g-score.
            if f > g[node] + self.heuristic(node, t) + 1e-9:
                continue
            closed.add(node)
            expanded += 1
            if node == t:
                cells = [node]
                while node in parent:
                    node = parent[node]
                    cells.append(node)
                cells.reverse()
                return PlanResult(cells, g[t], expanded)
            for neighbor, move_cost in self.neighbors(node):
                if neighbor in closed:
                    continue
                candidate = g[node] + move_cost
                if candidate + 1e-10 < g.get(neighbor, math.inf):
                    g[neighbor] = candidate
                    parent[neighbor] = node
                    heapq.heappush(opens,
                                   (candidate + self.heuristic(neighbor, t),
                                    next(serial), neighbor))
        raise RuntimeError(f'A* no encontró ruta desde {start} a {goal}')

    def simplify(self, start: Point, goal: Point, cells: list[Cell]) -> list[Point]:
        """Quita waypoints redundantes sin atravesar obstáculos inflados.

        Devuelve puntos que debe seguir el robot; no incluye su pose actual.
        La optimalidad en grafo corresponde a `astar`; el suavizado produce
        una polilínea continua de ejecución, no un plan CBS temporizado.
        """
        if not cells:
            raise ValueError('Ruta A* vacía')
        raw = [start] + [self.to_world(c) for c in cells] + [goal]
        kept = [start]
        i = 0
        while i < len(raw)-1:
            j = len(raw)-1
            while j > i and not self.is_clear_segment(raw[i], raw[j]):
                j -= 1
            if j == i:
                raise RuntimeError('No se pudo convertir el plan a segmentos libres')
            kept.append(raw[j])
            i = j
        return kept[1:]


def run_ros():
    # Importaciones tardías: posibilita probar el algoritmo de A* sin ROS 2.
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry, Path
    from rclpy.node import Node

    rclpy.init()
    node = Node('astar_grid')
    node.declare_parameter('goal_x', 9.5)
    node.declare_parameter('goal_y', -8.5)
    node.declare_parameter('odom_timeout', 12.0)  # tiempo máximo para recibir 1 odom
    node.declare_parameter('follower_timeout', 10.0)  # esperar subscriber DDS
    node.declare_parameter('simplify_path', True)
    try:
        goal = (float(node.get_parameter('goal_x').value),
                float(node.get_parameter('goal_y').value))
        sensor_state = {'msg': None}
        node.create_subscription(
            Odometry, 'odom', lambda msg: sensor_state.__setitem__('msg', msg), 10)
        pub = node.create_publisher(Path, 'planned_path', 10)
        node.get_logger().info('Esperando odometría, para calcular A*...')
        deadline = time.monotonic() + float(node.get_parameter('odom_timeout').value)
        while rclpy.ok() and sensor_state['msg'] is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if sensor_state['msg'] is None:
            raise RuntimeError('No llegó odometría; verifica /summit0/odom y el namespace')

        odom = sensor_state['msg']
        frame = odom.header.frame_id
        if not frame:
            raise ValueError('odom.header.frame_id vacío; no enviar una ruta sin frame')
        start = (odom.pose.pose.position.x, odom.pose.pose.position.y)
        grid = WarehouseGrid()
        result = grid.astar(start, goal)
        if node.get_parameter('simplify_path').value:
            points = grid.simplify(start, goal, result.cells)
        else:
            points = [grid.to_world(c) for c in result.cells] + [goal]
        if not points or any(not grid.is_clear_segment(a, b)
                             for a, b in zip([start]+points[:-1], points)):
            raise RuntimeError('Validación geométrica de la ruta fallida')

        node.get_logger().info(
            f'Grid {grid.nx}x{grid.ny} ({RESOLUTION} m/celda); '
            f'robot radius={ROBOT_RADIUS:.3f} m; columnas infladas={COLUMN_CLEARANCE:.3f} m')
        node.get_logger().info(
            f'Frame={frame!r}; inicio=({start[0]:.2f},{start[1]:.2f}); '
            f'meta=({goal[0]:.2f},{goal[1]:.2f}); '
            f'A* costo={result.cost_m:.2f} m; expandidos={result.expanded}; '
            f'celdas={len(result.cells)}; waypoints={len(points)}')
        node.get_logger().info('Waypoints: ' + ', '.join(f'({x:.2f},{y:.2f})' for x,y in points))

        msg = Path()
        msg.header.frame_id = frame  # debe coincidir exactamente con la odom
        msg.header.stamp = node.get_clock().now().to_msg()
        for x, y in points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        deadline = time.monotonic() + float(node.get_parameter('follower_timeout').value)
        while rclpy.ok() and pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if pub.get_subscription_count() == 0:
            raise RuntimeError('Nadie escucha planned_path; inicia path_follower primero')
        pub.publish(msg)
        node.get_logger().info('Ruta publicada en planned_path; observa path_follower y Gazebo.')
        # Dar tiempo a DDS a transmitir antes de cerrar este publicador de un solo uso.
        end = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    run_ros()
