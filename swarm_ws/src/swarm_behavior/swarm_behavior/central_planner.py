"""central_planner: asignacion centralizada greedy (Fase A).

Nodo unico que actua como "cerebro" del enjambre con vista cenital. Lee las
poses ground-truth de Gazebo (equivale a una camara cenital perfecta) y asigna
tareas de forma greedy:

  - robot LIBRE  -> basura mas cercana no reclamada (estado SEEK).
  - al llegar a la basura (< pickup_radius) dispara el agarre
    (/summitN/gripper/grasp) y pasa a DELIVER, con meta = deposito mas cercano.
  - al llegar al deposito (< deposit_radius) dispara la suelta
    (/summitN/gripper/release), marca esa basura como DEPOSITADA y vuelve a SEEK.

Publica una meta por robot en /summitN/goal_pose (PoseStamped, frame 'map') que
la capa de navegacion (go_to_goal, Fase B) sigue. Requiere un grasp_manager por
robot corriendo para que el agarre/suelta sea efectivo.

Percepcion: posiciones de los robots por /summitN/odom (ROS, ya bridgeado);
posiciones de basura y depositos por instantanea `gz topic .../pose/info`
(no tienen topico ROS). Mismo patron que grasp_manager.

  ros2 run swarm_behavior central_planner --ros-args -p n_robots:=3 -p world:=world_demo
"""
import math
import re
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class CentralPlanner(Node):
    def __init__(self):
        super().__init__("central_planner")
        self.n_robots = self.declare_parameter("n_robots", 3).value
        self.world = self.declare_parameter("world", "world_demo").value
        self.trash_prefix = self.declare_parameter("trash_prefix", "trash_").value
        self.deposit_prefix = self.declare_parameter("deposit_prefix", "deposit_").value
        # La meta de la basura se pone approach_offset m ANTES de la pieza
        # (sobre la linea robot->basura) para que el chasis no la embista/tumbe;
        # el agarre es por soldadura (DetachableJoint), no hace falta tocarla.
        self.approach_offset = self.declare_parameter("approach_offset", 0.35).value
        # pickup_radius DEBE ser > approach_offset + goal_tol de go_to_goal
        # (~0.35+0.3): si no, el robot se para en la meta-offset y el agarre no
        # dispara nunca (deadlock). El agarre es por soldadura y la pinza
        # alcanza ~0.9 m (tip 0.45 + grasp_radius 0.45), asi que 0.75 es seguro.
        self.pickup_radius = self.declare_parameter("pickup_radius", 0.75).value
        self.deposit_radius = self.declare_parameter("deposit_radius", 0.5).value
        self.rate = self.declare_parameter("rate", 2.0).value
        # colision robot-robot: por debajo de collision_dist cuenta como choque
        # (con histeresis collision_clear para no recontar el mismo evento).
        self.collision_dist = self.declare_parameter("collision_dist", 0.7).value
        self.collision_clear = self.declare_parameter("collision_clear", 0.9).value

        self.robots = [f"summit{i}" for i in range(self.n_robots)]
        # estado por robot
        self.state = {r: "seek" for r in self.robots}     # seek | deliver | idle
        self.target = {r: None for r in self.robots}       # (kind, name) o None
        self.carried = {r: None for r in self.robots}      # nombre de la basura agarrada
        self.done_trash = set()                            # basuras ya depositadas
        self.robot_xy = {r: None for r in self.robots}     # de odom
        self.last_goal = {r: None for r in self.robots}    # (kind, name) ya publicado

        # --- metricas ---
        self.t_start = None            # 1er tick con basura visible
        self.total_trash = None        # nº de basuras al inicio
        self.makespan = None           # s hasta depositar todas
        self.n_collisions = 0          # eventos de choque robot-robot
        self._colliding = set()        # pares actualmente en colision (frozenset)

        # pubs/subs por robot
        self.goal_pub = {}
        self.grasp_pub = {}
        self.release_pub = {}
        for r in self.robots:
            self.goal_pub[r] = self.create_publisher(PoseStamped, f"/{r}/goal_pose", 10)
            self.grasp_pub[r] = self.create_publisher(Empty, f"/{r}/gripper/grasp", 10)
            self.release_pub[r] = self.create_publisher(Empty, f"/{r}/gripper/release", 10)
            self.create_subscription(
                Odometry, f"/{r}/odom",
                lambda msg, rr=r: self._on_odom(rr, msg), 10)

        self.timer = self.create_timer(1.0 / max(self.rate, 0.1), self._tick)
        self.get_logger().info(
            f"central_planner: {self.n_robots} robots, mundo '{self.world}'. "
            "Asignacion greedy basura->deposito.")

    # ---- percepcion ----
    def _on_odom(self, robot, msg):
        p = msg.pose.pose.position
        self.robot_xy[robot] = (p.x, p.y)

    def _gz_poses(self, prefixes):
        """{name: (x,y)} de los modelos cuyo nombre empieza por algun prefijo."""
        try:
            out = subprocess.run(
                ["gz", "topic", "-e", "-t",
                 f"/world/{self.world}/pose/info", "-n", "1"],
                capture_output=True, text=True, timeout=5).stdout
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"gz pose/info fallo: {e}")
            return {}
        poses = {}
        lines = out.splitlines()
        cur = None
        for idx, raw in enumerate(lines):
            line = raw.strip()
            m = re.match(r'name:\s*"([^"]+)"', line)
            if m:
                cur = m.group(1)
                continue
            if line.startswith("position") and cur and cur.startswith(tuple(prefixes)):
                x = y = None
                for j in range(idx + 1, min(idx + 5, len(lines))):
                    s = lines[j].strip()
                    xm = re.match(r"x:\s*([-\d.eE+]+)", s)
                    ym = re.match(r"y:\s*([-\d.eE+]+)", s)
                    if xm:
                        x = float(xm.group(1))
                    if ym:
                        y = float(ym.group(1))
                    if x is not None and y is not None:
                        break
                if x is not None and y is not None:
                    poses[cur] = (x, y)
                cur = None
        return poses

    # ---- acciones ----
    def _publish_goal(self, robot, xy):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(xy[0])
        msg.pose.position.y = float(xy[1])
        msg.pose.orientation.w = 1.0
        self.goal_pub[robot].publish(msg)

    def _send_goal(self, robot, kind, name, xy):
        """Publica la meta si cambio de objetivo, si su posicion se movio
        >0.15 m (la meta de aproximacion se recalcula al avanzar el robot), o
        en el keepalive. Evita resetear el latch/spamear en cada tick."""
        prev = self.last_goal[robot]
        moved = (prev is None or prev[:2] != (kind, name)
                 or math.hypot(prev[2] - xy[0], prev[3] - xy[1]) > 0.15)
        if moved or self._keepalive:
            self.last_goal[robot] = (kind, name, xy[0], xy[1])
            self._publish_goal(robot, xy)

    def _count_collisions(self):
        """Cuenta eventos de choque robot-robot (flanco de subida, con
        histeresis): un par pasa a < collision_dist => +1; sale al superar
        collision_clear."""
        rs = [r for r in self.robots if self.robot_xy[r] is not None]
        for i in range(len(rs)):
            for j in range(i + 1, len(rs)):
                a, b = rs[i], rs[j]
                pa, pb = self.robot_xy[a], self.robot_xy[b]
                d = math.hypot(pa[0] - pb[0], pa[1] - pb[1])
                pair = frozenset((a, b))
                if d < self.collision_dist and pair not in self._colliding:
                    self._colliding.add(pair)
                    self.n_collisions += 1
                    self.get_logger().warn(
                        f"COLISION {a}-{b} (d={d:.2f} m). Total: {self.n_collisions}")
                elif d > self.collision_clear and pair in self._colliding:
                    self._colliding.discard(pair)

    # ---- bucle de planificacion ----
    def _tick(self):
        # keepalive: republicar la meta cada ~3 s aunque no cambie (por si un
        # go_to_goal arranco tarde y perdio la 1a publicacion).
        self._keepalive = getattr(self, "_tick_n", 0) % 6 == 0
        self._tick_n = getattr(self, "_tick_n", 0) + 1

        objs = self._gz_poses((self.trash_prefix, self.deposit_prefix))
        trash = {n: xy for n, xy in objs.items()
                 if n.startswith(self.trash_prefix) and n not in self.done_trash}
        deposits = {n: xy for n, xy in objs.items()
                    if n.startswith(self.deposit_prefix)}
        if not deposits:
            return  # sin depositos no hay nada que planificar

        # metricas: arranque del cronometro y total de basuras (1er tick util)
        if self.t_start is None and (trash or self.done_trash):
            self.t_start = self.get_clock().now()
            self.total_trash = len(trash) + len(self.done_trash)
        self._count_collisions()

        # basuras ya reclamadas por otro robot en SEEK (para no perseguir la misma)
        claimed = {self.target[r][1] for r in self.robots
                   if self.state[r] == "seek" and self.target[r]
                   and self.target[r][0] == "trash"}

        for r in self.robots:
            rxy = self.robot_xy[r]
            if rxy is None:
                continue  # aun sin odom

            if self.state[r] == "seek":
                # (re)asignar si no hay target valido
                tgt = self.target[r]
                if tgt is None or tgt[1] not in trash:
                    best, best_d = None, 1e9
                    for name, xy in trash.items():
                        if name in claimed:
                            continue
                        d = math.hypot(xy[0] - rxy[0], xy[1] - rxy[1])
                        if d < best_d:
                            best, best_d = name, d
                    if best is None:
                        self.state[r] = "idle"
                        self.target[r] = None
                        continue
                    self.target[r] = ("trash", best)
                    claimed.add(best)
                    self.get_logger().info(f"{r}: SEEK -> {best} ({best_d:.1f} m)")
                name = self.target[r][1]
                txy = trash[name]
                # meta = punto approach_offset m antes de la basura, sobre la
                # linea robot->basura (no el centro exacto, para no tumbarla).
                dx, dy = rxy[0] - txy[0], rxy[1] - txy[1]
                d = math.hypot(dx, dy) or 1.0
                goal = (txy[0] + self.approach_offset * dx / d,
                        txy[1] + self.approach_offset * dy / d)
                self._send_goal(r, "trash", name, goal)
                if math.hypot(txy[0] - rxy[0], txy[1] - rxy[1]) < self.pickup_radius:
                    self.grasp_pub[r].publish(Empty())
                    self.carried[r] = name
                    # deposito mas cercano
                    dname = min(deposits,
                                key=lambda n: math.hypot(deposits[n][0] - rxy[0],
                                                         deposits[n][1] - rxy[1]))
                    self.target[r] = ("deposit", dname)
                    self.state[r] = "deliver"
                    self.get_logger().info(
                        f"{r}: AGARRA {name} -> DELIVER a {dname}")

            elif self.state[r] == "deliver":
                dname = self.target[r][1]
                dxy = deposits.get(dname)
                if dxy is None:
                    self.state[r] = "seek"
                    self.target[r] = None
                    continue
                self._send_goal(r, "deposit", dname, dxy)
                if math.hypot(dxy[0] - rxy[0], dxy[1] - rxy[1]) < self.deposit_radius:
                    self.release_pub[r].publish(Empty())
                    if self.carried[r]:
                        self.done_trash.add(self.carried[r])
                    self.get_logger().info(
                        f"{r}: SUELTA {self.carried[r]} en {dname} -> SEEK")
                    self.carried[r] = None
                    self.state[r] = "seek"
                    self.target[r] = None

            elif self.state[r] == "idle":
                if trash:
                    self.state[r] = "seek"

        # ---- metricas ----
        pieces = len(self.done_trash)
        elapsed = (self.get_clock().now() - self.t_start).nanoseconds / 1e9 \
            if self.t_start else 0.0
        self.get_logger().info(
            f"[metricas] piezas {pieces}/{self.total_trash}  "
            f"colisiones {self.n_collisions}  t {elapsed:.1f}s",
            throttle_duration_sec=3.0)

        # makespan: todas depositadas y ningun robot cargando
        all_done = (self.total_trash is not None and pieces >= self.total_trash
                    and not trash and all(self.state[r] != "deliver"
                                          for r in self.robots))
        if all_done and self.makespan is None:
            self.makespan = elapsed
            self.get_logger().info(
                "==== TAREA COMPLETA ====\n"
                f"  piezas depositadas : {pieces}/{self.total_trash}\n"
                f"  makespan           : {self.makespan:.1f} s\n"
                f"  colisiones         : {self.n_collisions}")


def main():
    rclpy.init()
    node = CentralPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
