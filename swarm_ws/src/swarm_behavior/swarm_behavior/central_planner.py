"""central_planner: asignacion centralizada greedy (Fase A).

Nodo unico que actua como "cerebro" del enjambre con vista cenital. Lee las
poses ground-truth de Gazebo (equivale a una camara cenital perfecta) y asigna
tareas de forma greedy:

  - robot LIBRE  -> basura mas cercana no reclamada (estado SEEK).
  - cerca de la basura delega la aproximacion final a RGB-D; solo cuando queda
    centrada dispara el agarre y pasa a DELIVER.
  - al llegar al deposito (< deposit_radius) dispara la suelta
    (/summitN/gripper/release), marca esa basura como DEPOSITADA y vuelve a SEEK.

CLASIFICACION POR COLOR: cada pieza va al deposito de SU MISMO COLOR. El color
viaja en el TOKEN FINAL del nombre del modelo (trash_2_azul -> deposit_1_azul),
que es lo unico que el planner ve de Gazebo. Cualquier robot puede coger
cualquier pieza (manda el color de la PIEZA, no el del robot). Si un color no
tiene deposito (o el nombre no lleva color, p.ej. un mundo antiguo), se cae al
deposito MAS CERCANO: nunca se bloquea por esto.

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
from std_msgs.msg import Empty, Int8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray, PoseStamped


class CentralPlanner(Node):
    def __init__(self):
        super().__init__("central_planner")
        self.n_robots = self.declare_parameter("n_robots", 3).value
        self.world = self.declare_parameter("world", "world_demo").value
        self.trash_prefix = self.declare_parameter("trash_prefix", "trash_").value
        self.deposit_prefix = self.declare_parameter("deposit_prefix", "deposit_").value
        # La meta de la basura se pone approach_offset m ANTES de la pieza
        # (sobre la linea robot->basura) para que el chasis no la embista/tumbe.
        self.approach_offset = self.declare_parameter("approach_offset", 0.30).value
        # deposit_radius GENEROSO (> distancia a la que N robots se amontonan por
        # repulsion LiDAR ~0.6-0.7 m): sin evitacion mutua (Fase C/D), varios
        # robots hacia el MISMO deposito se bloqueaban a ~0.6-1.0 m del centro,
        # justo fuera de un radio pequeno -> nunca soltaban (deadlock). Al soltar,
        # la pieza queda delante del robot, dentro de la pinza, por lo que al
        # soltar cerca del centro cae dentro del disco. Rompe el deadlock.
        self.deposit_radius = self.declare_parameter("deposit_radius", 0.85).value
        self.rate = self.declare_parameter("rate", 2.0).value
        # colision robot-robot: por debajo de collision_dist cuenta como choque
        # (con histeresis collision_clear para no recontar el mismo evento).
        self.collision_dist = self.declare_parameter("collision_dist", 0.7).value
        self.collision_clear = self.declare_parameter("collision_clear", 0.9).value
        # Un robot IDLE (sin basura que buscar) parado sobre un deposito bloquea
        # a otro que va a depositar (aun no hay evitacion mutua: Fase C/D). Como
        # stopgap, un robot idle a < retreat_clear de un deposito se RETIRA a
        # retreat_dist de el (hacia afuera) para vaciar la zona.
        self.retreat_clear = self.declare_parameter("retreat_clear", 1.4).value
        self.retreat_dist = self.declare_parameter("retreat_dist", 2.2).value
        self.visual_start_radius = self.declare_parameter(
            "visual_start_radius", 0.95).value
        # ---- Evitar ARRASTRAR otras piezas mientras se transporta una ----
        # El LiDAR va a z=0.557 m y las cajas miden 0.20 m: son INVISIBLES al
        # scan, asi que go_to_goal no puede esquivarlas por su cuenta. El
        # planner (que si las ve por ground-truth) le publica las posiciones a
        # evitar en /<ns>/avoid_points mientras el robot lleva una pieza.
        # Las piezas que ya estan EN el deposito destino se excluyen: repelerse
        # de ellas impediria acercarse a soltar (deadlock del deposito).
        self.deposit_avoid_clear = self.declare_parameter(
            "deposit_avoid_clear", 1.0).value

        self.robots = [f"summit{i}" for i in range(self.n_robots)]
        # estado por robot
        # seek | align | grasp | deliver | release | idle
        self.state = {r: "seek" for r in self.robots}
        self.target = {r: None for r in self.robots}       # (kind, name) o None
        self.carried = {r: None for r in self.robots}      # nombre de la basura agarrada
        self.done_trash = set()                            # basuras ya depositadas
        self.n_right_color = 0                             # piezas en su deposito
        self.n_wrong_color = 0                             # piezas por fallback
        self.robot_xy = {r: None for r in self.robots}     # de odom
        self.last_goal = {r: None for r in self.robots}    # (kind, name) ya publicado
        self.align_result = {r: 0 for r in self.robots}    # 0 esperando, 1 listo, -1 aborto
        self.grasp_result = {r: 0 for r in self.robots}    # ACK real de DetachableJoint
        self.release_result = {r: 0 for r in self.robots}  # ACK `detached`

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
        self.retreat_pub = {}
        self.align_target_pub = {}
        self.avoid_pub = {}
        for r in self.robots:
            self.goal_pub[r] = self.create_publisher(PoseStamped, f"/{r}/goal_pose", 10)
            self.grasp_pub[r] = self.create_publisher(Empty, f"/{r}/gripper/grasp", 10)
            self.release_pub[r] = self.create_publisher(Empty, f"/{r}/gripper/release", 10)
            # Orden de reverso recto tras soltar (aparta el gripper de la pieza
            # dejada para que el giro hacia la siguiente basura no la empuje).
            self.retreat_pub[r] = self.create_publisher(Empty, f"/{r}/retreat", 10)
            self.align_target_pub[r] = self.create_publisher(
                PoseStamped, f"/{r}/visual_grasp/target", 10)
            self.avoid_pub[r] = self.create_publisher(
                PoseArray, f"/{r}/avoid_points", 10)
            self.create_subscription(
                Odometry, f"/{r}/odom",
                lambda msg, rr=r: self._on_odom(rr, msg), 10)
            self.create_subscription(
                Int8, f"/{r}/visual_grasp/result",
                lambda msg, rr=r: self._on_align_result(rr, msg), 10)
            self.create_subscription(
                Int8, f"/{r}/gripper/result",
                lambda msg, rr=r: self._on_grasp_result(rr, msg), 10)
            self.create_subscription(
                Int8, f"/{r}/gripper/release_result",
                lambda msg, rr=r: self._on_release_result(rr, msg), 10)

        self.timer = self.create_timer(1.0 / max(self.rate, 0.1), self._tick)
        self.get_logger().info(
            f"central_planner: {self.n_robots} robots, mundo '{self.world}'. "
            "Asignacion greedy basura->deposito.")

    # ---- percepcion ----
    def _on_odom(self, robot, msg):
        p = msg.pose.pose.position
        self.robot_xy[robot] = (p.x, p.y)

    @staticmethod
    def _color(name):
        """Color de un modelo = ultimo token del nombre (trash_2_azul -> azul).

        Devuelve None si el nombre no lleva color (nombres antiguos tipo
        'trash_2'), en cuyo caso el emparejamiento se desactiva y se usa el
        deposito mas cercano, como antes."""
        tail = name.rsplit("_", 1)[-1]
        return tail if tail and not tail.isdigit() else None

    def _best_deposit(self, deposits, rxy, trash_name):
        """Deposito mas cercano DEL COLOR de la pieza; si no hay ninguno de ese
        color, el mas cercano a secas (fallback: mejor depositar en el sitio
        equivocado que quedarse con la pieza para siempre)."""
        want = self._color(trash_name) if trash_name else None
        same = {n: xy for n, xy in deposits.items() if self._color(n) == want} \
            if want else {}
        pool = same or deposits
        best = min(pool, key=lambda n: math.hypot(pool[n][0] - rxy[0],
                                                  pool[n][1] - rxy[1]))
        return best, bool(same)

    def _on_align_result(self, robot, msg):
        self.align_result[robot] = int(msg.data)

    def _on_grasp_result(self, robot, msg):
        self.grasp_result[robot] = int(msg.data)

    def _on_release_result(self, robot, msg):
        self.release_result[robot] = int(msg.data)

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

    def _publish_avoid_points(self, robot, objs, deposits):
        """Publica en /<ns>/avoid_points las piezas que este robot NO debe
        barrer con la que lleva. Vacio si no transporta nada (desactiva la
        repulsion virtual en go_to_goal)."""
        msg = PoseArray()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        held = self.carried[robot]
        if held is not None:
            # La pieza que va en la pinza no se esquiva a si misma; las que
            # llevan otros robots tampoco (se mueven con ellos y el robot SI es
            # visible al LiDAR, que ya las evita por el chasis).
            skip = {n for n in self.carried.values() if n is not None}
            # Zona de descarga: no repelerse de lo ya depositado en el deposito
            # destino, o el robot no podria acercarse a soltar.
            tgt = self.target[robot]
            dxy = deposits.get(tgt[1]) if tgt and tgt[0] == "deposit" else None
            for name, xy in objs.items():
                if not name.startswith(self.trash_prefix) or name in skip:
                    continue
                if dxy is not None and math.hypot(xy[0] - dxy[0],
                                                  xy[1] - dxy[1]) < self.deposit_avoid_clear:
                    continue
                pt = Pose()
                pt.position.x = float(xy[0])
                pt.position.y = float(xy[1])
                pt.orientation.w = 1.0
                msg.poses.append(pt)
        self.avoid_pub[robot].publish(msg)

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
        # Una pieza que un robot ya lleva (carried) sigue existiendo en el mundo
        # (flota delante de su robot) y aun NO esta en done_trash -> hay que
        # excluirla del pool o OTRO robot en SEEK la perseguiria y la
        # "depositaria" tambien (doble-agarre: un mismo trash transportado 2
        # veces, viaje desperdiciado).
        carried_set = {self.carried[r] for r in self.robots if self.carried[r]}
        trash = {n: xy for n, xy in objs.items()
                 if n.startswith(self.trash_prefix)
                 and n not in self.done_trash and n not in carried_set}
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
                   if self.state[r] in ("seek", "align", "grasp") and self.target[r]
                   and self.target[r][0] == "trash"}
        claimed.update(name for name in self.carried.values() if name is not None)

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
                if math.hypot(txy[0] - rxy[0], txy[1] - rxy[1]) < self.visual_start_radius:
                    target_msg = PoseStamped()
                    target_msg.header.frame_id = "map"
                    target_msg.header.stamp = self.get_clock().now().to_msg()
                    target_msg.pose.position.x = float(txy[0])
                    target_msg.pose.position.y = float(txy[1])
                    target_msg.pose.orientation.w = 1.0
                    self.align_result[r] = 0
                    self.align_target_pub[r].publish(target_msg)
                    self.state[r] = "align"
                    self.get_logger().info(f"{r}: ALINEA con RGB-D -> {name}")

            elif self.state[r] == "align":
                name = self.target[r][1]
                if self.align_result[r] < 0:
                    # Reintentar desde navegacion global. No se agarra si la
                    # profundidad se perdio o aparecio un obstaculo.
                    self.align_result[r] = 0
                    self.state[r] = "seek"
                    self.last_goal[r] = None
                    self.get_logger().warn(f"{r}: alineacion abortada; reaproxima")
                    continue
                if self.align_result[r] > 0:
                    self.grasp_result[r] = 0
                    self.grasp_pub[r].publish(Empty())
                    self.state[r] = "grasp"
                    self.get_logger().info(
                        f"{r}: pieza alineada; espera confirmacion del joint")

            elif self.state[r] == "grasp":
                name = self.target[r][1]
                if self.grasp_result[r] < 0:
                    self.grasp_result[r] = 0
                    self.align_result[r] = 0
                    self.state[r] = "seek"
                    self.last_goal[r] = None
                    self.get_logger().warn(
                        f"{r}: agarre de {name} no confirmado; reaproxima")
                    continue
                if self.grasp_result[r] > 0:
                    self.grasp_result[r] = 0
                    self.carried[r] = name
                    # Deposito del MISMO COLOR que la pieza (fallback: el mas
                    # cercano, para no quedarse con la pieza si falta ese color).
                    dname, matched = self._best_deposit(deposits, rxy, name)
                    self.target[r] = ("deposit", dname)
                    self.state[r] = "deliver"
                    if matched:
                        self.get_logger().info(
                            f"{r}: JOINT confirmado para {name} -> DELIVER a "
                            f"{dname} (color {self._color(name)})")
                    else:
                        self.get_logger().warn(
                            f"{r}: sin deposito del color de {name}; "
                            f"fallback al mas cercano ({dname})")

            elif self.state[r] == "deliver":
                dname = self.target[r][1]
                dxy = deposits.get(dname)
                if dxy is None:
                    self.state[r] = "seek"
                    self.target[r] = None
                    continue
                self._send_goal(r, "deposit", dname, dxy)
                if math.hypot(dxy[0] - rxy[0], dxy[1] - rxy[1]) < self.deposit_radius:
                    # Congelar la navegacion en la pose actual antes de abrir.
                    # Sin esto go_to_goal seguiria avanzando hacia el centro del
                    # deposito durante la espera del ACK de Gazebo.
                    self._send_goal(r, "hold_release", self.carried[r], rxy)
                    self.release_result[r] = 0
                    self.release_pub[r].publish(Empty())
                    self.state[r] = "release"
                    self.get_logger().info(
                        f"{r}: espera confirmacion de suelta de {self.carried[r]}")

            elif self.state[r] == "release":
                if self.release_result[r] < 0:
                    # No moverse: si el primer detach se perdio, la pieza puede
                    # seguir unida. grasp_manager conserva su estado y reintenta.
                    self.release_result[r] = 0
                    self.release_pub[r].publish(Empty())
                    self.get_logger().warn(
                        f"{r}: suelta no confirmada; reintenta sin moverse")
                    continue
                if self.release_result[r] > 0:
                    self.release_result[r] = 0
                    # Solo ahora apartar el gripper; el objeto ya es libre.
                    self.retreat_pub[r].publish(Empty())
                    if self.carried[r]:
                        self.done_trash.add(self.carried[r])
                        dep = self.target[r][1] if self.target[r] else ""
                        if self._color(self.carried[r]) == self._color(dep):
                            self.n_right_color += 1
                        else:
                            self.n_wrong_color += 1
                    self.get_logger().info(
                        f"{r}: DETACH confirmado para {self.carried[r]} "
                        "-> retrocede -> SEEK")
                    self.carried[r] = None
                    self.state[r] = "seek"
                    self.target[r] = None

            elif self.state[r] == "idle":
                if trash:
                    self.state[r] = "seek"
                else:
                    # sin tareas: retirarse de la zona de depositos si esta encima
                    # (un idle parado ahi bloquea a los que aun depositan).
                    dname = min(deposits,
                                key=lambda n: math.hypot(deposits[n][0] - rxy[0],
                                                         deposits[n][1] - rxy[1]))
                    dxy = deposits[dname]
                    dist = math.hypot(dxy[0] - rxy[0], dxy[1] - rxy[1])
                    if dist < self.retreat_clear:
                        ux = (rxy[0] - dxy[0]) / (dist or 1.0)
                        uy = (rxy[1] - dxy[1]) / (dist or 1.0)
                        park = (dxy[0] + self.retreat_dist * ux,
                                dxy[1] + self.retreat_dist * uy)
                        self._send_goal(r, "park", dname, park)

        # ---- piezas a esquivar mientras se transporta (anti-arrastre) ----
        # `objs` trae TODAS las basuras fisicas del mundo, incluidas las ya
        # depositadas (siguen ahi) y las que otro robot lleva. Se publica la
        # lista solo a los robots que cargan; a los demas, lista vacia (= off).
        for r in self.robots:
            self._publish_avoid_points(r, objs, deposits)

        # ---- metricas ----
        pieces = len(self.done_trash)
        elapsed = (self.get_clock().now() - self.t_start).nanoseconds / 1e9 \
            if self.t_start else 0.0
        self.get_logger().info(
            f"[metricas] piezas {pieces}/{self.total_trash}  "
            f"color OK {self.n_right_color}/{pieces}  "
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
                f"  color correcto     : {self.n_right_color}/{pieces}"
                f" (mal: {self.n_wrong_color})\n"
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
