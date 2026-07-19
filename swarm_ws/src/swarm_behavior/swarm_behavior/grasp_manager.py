"""grasp_manager: agarre fiable de la pinza del Summit por TRANSPORTE CINEMATICO.

El gripper del Summit "solo abre/cierra" (sin fisica de agarre real, que en
DART es poco fiable). Se probo sujetar la basura con DetachableJoint (soldadura
rigida), pero soldar un objeto que toca el suelo lo ARRASTRA y con la pieza
0.6 m por delante actua de "pata" que descarga las ruedas -> el mecanum se
CONGELA. Soldarlo "en el aire" es imposible de forma repetible: `set_pose`
tarda ~0.3 s (CLI) y el cubo cae al suelo en <0.3 s -> la soldadura captura
una altura no determinista (malo para el dataset).

Solucion: TRANSPORTE CINEMATICO. Mientras lleva la pieza, el nodo la teleporta
a ~30 Hz a una pose FIJA flotando delante del robot (hold_forward/hold_height,
en frame cuerpo). Asi la pieza va SIEMPRE en el mismo sitio (repetible), NUNCA
toca el suelo (sin arrastre) y sigue al robot con exactitud. No se usa la
soldadura para el transporte (fightearia con el teleport). El `set_pose` se
hace por los bindings Python de gz-transport (gz.transport13), que tras el
discovery inicial responde en ~1-5 ms (el CLI `gz service` costaba ~0.3 s).

  /<robot>/gripper/grasp   (std_msgs/Empty) -> cierra los dedos y "coge" el
                            objeto mas cercano dentro de grasp_radius; empieza a
                            transportarlo (teleport a la pose fija de sujecion).
  /<robot>/gripper/release (std_msgs/Empty) -> abre los dedos, deja de
                            transportar y suelta la pieza (cae al suelo donde
                            este = deposito).

Posiciones: la del robot se lee de /<robot>/odom (pose mundo). Las de los
objetos se obtienen con una instantanea de gz topic .../pose/info (on-demand).

  ros2 run swarm_behavior grasp_manager --ros-args -p robot:=summit0 -p world:=world_demo
"""
import math
import re
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.msgs10.boolean_pb2 import Boolean as GzBoolean


class GraspManager(Node):
    def __init__(self):
        super().__init__("grasp_manager")
        self.ns = self.declare_parameter("robot", "summit0").value
        self.world = self.declare_parameter("world", "world_demo").value
        self.trash_prefix = self.declare_parameter("trash_prefix", "trash_").value
        # offset desde el centro del robot hasta la zona de los dedos (frame robot)
        self.palm_forward = self.declare_parameter("palm_forward", 0.45).value
        # Radio de agarre generoso: en teleop el objeto (sobre todo los prismas,
        # que se deslizan al empujarlos) rara vez queda a <0.30 m del tip. Con
        # 0.45 m el agarre es tolerante y, al elegir siempre el objeto MAS
        # cercano al tip, no coge el equivocado. Afinable por parametro.
        self.grasp_radius = self.declare_parameter("grasp_radius", 0.45).value
        self.finger_open = self.declare_parameter("finger_open", 0.04).value
        self.finger_close = self.declare_parameter("finger_close", 0.0).value
        # Posicion de los dedos al AGARRAR: no cierran del todo (0.0), sino que
        # se pinzan al ancho de la pieza (0.10 m). Hueco = 0.06 + 2*v, cara
        # interna en ±(0.03+v); v=0.025 -> caras a ±0.055 (5 mm de holgura sobre
        # la pieza ±0.05) -> los dedos abrazan la pieza sin solaparla (sin
        # jitter). Da el aspecto de agarre real en vez de cerrar sobre el vacio.
        self.finger_grip = self.declare_parameter("finger_grip", 0.025).value
        # Pose FIJA de sujecion (frame cuerpo): mientras transporta, la pieza se
        # teleporta AQUI cada tick para que el transporte sea identico y
        # repetible (dataset). hold_forward = x delante del centro (> alcance de
        # los dedos ~0.47 m para NO solaparse con la colision del gripper);
        # hold_height = z LEVANTADA para que viaje FLOTANDO (nunca toca el suelo
        # -> sin arrastre; con la pieza apoyada la friccion congela el mecanum).
        # Pose de sujecion para un AGARRE NATURAL (la pieza NO flota): la pieza
        # de 0.20 alto se lleva de pie A RAS DE SUELO, sujeta entre los dedos
        # (que estan en z~=0.097-0.157 -> abrazan su mitad inferior).
        # hold_forward=0.50 la coloca en la boca del gripper (cara trasera
        # ~x=0.45, a la altura de las puntas de los dedos ~0.475, sin chocar con
        # la palma que acaba en x=0.385). hold_height=0.11 = altura de reposo de
        # la pieza (centro a media altura, fondo ~ras de suelo) -> el teleport no
        # pelea con la gravedad, la pieza va estable sin flotar ni dar botes.
        # Como el transporte es cinematico (teleport, sin union rigida al robot),
        # que la pieza toque el suelo NO reintroduce arrastre (el robot va a
        # velocidad normal; medido ~0.5 m/s con pieza).
        self.hold_forward = self.declare_parameter("hold_forward", 0.50).value
        self.hold_height = self.declare_parameter("hold_height", 0.11).value
        # media ALTURA de la pieza (m): al soltar se baja a z=trash_half para
        # que caiga a ras de suelo sin penetrar. Debe casar con _TRASH_H/2 del
        # launch (pieza 0.20 alto -> 0.10).
        self.trash_half = self.declare_parameter("trash_half", 0.10).value
        self.carry_hz = self.declare_parameter("carry_hz", 30.0).value

        self.pose = None      # (x, y, yaw) del robot
        self.held = None      # nombre del objeto agarrado, o None

        # Cliente gz-transport para teleport rapido (set_pose). El CLI cuesta
        # ~0.3 s/llamada -> inutil a 30 Hz; estos bindings ~1-5 ms tras warmup.
        self.gz = GzNode()
        self.set_pose_srv = f"/world/{self.world}/set_pose"

        self.create_subscription(Odometry, f"/{self.ns}/odom", self._odom, 10)
        self.create_subscription(Empty, f"/{self.ns}/gripper/grasp", self._grasp, 10)
        self.create_subscription(Empty, f"/{self.ns}/gripper/release", self._release, 10)
        self.traj_pub = self.create_publisher(
            JointTrajectory, f"/{self.ns}/gripper_controller/joint_trajectory", 10)

        # Timer de transporte cinematico: teleporta la pieza agarrada a la pose
        # fija de sujecion (no-op cuando no lleva nada).
        self.create_timer(1.0 / max(self.carry_hz, 1.0), self._carry)

        # El DetachableJoint de gz-sim8 NACE ADJUNTADO -> los objetos siguen al
        # robot desde el inicio. Soltarlos todos al arrancar (reintenta hasta
        # que la sim publica poses). Timer one-shot que se autocancela.
        self._init_timer = self.create_timer(2.0, self._startup_detach)

        self.get_logger().info(
            f"grasp_manager listo. Agarrar: /{self.ns}/gripper/grasp  |  "
            f"Soltar: /{self.ns}/gripper/release")

    def _startup_detach(self):
        objs = self._object_positions()
        if not objs:
            self.get_logger().warn("Arranque: aun no veo objetos, reintento...")
            return  # el timer vuelve a disparar en 2 s
        for name in objs:
            self._gz_empty(f"/{self.ns}/grasp/{name}/detach")
        self.get_logger().info(
            f"Soltados {len(objs)} objetos al inicio (nacen pegados al robot).")
        self._init_timer.cancel()

    # ---- estado del robot ----
    def _odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    # ---- dedos ----
    def _move_fingers(self, pos):
        jt = JointTrajectory()
        jt.joint_names = ["finger_left_joint", "finger_right_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = [float(pos), float(pos)]
        pt.time_from_start.sec = 1
        jt.points = [pt]
        self.traj_pub.publish(jt)

    # ---- poses de los objetos (instantanea gz) ----
    def _object_positions(self):
        try:
            out = subprocess.run(
                ["gz", "topic", "-e", "-t",
                 f"/world/{self.world}/pose/info", "-n", "1"],
                capture_output=True, text=True, timeout=5).stdout
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"gz topic pose/info fallo: {e}")
            return {}
        objs = {}
        lines = out.splitlines()
        cur = None
        for idx, raw in enumerate(lines):
            line = raw.strip()
            m = re.match(r'name:\s*"([^"]+)"', line)
            if m:
                cur = m.group(1)
                continue
            if line.startswith("position") and cur and cur.startswith(self.trash_prefix):
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
                    objs[cur] = (x, y)
                cur = None
        return objs

    def _gz_empty(self, topic):
        subprocess.run(
            ["gz", "topic", "-t", topic, "-m", "gz.msgs.Empty", "-p", ""],
            capture_output=True, text=True, timeout=5)

    def _gz_set_pose(self, name, x, y, z, yaw):
        """Teleporta un modelo a (x,y,z) con orientacion vertical y el yaw dado,
        via bindings Python de gz-transport (rapido, ~1-5 ms tras discovery).
        Devuelve True si el servicio aplico la pose."""
        req = GzPose()
        req.name = name
        req.position.x = float(x)
        req.position.y = float(y)
        req.position.z = float(z)
        req.orientation.z = math.sin(yaw / 2.0)
        req.orientation.w = math.cos(yaw / 2.0)
        # timeout amplio en la 1a llamada (discovery); luego responde al instante
        ok, res = self.gz.request(self.set_pose_srv, req, GzPose, GzBoolean, 1000)
        return ok and res.data

    def _hold_pose(self):
        """(x, y, z, yaw) de la pose fija de sujecion, en frame mundo, a partir
        de la odom actual del robot."""
        rx, ry, yaw = self.pose
        hx = rx + self.hold_forward * math.cos(yaw)
        hy = ry + self.hold_forward * math.sin(yaw)
        return hx, hy, self.hold_height, yaw

    def _carry(self):
        """Transporte cinematico: teleporta la pieza agarrada a la pose fija de
        sujecion (flotando delante del robot). No-op si no lleva nada."""
        if self.held is None or self.pose is None:
            return
        hx, hy, hz, yaw = self._hold_pose()
        self._gz_set_pose(self.held, hx, hy, hz, yaw)

    # ---- callbacks ----
    def _grasp(self, _msg):
        if self.held is not None:
            self.get_logger().warn(f"Ya tengo agarrado '{self.held}'. Suelta antes.")
            return
        if self.pose is None:
            self.get_logger().warn("Sin odometria del robot todavia.")
            return
        rx, ry, yaw = self.pose
        tipx = rx + self.palm_forward * math.cos(yaw)
        tipy = ry + self.palm_forward * math.sin(yaw)
        objs = self._object_positions()
        if not objs:
            self.get_logger().warn("No veo objetos de basura en el mundo.")
            return
        best, best_d = None, 1e9
        for name, (ox, oy) in objs.items():
            d = math.hypot(ox - tipx, oy - tipy)
            if d < best_d:
                best, best_d = name, d
        if best is None or best_d > self.grasp_radius:
            self.get_logger().warn(
                f"Objeto mas cercano '{best}' a {best_d:.2f} m "
                f"(> grasp_radius {self.grasp_radius:.2f}). Acerca el robot.")
            return
        # Transporte cinematico: pinzar los dedos al ancho de la pieza (agarre
        # natural) y empezar a teleportar la pieza a la pose fija de sujecion (el
        # timer _carry lo hace a carry_hz). NO se suelda (fightearia con el
        # teleport). Un primer teleport aqui la coloca ya en su sitio.
        self._move_fingers(self.finger_grip)
        self.held = best
        self._carry()
        self.get_logger().info(
            f"AGARRADO '{best}' (a {best_d:.2f} m) -> transporte cinematico "
            "(flotando delante del robot).")

    def _release(self, _msg):
        self._move_fingers(self.finger_open)
        if self.held is None:
            self.get_logger().info("Pinza abierta (no tenia nada agarrado).")
            return
        # Dejar de transportar y soltar: la pieza cae al suelo donde este (sobre
        # el deposito). Un ultimo teleport la baja a ras de suelo delante del
        # robot para que caiga limpia (no desde la altura de sujecion).
        dropped = self.held
        self.held = None
        if self.pose is not None:
            rx, ry, yaw = self.pose
            dx = rx + self.hold_forward * math.cos(yaw)
            dy = ry + self.hold_forward * math.sin(yaw)
            self._gz_set_pose(dropped, dx, dy, self.trash_half, yaw)
        self.get_logger().info(f"SOLTADO '{dropped}'.")


def main():
    rclpy.init()
    node = GraspManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
