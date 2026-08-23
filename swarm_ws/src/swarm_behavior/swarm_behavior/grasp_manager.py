"""Agarre de la pinza del Summit mediante una union fisica DetachableJoint.

La aproximacion deja la pieza dentro de la boca de la pinza. Al agarrar se
crea una union fija en la pose que la pieza ya ocupa: no se cambia su pose ni
se mueve artificialmente durante el transporte. Al soltar se elimina la union
y Gazebo vuelve a simular la pieza libremente.

  /<robot>/gripper/grasp   (std_msgs/Empty) -> cierra los dedos y "coge" el
                            objeto mas cercano dentro de grasp_radius y lo une
                            al robot en su pose actual.
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
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gz.transport13 import Node as GzNode
from gz.msgs10.stringmsg_pb2 import StringMsg

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
        # Posicion de los dedos al AGARRAR: se pinzan al ancho de la caja (0.12).
        # Hueco geometrico = 0.06 + 2*v. v=0.028 manda 0.116 m para una caja de
        # 0.120 m: los 2 mm por lado generan contacto/presion visible y el joint
        # se confirma despues, antes de que el solver pueda expulsarla.
        self.finger_grip = self.declare_parameter("finger_grip", 0.028).value
        self.close_delay = self.declare_parameter("close_delay", 0.8).value
        self.attach_timeout = self.declare_parameter("attach_timeout", 2.5).value
        self.attach_retry = self.declare_parameter("attach_retry", 0.35).value
        self.pose = None      # (x, y, yaw) del robot
        self.held = None      # nombre del objeto agarrado, o None
        self.pending = None   # objeto durante cierre + confirmacion del joint
        self._grasp_t0 = None
        self._last_attach = None
        self.release_pending = None
        self._release_t0 = None
        self._last_detach = None
        self._joint_state = None
        self.gz = GzNode()
        # El joint URDF arranca en 0.0 (cerrado). Repetir la orden abierta
        # mientras el robot esta libre tambien cubre el arranque tardio del
        # gripper_controller: la caja puede entrar antes de empezar a cerrar.
        self._keep_open = True
        self._open_logged = False

        self.create_subscription(Odometry, f"/{self.ns}/odom", self._odom, 10)
        self.create_subscription(Empty, f"/{self.ns}/gripper/grasp", self._grasp, 10)
        self.create_subscription(Empty, f"/{self.ns}/gripper/release", self._release, 10)
        self.traj_pub = self.create_publisher(
            JointTrajectory, f"/{self.ns}/gripper_controller/joint_trajectory", 10)
        self.result_pub = self.create_publisher(
            Int8, f"/{self.ns}/gripper/result", 10)
        self.release_result_pub = self.create_publisher(
            Int8, f"/{self.ns}/gripper/release_result", 10)
        self.create_timer(0.05, self._grasp_step)
        self.create_timer(0.05, self._release_step)
        self.create_timer(1.0, self._ensure_open)

        self.get_logger().info(
            f"grasp_manager listo. Agarrar: /{self.ns}/gripper/grasp  |  "
            f"Soltar: /{self.ns}/gripper/release")

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

    def _ensure_open(self):
        """Mantiene la boca abierta hasta que comienza un agarre real."""
        if not self._keep_open or self.held is not None or self.pending is not None:
            return
        self._move_fingers(self.finger_open)
        if not self._open_logged:
            self.get_logger().info(
                f"Pinza abierta para aproximacion ({self.finger_open:.3f} m).")
            self._open_logged = True

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
        result = subprocess.run(
            ["gz", "topic", "-t", topic, "-m", "gz.msgs.Empty", "-p", ""],
            capture_output=True, text=True, timeout=5)
        return result.returncode == 0

    def _joint_state_cb(self, msg: StringMsg):
        self._joint_state = msg.data

    def _finish_pending(self, success):
        name = self.pending
        if name is not None:
            self.gz.unsubscribe(f"/{self.ns}/grasp/{name}/state")
        self.pending = None
        self._grasp_t0 = None
        self._last_attach = None
        self._joint_state = None
        if not success:
            self._keep_open = True
        self.result_pub.publish(Int8(data=1 if success else -1))

    def _grasp_step(self):
        """Cierra primero y adjunta despues; no autoriza transporte sin ACK."""
        if self.pending is None or self._grasp_t0 is None:
            return
        now = self.get_clock().now()
        elapsed = (now - self._grasp_t0).nanoseconds / 1e9
        if self._joint_state == "attached":
            self.held = self.pending
            self.get_logger().info(f"JOINT CONFIRMADO para '{self.held}'.")
            self._finish_pending(True)
            return
        if elapsed > self.attach_timeout:
            failed = self.pending
            self._move_fingers(self.finger_open)
            self.get_logger().error(
                f"Gazebo no confirmo el agarre de '{failed}'; se reintentara.")
            self._finish_pending(False)
            return
        if elapsed < self.close_delay:
            return
        since_last = float("inf") if self._last_attach is None else \
            (now - self._last_attach).nanoseconds / 1e9
        if since_last >= self.attach_retry:
            self._gz_empty(f"/{self.ns}/grasp/{self.pending}/attach")
            self._last_attach = now

    def _finish_release(self, success):
        name = self.release_pending
        if name is not None:
            self.gz.unsubscribe(f"/{self.ns}/grasp/{name}/state")
        if success:
            self.held = None
        self.release_pending = None
        self._release_t0 = None
        self._last_detach = None
        self._joint_state = None
        self.release_result_pub.publish(Int8(data=1 if success else -1))

    def _release_step(self):
        """Repite detach y no libera el estado local hasta recibir `detached`."""
        if self.release_pending is None or self._release_t0 is None:
            return
        now = self.get_clock().now()
        elapsed = (now - self._release_t0).nanoseconds / 1e9
        if self._joint_state == "detached":
            dropped = self.release_pending
            self.get_logger().info(
                f"DETACH CONFIRMADO para '{dropped}'; pieza libre en deposito.")
            self._finish_release(True)
            return
        if elapsed > self.attach_timeout:
            name = self.release_pending
            self.get_logger().error(
                f"Gazebo no confirmo la suelta de '{name}'; robot seguira parado.")
            self._finish_release(False)
            return
        since_last = float("inf") if self._last_detach is None else \
            (now - self._last_detach).nanoseconds / 1e9
        if since_last >= self.attach_retry:
            self._gz_empty(f"/{self.ns}/grasp/{self.release_pending}/detach")
            self._last_detach = now

    # ---- callbacks ----
    def _grasp(self, _msg):
        if self.held is not None or self.pending is not None:
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
        # La pieza ya fue colocada por la aproximacion visual. Primero se cierran
        # los dedos hasta tocarla; despues se crea y confirma la union en esa
        # misma pose, sin mover artificialmente el objeto.
        self._joint_state = None
        state_topic = f"/{self.ns}/grasp/{best}/state"
        self.gz.subscribe(StringMsg, state_topic, self._joint_state_cb)
        self._keep_open = False
        self._move_fingers(self.finger_grip)
        self.pending = best
        self._grasp_t0 = self.get_clock().now()
        self._last_attach = None
        self.get_logger().info(
            f"CERRANDO sobre '{best}' (a {best_d:.2f} m); esperando joint.")

    def _release(self, _msg):
        self._keep_open = True
        self._move_fingers(self.finger_open)
        if self.release_pending is not None:
            self.get_logger().info("La confirmacion de suelta ya esta en curso.")
            return
        if self.pending is not None:
            self._finish_pending(False)
        if self.held is None:
            self.get_logger().info("Pinza abierta (no tenia nada agarrado).")
            self.release_result_pub.publish(Int8(data=1))
            return
        self.release_pending = self.held
        self._joint_state = None
        state_topic = f"/{self.ns}/grasp/{self.release_pending}/state"
        self.gz.subscribe(StringMsg, state_topic, self._joint_state_cb)
        self._release_t0 = self.get_clock().now()
        self._last_detach = None
        self.get_logger().info(
            f"ABRIENDO y soltando '{self.release_pending}'; esperando confirmacion.")


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
