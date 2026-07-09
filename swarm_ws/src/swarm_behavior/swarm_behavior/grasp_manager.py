"""grasp_manager: agarre fiable de la pinza del Summit con DetachableJoint.

El gripper del Summit "solo abre/cierra" (sin fisica de agarre real, que en
DART es poco fiable). Para sujetar objetos de verdad se usan plugins
DetachableJoint (uno por objeto de basura, definidos en gripper.urdf.xacro,
que NACEN sueltos). Este nodo decide CUANDO y QUE objeto adjuntar:

  /<robot>/gripper/grasp   (std_msgs/Empty) -> cierra los dedos y adjunta el
                            objeto de basura mas cercano a la pinza (si hay uno
                            dentro de grasp_radius).
  /<robot>/gripper/release (std_msgs/Empty) -> abre los dedos y suelta el
                            objeto que tuviera agarrado.

Posiciones: la del robot se lee de /<robot>/odom (pose mundo). Las de los
objetos se obtienen con una instantanea de gz topic .../pose/info (on-demand,
mismo patron que la secuencia de despegue de los drones). El attach/detach se
dispara publicando gz.msgs.Empty en los topicos gz del DetachableJoint.

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

        self.pose = None      # (x, y, yaw) del robot
        self.held = None      # nombre del objeto agarrado, o None

        self.create_subscription(Odometry, f"/{self.ns}/odom", self._odom, 10)
        self.create_subscription(Empty, f"/{self.ns}/gripper/grasp", self._grasp, 10)
        self.create_subscription(Empty, f"/{self.ns}/gripper/release", self._release, 10)
        self.traj_pub = self.create_publisher(
            JointTrajectory, f"/{self.ns}/gripper_controller/joint_trajectory", 10)

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
        self._move_fingers(self.finger_close)
        self._gz_empty(f"/{self.ns}/grasp/{best}/attach")
        self.held = best
        self.get_logger().info(f"AGARRADO '{best}' (a {best_d:.2f} m de la pinza).")

    def _release(self, _msg):
        self._move_fingers(self.finger_open)
        if self.held is None:
            self.get_logger().info("Pinza abierta (no tenia nada agarrado).")
            return
        self._gz_empty(f"/{self.ns}/grasp/{self.held}/detach")
        self.get_logger().info(f"SOLTADO '{self.held}'.")
        self.held = None


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
