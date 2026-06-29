"""ps5_teleop: maneja un Summit (+ gripper) con un mando DualSense de PS5.

Lee /joy (driver `joy`) y publica:
  /<robot>/cmd_vel          (Twist)  -> stick izq = avanzar/strafe, stick der = girar
  /<robot>/gripper/grasp    (Empty)  -> boton (def R1) = agarrar el objeto mas cercano
  /<robot>/gripper/release  (Empty)  -> boton (def L1) = soltar
Mantener un boton (def R2) = turbo (x2 velocidad).

Pensado para teleoperar y GRABAR demostraciones (ACT): observacion
(/<robot>/odom, /imu, /camera) <-> accion (/cmd_vel + grasp/release).

El mapeo de ejes/botones del DualSense puede variar segun kernel/driver.
Verificalo con `ros2 topic echo /joy` y ajusta los params axis_*/button_* o los
signos invert_* si algun eje va al reves.

  ros2 launch swarm_behavior teleop_ps5.launch.py robot:=summit0
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class PS5Teleop(Node):
    def __init__(self):
        super().__init__("ps5_teleop")
        self.ns = self.declare_parameter("robot", "summit0").value
        # ejes (DualSense via kernel hid-playstation): 0 izqX, 1 izqY, 3 derX, 4 derY
        self.ax_fwd = self.declare_parameter("axis_forward", 1).value
        self.ax_str = self.declare_parameter("axis_strafe", 0).value
        self.ax_yaw = self.declare_parameter("axis_yaw", 3).value
        # signos (ajustados a este mando): stick arriba = avanzar (+x).
        self.inv_fwd = self.declare_parameter("invert_forward", 1.0).value
        self.inv_str = self.declare_parameter("invert_strafe", -1.0).value
        self.inv_yaw = self.declare_parameter("invert_yaw", -1.0).value
        self.max_lin = self.declare_parameter("max_linear", 0.5).value
        self.max_str = self.declare_parameter("max_strafe", 0.5).value
        self.max_yaw = self.declare_parameter("max_yaw", 1.0).value
        self.deadzone = self.declare_parameter("deadzone", 0.08).value
        # frame de control:
        #   "world" (def) = orientado al campo: el stick apunta a una direccion
        #          FIJA en pantalla sin importar como este girado el robot
        #          (intuitivo conduciendo desde una camara fija/cenital).
        #   "body" = en el morro del robot (intuitivo desde la camara frontal).
        self.frame = self.declare_parameter("frame", "world").value
        # offset (grados) para alinear "stick arriba" con "arriba en TU camara".
        # Si arriba no es el +X del mundo, prueba 90/180/-90 hasta que cuadre.
        self.view_yaw = math.radians(
            self.declare_parameter("view_yaw_deg", 0.0).value)
        self.yaw = None  # heading del robot (de odom), para el modo world
        # botones (mapeo SDL del DualSense en esta maquina: L1=9, R1=10; los
        # gatillos L2/R2 son EJES, no botones). R1=agarrar/cerrar, L1=soltar/abrir.
        self.btn_grasp = self.declare_parameter("button_grasp", 10).value    # R1
        self.btn_release = self.declare_parameter("button_release", 9).value  # L1
        # turbo desactivado por defecto (-1); R2 es eje aqui. Pon un indice de
        # boton valido para activarlo.
        self.btn_turbo = self.declare_parameter("button_turbo", -1).value
        self.turbo = self.declare_parameter("turbo_mult", 2.0).value

        self.finger_open = self.declare_parameter("finger_open", 0.04).value
        self.finger_close = self.declare_parameter("finger_close", 0.0).value
        # log el indice de cada boton al pulsarlo (para averiguar el mapeo de TU
        # mando: pulsa L1/R1 y mira el numero; luego pon button_grasp/release)
        self.debug_buttons = self.declare_parameter("debug_buttons", True).value

        self.cmd_pub = self.create_publisher(Twist, f"/{self.ns}/cmd_vel", 10)
        self.grasp_pub = self.create_publisher(Empty, f"/{self.ns}/gripper/grasp", 10)
        self.release_pub = self.create_publisher(Empty, f"/{self.ns}/gripper/release", 10)
        # control directo de los dedos (para que los botones muevan la pinza
        # aunque grasp_manager no este corriendo)
        self.traj_pub = self.create_publisher(
            JointTrajectory, f"/{self.ns}/gripper_controller/joint_trajectory", 10)
        self.prev = []
        self.create_subscription(Joy, "/joy", self._joy, 10)
        self.create_subscription(Odometry, f"/{self.ns}/odom", self._odom, 10)
        self.get_logger().info(
            f"ps5_teleop -> /{self.ns}. Stick izq: avanzar/strafe | stick der: girar "
            f"| R1(btn {self.btn_grasp}): agarrar/cerrar | L1(btn {self.btn_release}): "
            f"soltar/abrir | R2: turbo. Si no responde, pulsa los botones y mira el "
            f"log 'boton N' para ajustar button_grasp/button_release.")

    def _move_fingers(self, pos):
        jt = JointTrajectory()
        jt.joint_names = ["finger_left_joint", "finger_right_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = [float(pos), float(pos)]
        pt.time_from_start.sec = 1
        jt.points = [pt]
        self.traj_pub.publish(jt)

    def _odom(self, msg):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _ax(self, axes, i):
        if not 0 <= i < len(axes):
            return 0.0
        v = axes[i]
        return 0.0 if abs(v) < self.deadzone else v

    def _edge(self, buttons, i):
        cur = buttons[i] if 0 <= i < len(buttons) else 0
        prv = self.prev[i] if 0 <= i < len(self.prev) else 0
        return cur and not prv

    def _joy(self, msg):
        mult = self.turbo if (0 <= self.btn_turbo < len(msg.buttons)
                              and msg.buttons[self.btn_turbo]) else 1.0
        # entradas normalizadas (-1..1): adelante (stick arriba) y strafe
        fwd = self.inv_fwd * self._ax(msg.axes, self.ax_fwd)
        stf = self.inv_str * self._ax(msg.axes, self.ax_str)
        yaw_in = self.inv_yaw * self._ax(msg.axes, self.ax_yaw)

        t = Twist()
        if self.frame == "world" and self.yaw is not None:
            # Orientado al campo: el stick define velocidad en una direccion FIJA
            # de pantalla. "arriba" de pantalla = angulo view_yaw en el mundo;
            # "derecha" = view_yaw - 90.
            wx = fwd * math.cos(self.view_yaw) + stf * math.sin(self.view_yaw)
            wy = fwd * math.sin(self.view_yaw) - stf * math.cos(self.view_yaw)
            # rotar la velocidad mundo al frame del robot (cmd_vel es body)
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            t.linear.x = (wx * c + wy * s) * self.max_lin * mult
            t.linear.y = (-wx * s + wy * c) * self.max_lin * mult
        else:
            # Frame del robot (morro) — o world sin odom todavia.
            t.linear.x = fwd * self.max_lin * mult
            t.linear.y = stf * self.max_str * mult
        t.angular.z = yaw_in * self.max_yaw * mult
        self.cmd_pub.publish(t)

        # diagnostico: avisa el indice de cualquier boton recien pulsado
        if self.debug_buttons:
            for i in range(len(msg.buttons)):
                if self._edge(msg.buttons, i):
                    self.get_logger().info(f"boton {i} pulsado")

        if self._edge(msg.buttons, self.btn_grasp):
            self._move_fingers(self.finger_close)   # cerrar dedos
            self.grasp_pub.publish(Empty())          # + agarrar (si grasp_manager corre)
            self.get_logger().info("grasp (cerrar)")
        if self._edge(msg.buttons, self.btn_release):
            self._move_fingers(self.finger_open)     # abrir dedos
            self.release_pub.publish(Empty())        # + soltar
            self.get_logger().info("release (abrir)")
        self.prev = list(msg.buttons)


def main():
    rclpy.init()
    node = PS5Teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
