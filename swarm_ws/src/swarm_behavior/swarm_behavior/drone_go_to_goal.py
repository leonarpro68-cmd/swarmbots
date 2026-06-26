#!/usr/bin/env python3
"""Go-to-goal en 3D para un dron X3 del enjambre.

Comparte el mismo goal (x, y) que los Summit: escucha la herramienta "2D Goal
Pose" de RViz (`/goal_pose`) y vuela hasta ese punto manteniendo una ALTITUD de
crucero fija. Asi el mismo clic en RViz mueve summits (en el suelo, esquivando
obstaculos por LiDAR) y drones (sobrevolando el punto).

El dron NO lleva LiDAR -> no hay evasion reactiva; vuela por encima de los
obstaculos (cruise_z >> altura de estanterias). Para que varios drones no
colisionen, cada uno cruza a una altura distinta (capas: `cruise_z` por dron).

Frames (verificado empiricamente, 2026-06-25): `/droneN/cmd_vel` es velocidad en
el FRAME DEL CUERPO (yaw incluido); z = subir/bajar. Igual que el Summit, se
rota el error mundo->cuerpo via el yaw de la odometria. La odom del dron, como la
del summit, esta en frame MUNDO (OdometryPublisher arranca en la pose de spawn),
asi que goal y odom comparten marco sin TF ni SLAM. Twist cero = hover (el mando
persiste, por eso siempre se republica vz para sostener la altura).
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def _yaw_from_quat(q) -> float:
    """Yaw (rad) de un geometry_msgs/Quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class DroneGoToGoal(Node):
    def __init__(self):
        super().__init__("drone_go_to_goal")

        # --- Parametros ---
        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("cruise_z", 3.5)        # m: altitud de crucero (capa de este dron)
        self.declare_parameter("max_lin_vel", 0.8)     # m/s horizontal
        self.declare_parameter("max_climb_vel", 0.7)   # m/s vertical
        self.declare_parameter("goal_tol", 0.4)        # m horizontal para dar el goal por alcanzado
        self.declare_parameter("slow_radius", 1.5)     # m: dentro de esto frena suave
        self.declare_parameter("k_z", 0.8)             # ganancia de altitud
        self.declare_parameter("control_rate", 20.0)   # Hz
        self.declare_parameter("goal_topic", "/goal_pose")  # RViz "2D Goal Pose"
        self.declare_parameter("use_goal_topic", True)  # escuchar /goal_pose (off en modo pares)

        self.goal_x = self.get_parameter("goal_x").value
        self.goal_y = self.get_parameter("goal_y").value
        self.cruise_z = self.get_parameter("cruise_z").value
        self.max_lin = self.get_parameter("max_lin_vel").value
        self.max_climb = self.get_parameter("max_climb_vel").value
        self.goal_tol = self.get_parameter("goal_tol").value
        self.slow_radius = self.get_parameter("slow_radius").value
        self.k_z = self.get_parameter("k_z").value
        rate = self.get_parameter("control_rate").value

        # --- Estado ---
        self.pose = None          # (x, y, z, yaw) propio en frame mundo
        self._reached_logged = False

        # --- I/O (el nodo corre dentro del namespace del dron) ---
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        # Goal interactivo desde RViz: topico absoluto, una publicacion mueve a
        # todo el enjambre (summits y drones). En modo pares se desactiva.
        if self.get_parameter("use_goal_topic").value:
            goal_topic = self.get_parameter("goal_topic").value
            self.create_subscription(PoseStamped, goal_topic, self._on_goal, 10)

        self.timer = self.create_timer(1.0 / rate, self._control_step)
        self.get_logger().info(
            f"drone_go_to_goal activo: goal ({self.goal_x:.2f}, {self.goal_y:.2f}) "
            f"@ z={self.cruise_z:.1f} m"
        )

    # --- Callbacks ---
    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, p.position.z,
                     _yaw_from_quat(p.orientation))

    def _on_goal(self, msg: PoseStamped):
        # El goal (x, y) se comparte con los summits; la altura la fija cruise_z.
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self._reached_logged = False
        self.get_logger().info(
            f"nuevo goal: ({self.goal_x:.2f}, {self.goal_y:.2f}) @ z={self.cruise_z:.1f} m"
        )

    # --- Bucle de control ---
    def _control_step(self):
        if self.pose is None:
            return
        x, y, z, yaw = self.pose

        # Vertical: siempre sostener la altitud de crucero (hover persiste, hay
        # que republicar vz aunque el horizontal este parado).
        vz = max(-self.max_climb, min(self.max_climb, self.k_z * (self.cruise_z - z)))

        dgx, dgy = self.goal_x - x, self.goal_y - y
        dist = math.hypot(dgx, dgy)

        if dist <= self.goal_tol:
            # Sobre el goal: parar horizontal, seguir sosteniendo altura.
            cmd = Twist()
            cmd.linear.z = vz
            self.cmd_pub.publish(cmd)
            if not self._reached_logged:
                self.get_logger().info(f"goal alcanzado (dist horiz {dist:.2f} m)")
                self._reached_logged = True
            return
        self._reached_logged = False

        # Atraccion horizontal (frame mundo), frenando dentro de slow_radius.
        att_mag = self.max_lin * min(1.0, max(0.0, dist - self.goal_tol) / self.slow_radius)
        att_wx = (dgx / dist) * att_mag
        att_wy = (dgy / dist) * att_mag

        # Mundo -> cuerpo (cmd_vel del dron es body frame, igual que el summit).
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        vx = cos_y * att_wx + sin_y * att_wy
        vy = -sin_y * att_wx + cos_y * att_wy

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = DroneGoToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())  # Twist cero = mantener velocidad nula = hover
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
