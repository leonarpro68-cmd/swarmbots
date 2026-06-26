#!/usr/bin/env python3
"""Go-to-goal reactivo con evasion de obstaculos por LiDAR para un Summit XLS.

Controlador de campos potenciales HOLONOMICO (aprovecha el mecanum: manda vx,
vy y wz). El vector de mando es la suma de:
  - atraccion hacia el objetivo (goal en frame MUNDO), y
  - repulsion de cada rayo del /scan mas cercano que `influence_radius`.
El resultado (en frame mundo) se rota al frame del cuerpo via el yaw de la
odometria y se publica como Twist en `cmd_vel`. Ademas el robot gira despacio
para encarar la direccion de avance, manteniendo el FOV del LiDAR (270 deg)
mirando hacia donde va.

CLAVE: el OdometryPublisher de Harmonic inicializa la odom en la POSE MUNDIAL
del spawn, asi que `/summitN/odom` esta en frame mundo y todos los robots
comparten el mismo marco. Por eso un goal mundial unico vale para todo el
enjambre sin SLAM ni TF compartido.

Modos (parametro implicito via `follow_robot`):
  - follow_robot == ""  -> va al punto fijo (goal_x, goal_y).
  - follow_robot != ""  -> persigue la odom de ese robot (lider) manteniendo
    `standoff` metros de distancia. Asi se hace el lider-seguidor sin mapa.

FORMACION EN ANILLO (modo goal): si varios summits comparten el mismo goal,
apuntar TODOS al mismo punto exacto hace que el anti-colision (repulsion del
LiDAR) les impida amontonarse y se queden oscilando sin converger. Por eso cada
robot no va al centro sino a su propio slot en un ANILLO alrededor del goal:
angulo `2*pi*robot_index/n_robots` y radio `formation_radius` (si es 0 se calcula
de `formation_spacing` para que los vecinos queden a esa distancia). Asi cada uno
tiene un destino propio y libre, y el enjambre rodea el punto sin bailar. Con
n_robots==1 el radio es 0 (va al punto exacto).
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan


def _yaw_from_quat(q) -> float:
    """Yaw (rad) de un geometry_msgs/Quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _wrap(a: float) -> float:
    """Normaliza un angulo a (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class GoToGoal(Node):
    def __init__(self):
        super().__init__("go_to_goal")

        # --- Parametros ---
        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("follow_robot", "")   # "" = punto fijo; si no, ns del lider
        self.declare_parameter("standoff", 1.5)       # m a mantener del lider
        self.declare_parameter("robot_index", 0)      # indice de este robot en el enjambre
        self.declare_parameter("n_robots", 1)         # nº total de summits (para el anillo)
        self.declare_parameter("formation_spacing", 2.0)  # m entre vecinos (> influence para no repelerse en reposo)
        self.declare_parameter("formation_radius", 0.0)   # m; 0 = auto desde el spacing
        self.declare_parameter("max_lin_vel", 0.5)    # m/s
        self.declare_parameter("max_ang_vel", 1.0)    # rad/s
        self.declare_parameter("goal_tol", 0.3)       # m para dar el goal por alcanzado
        self.declare_parameter("slow_radius", 1.2)    # m: dentro de esto frena suave
        self.declare_parameter("influence_radius", 1.5)  # m: alcance de la repulsion
        self.declare_parameter("k_rep", 0.30)         # ganancia de repulsion
        self.declare_parameter("safety_dist", 0.40)   # m: por debajo, corta el avance frontal
        self.declare_parameter("k_yaw", 1.0)          # ganancia del giro hacia el avance
        self.declare_parameter("control_rate", 20.0)  # Hz
        self.declare_parameter("goal_topic", "/goal_pose")  # RViz "2D Goal Pose"

        self.goal_x = self.get_parameter("goal_x").value
        self.goal_y = self.get_parameter("goal_y").value
        self.follow_robot = self.get_parameter("follow_robot").value
        self.standoff = self.get_parameter("standoff").value
        self.robot_index = self.get_parameter("robot_index").value
        self.n_robots = max(1, self.get_parameter("n_robots").value)
        spacing = self.get_parameter("formation_spacing").value
        radius = self.get_parameter("formation_radius").value
        # Offset (frame mundo) de este robot respecto al centro del goal: su slot
        # en el anillo. Radio: el dado, o el que hace que los vecinos queden a
        # `spacing` m. Con un solo robot el offset es 0 (va al punto exacto).
        if self.n_robots > 1:
            if radius <= 0.0:
                radius = spacing / (2.0 * math.sin(math.pi / self.n_robots))
            ang = 2.0 * math.pi * self.robot_index / self.n_robots
            self.off_x = radius * math.cos(ang)
            self.off_y = radius * math.sin(ang)
        else:
            self.off_x = self.off_y = 0.0
        self.max_lin = self.get_parameter("max_lin_vel").value
        self.max_ang = self.get_parameter("max_ang_vel").value
        self.goal_tol = self.get_parameter("goal_tol").value
        self.declare_parameter("arrive_hysteresis", 0.6)  # m extra a recorrer antes de re-activar
        self.arrive_hyst = self.get_parameter("arrive_hysteresis").value
        self.slow_radius = self.get_parameter("slow_radius").value
        self.influence = self.get_parameter("influence_radius").value
        self.k_rep = self.get_parameter("k_rep").value
        self.safety = self.get_parameter("safety_dist").value
        self.k_yaw = self.get_parameter("k_yaw").value
        rate = self.get_parameter("control_rate").value

        # --- Estado ---
        self.pose = None          # (x, y, yaw) propio en frame mundo
        self.scan = None          # ultimo LaserScan
        self.leader_pose = None   # (x, y) del lider en frame mundo
        self._reached_logged = False
        self.arrived = False      # latch: parado en el slot hasta que el goal se mueva

        # --- I/O (el nodo corre dentro del namespace del robot) ---
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "scan", self._on_scan, sensor_qos)
        if self.follow_robot:
            self.create_subscription(
                Odometry, f"/{self.follow_robot}/odom", self._on_leader_odom, 10
            )
        # Goal interactivo desde RViz (herramienta "2D Goal Pose" -> /goal_pose).
        # Topico absoluto: una sola publicacion reubica a todo el enjambre.
        goal_topic = self.get_parameter("goal_topic").value
        self.create_subscription(PoseStamped, goal_topic, self._on_goal, 10)

        self.timer = self.create_timer(1.0 / rate, self._control_step)
        tgt = f"sigue a '{self.follow_robot}' (standoff {self.standoff} m)" if self.follow_robot \
            else f"goal ({self.goal_x:.2f}, {self.goal_y:.2f})"
        self.get_logger().info(f"go_to_goal activo: {tgt}")

    # --- Callbacks ---
    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, _yaw_from_quat(p.orientation))

    def _on_leader_odom(self, msg: Odometry):
        self.leader_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_scan(self, msg: LaserScan):
        self.scan = msg

    def _on_goal(self, msg: PoseStamped):
        # El frame map/odom/world coinciden (la odom esta en frame mundo), asi
        # que x,y se toman como coordenadas mundo directamente.
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self._reached_logged = False
        self.arrived = False   # nuevo goal -> salir del latch y volver a moverse
        self.get_logger().info(
            f"nuevo goal: ({self.goal_x:.2f}, {self.goal_y:.2f}) [{msg.header.frame_id}]"
        )

    # --- Bucle de control ---
    def _control_step(self):
        if self.pose is None:
            return
        x, y, yaw = self.pose

        # Objetivo (frame mundo): slot propio en el anillo del goal, o lider.
        if self.follow_robot:
            if self.leader_pose is None:
                return
            gx, gy = self.leader_pose
        else:
            # Centro + offset del anillo -> destino propio (evita que todos
            # peleen por el mismo punto y bailen).
            gx, gy = self.goal_x + self.off_x, self.goal_y + self.off_y

        dgx, dgy = gx - x, gy - y
        dist = math.hypot(dgx, dgy)

        # ¿Alcanzado? Para el seguidor el "alcanzado" es estar a standoff.
        # LATCH con histeresis: al llegar al slot, quedarse QUIETO y no
        # re-activarse hasta que el objetivo se aleje > goal_tol + histeresis.
        # Sin esto el robot entra/sale de la tolerancia y "baila" en el sitio.
        arrive_d = self.standoff if self.follow_robot else self.goal_tol
        if self.arrived:
            if dist > arrive_d + self.arrive_hyst:
                self.arrived = False          # el goal se movio -> reactivar
            else:
                self.cmd_pub.publish(Twist())  # frenar y mantener
                return
        elif dist <= arrive_d:
            self.arrived = True
            self.cmd_pub.publish(Twist())
            if not self.follow_robot and not self._reached_logged:
                self.get_logger().info(f"goal alcanzado (dist {dist:.2f} m), me quedo quieto")
                self._reached_logged = True
            return
        else:
            self._reached_logged = False

        # Atraccion: vector unidad al goal, frenando dentro de slow_radius.
        att_mag = self.max_lin * min(1.0, max(0.0, (dist - arrive_d)) / self.slow_radius)
        att_wx = (dgx / dist) * att_mag
        att_wy = (dgy / dist) * att_mag

        # Repulsion (en frame del cuerpo) a partir del scan.
        rep_bx, rep_by, min_front = self._repulsion()

        # Atraccion mundo -> cuerpo.
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        att_bx = cos_y * att_wx + sin_y * att_wy
        att_by = -sin_y * att_wx + cos_y * att_wy

        vx = att_bx + rep_bx
        vy = att_by + rep_by

        # Limitar modulo a max_lin.
        speed = math.hypot(vx, vy)
        if speed > self.max_lin:
            vx *= self.max_lin / speed
            vy *= self.max_lin / speed

        # Parada de seguridad: obstaculo muy cerca al frente -> no avanzar (deja
        # que la repulsion lateral y el giro lo saquen).
        if min_front < self.safety and vx > 0.0:
            vx = 0.0

        # Giro: encarar la direccion de avance (mantiene el FOV mirando al frente).
        wz = 0.0
        if math.hypot(vx, vy) > 0.05:
            wz = max(-self.max_ang, min(self.max_ang, self.k_yaw * math.atan2(vy, vx)))

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)

    def _repulsion(self):
        """Suma de repulsiones de los rayos < influence_radius, en frame cuerpo.

        Devuelve (rep_x, rep_y, min_front) donde min_front es la distancia
        minima en el sector frontal (+-30 deg)."""
        rep_x = rep_y = 0.0
        min_front = float("inf")
        scan = self.scan
        if scan is None:
            return 0.0, 0.0, min_front

        ang = scan.angle_min
        rmin = scan.range_min
        for r in scan.ranges:
            a = ang
            ang += scan.angle_increment
            if not math.isfinite(r) or r < rmin:
                continue
            if abs(a) < math.radians(30.0):
                min_front = min(min_front, r)
            if r >= self.influence:
                continue
            # Gradiente de campo potencial repulsivo, empujando en sentido
            # opuesto al rayo (el obstaculo esta en la direccion 'a').
            w = self.k_rep * (1.0 / r - 1.0 / self.influence) / (r * r)
            rep_x -= w * math.cos(a)
            rep_y -= w * math.sin(a)
        return rep_x, rep_y, min_front


def main():
    rclpy.init()
    node = GoToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())  # frenar al salir
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
