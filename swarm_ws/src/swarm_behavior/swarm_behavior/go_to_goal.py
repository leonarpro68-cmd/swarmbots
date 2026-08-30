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
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty


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
        # Cuanto se atenua el giro cuando la repulsion domina (0 = girar siempre
        # como antes; 1 = no girar nada mientras esquiva, puro strafe mecanum).
        self.declare_parameter("yaw_damp", 1.0)
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
        self.yaw_damp = self.get_parameter("yaw_damp").value
        rate = self.get_parameter("control_rate").value

        # Cesion de paso por prioridad (para grupos que cruzan trayectorias):
        # este robot CEDE (se detiene) si algun peer de MAYOR prioridad esta
        # cerca y en movimiento. yield_peers = CSV de namespaces prioritarios.
        self.declare_parameter("yield_peers", "")
        self.declare_parameter("yield_radius", 3.0)   # m: distancia a la que se cede
        self.declare_parameter("yield_speed", 0.05)   # m/s: el peer cuenta si se mueve mas que esto
        self.declare_parameter("yield_speed_aside", 0.45)  # m/s al apartarse
        self.declare_parameter("use_goal_topic", True)  # escuchar /goal_pose (RViz)
        peers_csv = self.get_parameter("yield_peers").value
        self.yield_peers = [p for p in peers_csv.split(",") if p]
        self.yield_radius = self.get_parameter("yield_radius").value
        self.yield_speed = self.get_parameter("yield_speed").value
        self.yield_speed_aside = self.get_parameter("yield_speed_aside").value

        # --- Anti-arrastre: piezas invisibles al LiDAR ---
        # El scan va a z=0.557 m y las cajas de basura miden 0.20 m: el LiDAR NO
        # las ve. Mientras el robot transporta una pieza (soldada ~0.48 m por
        # delante, a ras de suelo) barreria las demas. El central_planner, que
        # si las ve por ground-truth, las publica en `avoid_points` (frame
        # mundo) y aqui se convierten en repulsores VIRTUALES. Lista vacia (el
        # robot no carga nada) => cero efecto, navegacion como siempre.
        self.declare_parameter("k_avoid", 0.35)        # ganancia
        self.declare_parameter("avoid_radius", 1.0)    # m: alcance del campo
        self.declare_parameter("avoid_min_dist", 0.30) # m: satura por debajo
        # Punto de la pieza transportada (frame cuerpo). La distancia se mide al
        # SEGMENTO centro_robot -> pieza, que es el volumen que realmente barre.
        self.declare_parameter("carry_forward", 0.48)
        # Componente TANGENCIAL: sin ella, un repulsor justo enfrente empuja
        # hacia atras y el robot se para en seco contra la meta. Con ella el
        # campo circula alrededor de la pieza y el mecanum la RODEA de lado.
        self.declare_parameter("avoid_swirl", 1.2)
        self.declare_parameter("max_avoid", 0.45)      # m/s: tope del termino

        # Retroceso puntual (al soltar en el deposito): un reverso RECTO de
        # retreat_dist m manteniendo el rumbo (vx<0, wz=0, sin girar). Aparta el
        # gripper de la pieza recien dejada para que, al girar hacia la siguiente
        # basura, no la barra/empuje fuera del deposito. Se dispara publicando en
        # /<ns>/retreat (el central_planner lo hace tras el release).
        self.declare_parameter("retreat_dist", 0.15)
        self.declare_parameter("retreat_speed", 0.2)
        self.k_avoid = self.get_parameter("k_avoid").value
        self.avoid_radius = self.get_parameter("avoid_radius").value
        self.avoid_min_dist = self.get_parameter("avoid_min_dist").value
        self.carry_forward = self.get_parameter("carry_forward").value
        self.avoid_swirl = self.get_parameter("avoid_swirl").value
        self.max_avoid = self.get_parameter("max_avoid").value
        self.avoid_points = []    # [(x, y)] en frame mundo; vacio = inactivo
        self.retreat_dist = self.get_parameter("retreat_dist").value
        self.retreat_speed = self.get_parameter("retreat_speed").value

        # --- Estado ---
        self.pose = None          # (x, y, yaw) propio en frame mundo
        self.scan = None          # ultimo LaserScan
        self.leader_pose = None   # (x, y) del lider en frame mundo
        self._reached_logged = False
        self.arrived = False      # latch: parado en el slot hasta que el goal se mueva
        self.peer_state = {}      # ns -> (x, y, speed) de peers prioritarios
        self._yield_logged = False
        self.retreating = False   # en medio de un reverso puntual
        self.retreat_start = None # (x, y) donde empezo el reverso
        self.align_active = False
        self.align_cmd = Twist()
        self.align_cmd_time = None
        self.declare_parameter("align_cmd_timeout", 0.35)
        # Durante la alineacion el objeto objetivo debe entrar en la pinza. Un
        # clearance normal (0.55 m) lo interpretaba como obstaculo y hacia
        # imposible llegar a desired_depth=0.065 m. La camara confirma el blanco
        # y la velocidad final esta limitada a 0.05 m/s. Este umbral queda por
        # debajo de la cara objetivo para permitir completar la insercion.
        self.declare_parameter("align_safety_dist", 0.045)
        self.align_cmd_timeout = self.get_parameter("align_cmd_timeout").value
        self.align_safety_dist = self.get_parameter("align_safety_dist").value

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
        # Topico absoluto: una sola publicacion reubica a todo el enjambre. En
        # modo "pares" (cada grupo a su meta) se desactiva para que RViz no las pise.
        if self.get_parameter("use_goal_topic").value:
            goal_topic = self.get_parameter("goal_topic").value
            self.create_subscription(PoseStamped, goal_topic, self._on_goal, 10)

        # Orden de retroceso puntual (reverso recto) tras soltar en el deposito.
        self.create_subscription(Empty, "retreat", self._on_retreat, 10)
        self.create_subscription(PoseArray, "avoid_points", self._on_avoid_points, 10)
        self.create_subscription(Bool, "visual_grasp/active", self._on_align_active, 10)
        self.create_subscription(Twist, "visual_grasp/cmd_vel", self._on_align_cmd, 10)

        # Suscripcion a la odom de cada peer prioritario (para cederle el paso).
        for peer in self.yield_peers:
            self.create_subscription(
                Odometry, f"/{peer}/odom",
                lambda msg, ns=peer: self._on_peer(ns, msg), 10,
            )

        self.timer = self.create_timer(1.0 / rate, self._control_step)
        tgt = f"sigue a '{self.follow_robot}' (standoff {self.standoff} m)" if self.follow_robot \
            else f"goal ({self.goal_x:.2f}, {self.goal_y:.2f})"
        ceder = f", cede ante {self.yield_peers}" if self.yield_peers else ""
        self.get_logger().info(f"go_to_goal activo: {tgt}{ceder}")

    # --- Callbacks ---
    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, _yaw_from_quat(p.orientation))

    def _on_leader_odom(self, msg: Odometry):
        self.leader_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_peer(self, ns: str, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.peer_state[ns] = (p.x, p.y, math.hypot(v.x, v.y))

    def _nearest_priority_peer(self, x: float, y: float):
        """(px, py) del peer prioritario mas cercano que esta cerca Y en
        movimiento, o None. No se cede ante uno parado (ya llego o tambien cede)
        para no bloquearse mutuamente."""
        best = None
        best_d = self.yield_radius
        for px, py, spd in self.peer_state.values():
            d = math.hypot(px - x, py - y)
            if spd > self.yield_speed and d < best_d:
                best, best_d = (px, py), d
        return best

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

    def _on_retreat(self, _msg: Empty):
        # Iniciar (o reiniciar) un reverso recto de retreat_dist m desde aqui.
        if self.pose is not None:
            self.retreating = True
            self.retreat_start = (self.pose[0], self.pose[1])

    def _on_avoid_points(self, msg: PoseArray):
        self.avoid_points = [(p.position.x, p.position.y) for p in msg.poses]

    def _on_align_active(self, msg: Bool):
        self.align_active = msg.data
        if not msg.data:
            self.align_cmd = Twist()

    def _on_align_cmd(self, msg: Twist):
        self.align_cmd = msg
        self.align_cmd_time = self.get_clock().now()

    # --- Bucle de control ---
    def _control_step(self):
        if self.pose is None:
            return
        x, y, yaw = self.pose

        # Retroceso puntual: reverso RECTO (vx<0 en frame cuerpo, wz=0 -> sin
        # girar) hasta recorrer retreat_dist. Preempta todo (goal, cesion) para
        # apartar el gripper de la pieza recien soltada antes de girar hacia la
        # siguiente. Al terminar, la nav normal retoma (rumbo intacto).
        if self.retreating and self.retreat_start is not None:
            moved = math.hypot(x - self.retreat_start[0], y - self.retreat_start[1])
            if moved < self.retreat_dist:
                cmd = Twist()
                cmd.linear.x = -self.retreat_speed
                self.cmd_pub.publish(cmd)
                return
            self.retreating = False
            self.cmd_pub.publish(Twist())  # frenar al acabar el reverso

        # La alineacion RGB-D preempta la navegacion global, pero go_to_goal
        # sigue siendo el UNICO publicador efectivo de cmd_vel. Un comando
        # vencido o un obstaculo LiDAR en la direccion de movimiento => STOP.
        if self.align_active:
            if self.align_cmd_time is None or (
                    self.get_clock().now() - self.align_cmd_time).nanoseconds / 1e9 \
                    > self.align_cmd_timeout:
                self.cmd_pub.publish(Twist())
                return
            cmd = Twist()
            cmd.linear.x = self.align_cmd.linear.x
            cmd.linear.y = self.align_cmd.linear.y
            cmd.angular.z = self.align_cmd.angular.z
            if not self._direction_clear(cmd.linear.x, cmd.linear.y,
                                         self.align_safety_dist):
                self.cmd_pub.publish(Twist())
                return
            self.cmd_pub.publish(cmd)
            return

        # Cesion de paso: si un grupo prioritario pasa cerca, APARTARSE de su
        # linea (no solo frenar: frenar en medio del pasillo bloquea al
        # prioritario -> deadlock). Se hace strafe perpendicular a la direccion
        # hacia el peer (aprovecha el mecanum) + repulsion para no chocar muros,
        # sin atraccion al goal. Al alejarse el prioritario, se retoma la nav.
        peer = self._nearest_priority_peer(x, y)
        if peer is not None:
            px, py = peer
            bx, by = px - x, py - y
            b = math.hypot(bx, by) or 1.0
            # Perpendicular a la linea hacia el peer + componente de alejamiento.
            perp_wx, perp_wy = -by / b, bx / b
            wvx = self.yield_speed_aside * (perp_wx - 0.4 * bx / b)
            wvy = self.yield_speed_aside * (perp_wy - 0.4 * by / b)
            rep_bx, rep_by, _, _ = self._repulsion()
            cos_y, sin_y = math.cos(yaw), math.sin(yaw)
            vx = cos_y * wvx + sin_y * wvy + rep_bx
            vy = -sin_y * wvx + cos_y * wvy + rep_by
            speed = math.hypot(vx, vy)
            if speed > self.max_lin:
                vx *= self.max_lin / speed
                vy *= self.max_lin / speed
            cmd = Twist()
            cmd.linear.x = vx
            cmd.linear.y = vy
            self.cmd_pub.publish(cmd)
            if not self._yield_logged:
                self.get_logger().info("cediendo el paso (apartandose) a un grupo prioritario")
                self._yield_logged = True
            return
        self._yield_logged = False

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
        rep_bx, rep_by, min_front, front_angle = self._repulsion()

        # Atraccion mundo -> cuerpo.
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        att_bx = cos_y * att_wx + sin_y * att_wy
        att_by = -sin_y * att_wx + cos_y * att_wy

        # Repulsion virtual de las piezas que no hay que barrer (solo mientras
        # se transporta: si la lista esta vacia devuelve 0,0).
        avo_bx, avo_by = self._virtual_repulsion(x, y, yaw, att_bx, att_by)

        vx = att_bx + rep_bx + avo_bx
        vy = att_by + rep_by + avo_by

        # Limitar modulo a max_lin.
        speed = math.hypot(vx, vy)
        if speed > self.max_lin:
            vx *= self.max_lin / speed
            vy *= self.max_lin / speed

        # Seguridad: obstaculo muy cerca al frente. Antes esto hacia vx=0, que
        # anula TODO el avance aunque el robot pudiera pasar de lado: el robot
        # se clavaba, el giro (wz = k_yaw*atan2(vy,0) = +-90 deg) lo hacia rotar
        # sobre si mismo y salia describiendo un arco lento. Ahora se elimina
        # SOLO la componente de velocidad que se acerca al obstaculo (proyeccion
        # sobre la normal) y se conserva la tangencial: el mecanum se DESLIZA
        # rozando el obstaculo a velocidad plena. La garantia es la misma que
        # antes (velocidad de acercamiento nula), pero sin frenazo.
        if min_front < self.safety:
            nx, ny = math.cos(front_angle), math.sin(front_angle)
            approach = vx * nx + vy * ny
            if approach > 0.0:
                vx -= approach * nx
                vy -= approach * ny

        # Giro: encarar la direccion de avance (mantiene el FOV mirando al
        # frente). Mientras esquiva NO conviene girar: el mecanum puede
        # strafear de lado a velocidad plena, y encarar cada desvio convierte el
        # esquive en un arco lento. Se atenua la ganancia segun cuanto domine la
        # repulsion sobre la atraccion (f=0 en campo abierto => identico a antes;
        # f=1 con el obstaculo mandando => no gira, puro strafe).
        att_mag_b = math.hypot(att_bx, att_by)
        rep_mag_b = math.hypot(rep_bx, rep_by)
        f = rep_mag_b / (rep_mag_b + att_mag_b + 1e-6)
        k_yaw_eff = self.k_yaw * max(0.0, 1.0 - self.yaw_damp * f)
        wz = 0.0
        if math.hypot(vx, vy) > 0.05:
            wz = max(-self.max_ang, min(self.max_ang, k_yaw_eff * math.atan2(vy, vx)))

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)

    def _repulsion(self):
        """Suma de repulsiones de los rayos < influence_radius, en frame cuerpo.

        Devuelve (rep_x, rep_y, min_front, front_angle): min_front es la
        distancia minima en el sector frontal (+-30 deg) y front_angle el
        angulo de ese rayo (direccion al obstaculo mas cercano, frame cuerpo),
        que usa la parada de seguridad para deslizarse en vez de clavarse."""
        rep_x = rep_y = 0.0
        min_front = float("inf")
        front_angle = 0.0
        scan = self.scan
        if scan is None:
            return 0.0, 0.0, min_front, front_angle

        ang = scan.angle_min
        rmin = scan.range_min
        for r in scan.ranges:
            a = ang
            ang += scan.angle_increment
            if not math.isfinite(r) or r < rmin:
                continue
            if abs(a) < math.radians(30.0) and r < min_front:
                min_front = r
                front_angle = a
            if r >= self.influence:
                continue
            # Gradiente de campo potencial repulsivo, empujando en sentido
            # opuesto al rayo (el obstaculo esta en la direccion 'a').
            w = self.k_rep * (1.0 / r - 1.0 / self.influence) / (r * r)
            rep_x -= w * math.cos(a)
            rep_y -= w * math.sin(a)
        return rep_x, rep_y, min_front, front_angle

    def _virtual_repulsion(self, x, y, yaw, att_bx, att_by):
        """Repulsion (frame cuerpo) de los `avoid_points` del planner.

        La distancia se mide al SEGMENTO centro_robot -> pieza transportada
        (carry_forward por delante), que es el volumen que de verdad barre el
        robot: una caja a 0.5 m del centro puede estar a 0 m de la pieza que
        lleva. Al termino radial se le suma uno TANGENCIAL (swirl) para rodear
        en vez de frenar; el lado se elige para no ir contra la meta.
        """
        if not self.avoid_points:
            return 0.0, 0.0
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        rep_x = rep_y = 0.0
        for (px, py) in self.avoid_points:
            # Punto en frame cuerpo.
            dx, dy = px - x, py - y
            bx = cos_y * dx + sin_y * dy
            by = -sin_y * dx + cos_y * dy
            # Distancia al segmento [0,0] -> [carry_forward, 0] (eje X cuerpo).
            t = min(max(bx, 0.0), self.carry_forward)
            ex, ey = bx - t, by
            d = math.hypot(ex, ey)
            if d >= self.avoid_radius:
                continue
            d_eff = max(d, self.avoid_min_dist)
            w = self.k_avoid * (1.0 / d_eff - 1.0 / self.avoid_radius)
            # Unitario del robot HACIA la pieza (desde el punto mas cercano).
            ux, uy = (ex / d, ey / d) if d > 1e-6 else (1.0, 0.0)
            # Radial: alejarse. Tangencial: rodear por el lado que no pelea con
            # la atraccion (si la pieza queda a la izquierda del avance, se
            # esquiva por la derecha).
            cross = att_bx * uy - att_by * ux
            sgn = -1.0 if cross > 0.0 else 1.0
            tx, ty = -uy * sgn, ux * sgn
            rep_x += w * (-ux + self.avoid_swirl * tx)
            rep_y += w * (-uy + self.avoid_swirl * ty)
        mag = math.hypot(rep_x, rep_y)
        if mag > self.max_avoid:
            rep_x *= self.max_avoid / mag
            rep_y *= self.max_avoid / mag
        return rep_x, rep_y

    def _direction_clear(self, vx, vy, clearance):
        """Comprueba el sector hacia el que se movera el mecanum.

        A diferencia del chequeo frontal normal, cubre tambien strafe. Los
        rayos invalidos nunca autorizan movimiento: si aun no hay scan, para.
        """
        if self.scan is None or math.hypot(vx, vy) < 1e-4:
            return self.scan is not None
        direction = math.atan2(vy, vx)
        half_sector = math.radians(28.0)
        angle = self.scan.angle_min
        seen = False
        for distance in self.scan.ranges:
            if abs(_wrap(angle - direction)) <= half_sector:
                if math.isfinite(distance) and distance >= self.scan.range_min:
                    seen = True
                    if distance < clearance:
                        return False
            angle += self.scan.angle_increment
        return seen


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
