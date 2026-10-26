"""Alineacion final RGB-D para introducir una pieza en el gripper.

El planner entrega la pose de la pieza seleccionada. La odometria permite
predecir donde buscarla en la imagen de profundidad; la profundidad REAL
confirma el centro y la distancia antes de autorizar el agarre. Este nodo no
publica al motor: propone cmd_vel a go_to_goal, que mantiene la parada LiDAR.
"""
import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int8


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _near_field_geometry(pose, target, camera_forward,
                         min_range, max_range, max_lateral, max_camera_depth):
    """Cota geometrica, NO una lectura visual ni prueba de contacto.

    Solo se usa tras una historia positiva de RGB-D y con el robot detenido.
    Devuelve (apto, distancia_centro, lateral, profundidad_camara).
    """
    rx, ry, yaw = pose
    dx, dy = target[0] - rx, target[1] - ry
    forward = math.cos(yaw) * dx + math.sin(yaw) * dy
    lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
    camera_depth = forward - camera_forward
    distance = math.hypot(dx, dy)
    ok = (all(map(math.isfinite, (distance, lateral, camera_depth)))
          and min_range <= forward <= max_range
          and abs(lateral) <= max_lateral
          and 0.055 <= camera_depth <= max_camera_depth)
    return ok, distance, lateral, camera_depth


class VisualGrasp(Node):
    def __init__(self):
        super().__init__("visual_grasp")
        self.declare_parameter("horizontal_fov", 1.047)
        self.declare_parameter("camera_forward", 0.35)
        # La profundidad mide la cara trasera de la caja, no su centro. A
        # 0.060 m de la camara el centro queda ~0.470 m delante del robot: la
        # caja ocupa la zona util de los dedos y no queda prendida solo del tip.
        self.declare_parameter("desired_depth", 0.065)
        self.declare_parameter("depth_tol", 0.012)
        self.declare_parameter("pixel_tol", 26.0)
        self.declare_parameter("confirm_frames", 2)
        self.declare_parameter("max_forward", 0.05)
        self.declare_parameter("max_strafe", 0.08)
        # Aproximacion en DOS TRAMOS. El planner cede el control a ~0.95 m de la
        # pieza, pero la insercion real entre los dedos son solo los ultimos
        # centimetros: recorrer los ~0.48 m enteros a max_forward costaba ~9.5 s
        # (con timeout 12 s, sin margen para una correccion lateral).
        #   d > slow_depth  -> tramo LEJANO, rapido (far_forward / far_gain).
        #   d <= slow_depth -> tramo de INSERCION, intacto (max_forward + gain
        #                      proporcional 0.35 de siempre).
        # far_gain*(d-slow_depth)+max_forward es continuo en slow_depth: al
        # cruzar la frontera no hay salto de velocidad.
        self.declare_parameter("far_forward", 0.30)
        self.declare_parameter("slow_depth", 0.18)
        self.declare_parameter("far_gain", 1.2)
        # Suelo de velocidad: el termino proporcional del tramo de insercion
        # decae a ~0.012 m/s en los ultimos cm y ahi se iba la mitad del tiempo.
        # NO sube la velocidad de contacto (el techo sigue siendo max_forward),
        # solo evita el reptado asintotico. min_forward:=0.0 => comportamiento
        # exacto de antes.
        self.declare_parameter("min_forward", 0.03)
        self.declare_parameter("timeout", 15.0)
        self.declare_parameter("sensor_timeout", 0.5)
        # D.8.6: rescate opt-in ante perdida de profundidad cerca de la pinza.
        # NO ordena movimientos sin datos: valida odometria + historia RGB-D y
        # solamente entrega un OK provisional al grasp_manager, que debe
        # confirmar fisicamente el joint. Por defecto deshabilitado.
        self.declare_parameter("near_field_handoff_enabled", False)
        self.declare_parameter("near_field_min_range", 0.43)
        self.declare_parameter("near_field_max_range", 0.50)
        self.declare_parameter("near_field_max_lateral", 0.045)
        self.declare_parameter("near_field_max_camera_depth", 0.16)
        self.declare_parameter("near_field_depth_loss_s", 0.75)
        self.declare_parameter("near_field_recent_vision_s", 3.0)
        self.declare_parameter("near_field_vision_depth_limit", 0.46)
        self.fov = float(self.get_parameter("horizontal_fov").value)
        self.camera_forward = float(self.get_parameter("camera_forward").value)
        self.desired_depth = float(self.get_parameter("desired_depth").value)
        self.depth_tol = float(self.get_parameter("depth_tol").value)
        self.pixel_tol = float(self.get_parameter("pixel_tol").value)
        self.confirm_frames = int(self.get_parameter("confirm_frames").value)
        self.max_forward = float(self.get_parameter("max_forward").value)
        self.max_strafe = float(self.get_parameter("max_strafe").value)
        self.far_forward = float(self.get_parameter("far_forward").value)
        self.slow_depth = float(self.get_parameter("slow_depth").value)
        self.far_gain = float(self.get_parameter("far_gain").value)
        self.min_forward = float(self.get_parameter("min_forward").value)
        self.timeout = float(self.get_parameter("timeout").value)
        self.sensor_timeout = float(self.get_parameter("sensor_timeout").value)
        self.near_field_enabled = bool(self.get_parameter("near_field_handoff_enabled").value)
        self.near_min_range = float(self.get_parameter("near_field_min_range").value)
        self.near_max_range = float(self.get_parameter("near_field_max_range").value)
        self.near_max_lateral = float(self.get_parameter("near_field_max_lateral").value)
        self.near_max_camera_depth = float(self.get_parameter("near_field_max_camera_depth").value)
        self.near_depth_loss_s = float(self.get_parameter("near_field_depth_loss_s").value)
        self.near_recent_vision_s = float(self.get_parameter("near_field_recent_vision_s").value)
        self.near_vision_depth_limit = float(self.get_parameter("near_field_vision_depth_limit").value)
        if not (0.0 < self.near_min_range < self.near_max_range
                and 0.0 < self.near_max_lateral <= 0.10
                and 0.0 < self.near_depth_loss_s
                and 0.0 < self.near_recent_vision_s <= 5.0):
            raise ValueError('Parametros near_field invalidos')

        self.pose = None
        self.odom_stamp = None
        self.target = None
        self.depth = None
        self.depth_stamp = None
        self.started = None
        self.good_frames = 0
        self.active = False
        self.last_progress_log = None
        self.diag_last_xy = None
        self.diag_last_depth = None
        self.diag_last_progress_time = None
        self.diag_last_command = (0.0, 0.0)
        self.diag_stall_seconds = 3.0
        self.near_vision_history = deque(maxlen=100)
        self.near_missing_since = None

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.VOLATILE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=2)
        self.create_subscription(Odometry, "odom", self._odom, 10)
        self.create_subscription(Image, "camera/depth", self._depth, qos)
        self.create_subscription(PoseStamped, "visual_grasp/target", self._target, 10)
        self.cmd_pub = self.create_publisher(Twist, "visual_grasp/cmd_vel", 10)
        self.active_pub = self.create_publisher(Bool, "visual_grasp/active", 10)
        self.result_pub = self.create_publisher(Int8, "visual_grasp/result", 10)
        self.create_timer(0.05, self._step)

    def _odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, _yaw(q))
        self.odom_stamp = self.get_clock().now()

    def _depth(self, msg):
        if msg.encoding not in ("32FC1", "32FC"):
            self.get_logger().error(
                f"Profundidad con encoding no soportado: {msg.encoding}",
                throttle_duration_sec=5.0)
            return
        row_floats = msg.step // 4
        raw = np.frombuffer(msg.data, dtype=np.float32)
        if raw.size < row_floats * msg.height:
            return
        self.depth = raw[:row_floats * msg.height].reshape(msg.height, row_floats)[:, :msg.width]
        self.depth_stamp = self.get_clock().now()

    def _target(self, msg):
        self.target = (msg.pose.position.x, msg.pose.position.y)
        self.started = self.get_clock().now()
        self.good_frames = 0
        self.last_progress_log = None
        self.diag_last_xy = None
        self.diag_last_depth = None
        self.diag_last_progress_time = self.get_clock().now()
        self.diag_last_command = (0.0, 0.0)
        self.near_vision_history.clear()
        self.near_missing_since = None
        self.active = True
        self.active_pub.publish(Bool(data=True))

    def _finish(self, result):
        self.cmd_pub.publish(Twist())
        self.active = False
        self.active_pub.publish(Bool(data=False))
        self.result_pub.publish(Int8(data=result))

    def _nearfield_handoff(self, now):
        """Acepta una alineacion GEOMETRICA solo tras RGB-D previo consistente.

        Nunca publica velocidades no nulas. grasp_manager sigue siendo el
        unico que puede confirmar un DetachableJoint; si falla, D7 aborta.
        """
        if not self.near_field_enabled or self.pose is None or self.target is None:
            return False
        if self.odom_stamp is None:
            return False
        odom_age = (now - self.odom_stamp).nanoseconds / 1e9
        if not 0 <= odom_age <= self.sensor_timeout:
            return False
        good_geometry, distance, lateral, camera_depth = _near_field_geometry(
            self.pose, self.target, self.camera_forward, self.near_min_range,
            self.near_max_range, self.near_max_lateral, self.near_max_camera_depth)
        if not good_geometry:
            return False
        # El ultimo RGB-D valido debe ser reciente, centrado y de una
        # superficie razonablemente cercana; nunca solo una prediccion odom.
        recent = [(stamp, error, depth)
                  for stamp, error, depth in self.near_vision_history
                  if 0 <= (now.nanoseconds - stamp) / 1e9 <= self.near_recent_vision_s]
        if len(recent) < 3:
            return False
        # Tres muestras a lo largo de >= 0.12 s evitan un unico frame espurio.
        if (recent[-1][0] - recent[0][0]) / 1e9 < 0.12:
            return False
        self.get_logger().warn(
            f'RGBD_V2 NEAR_FIELD HANDOFF CONTROLADO: depth no disponible; '
            f'dist_centro={distance:.3f}m lateral={lateral:+.3f}m '
            f'camara_pred={camera_depth:.3f}m, '
            f'ult_rgbd={recent[-1][2]:.3f}m en {len(recent)} muestras. '
            'Robot detenido; alineacion geometrica provisional, NO joint confirmado.')
        self._finish(1)
        return True

    def _step(self):
        if not self.active:
            return
        now = self.get_clock().now()
        if self.started is None or (now - self.started).nanoseconds / 1e9 > self.timeout:
            self.get_logger().warn(
                f"RGBD_V2 TIMEOUT: objetivo={self.target}, pose={self.pose}, "
                f"profundidad_ultima={self.diag_last_depth}, "
                f"cmd_ultimo={self.diag_last_command}; no se agarra")
            self._finish(-1)
            return
        if self.pose is None or self.depth is None or self.depth_stamp is None or \
                (now - self.depth_stamp).nanoseconds / 1e9 > self.sensor_timeout:
            self.cmd_pub.publish(Twist())
            return

        rx, ry, yaw = self.pose
        dx, dy = self.target[0] - rx, self.target[1] - ry
        forward = math.cos(yaw) * dx + math.sin(yaw) * dy - self.camera_forward
        lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
        bearing = math.atan2(-lateral, max(forward, 1e-3))  # pixel +x = derecha
        if forward <= 0.0 or abs(bearing) > 0.48 * self.fov:
            self.get_logger().error(f"RGBD_V2 FOV: forward={forward:.3f} bearing={bearing:.3f}")
            self._finish(-1)
            return

        height, width = self.depth.shape
        focal = width / (2.0 * math.tan(self.fov / 2.0))
        expected_u = width * 0.5 + focal * math.tan(bearing)
        # Buscar la superficie de la caja cerca de su proyeccion prevista. La
        # banda vertical central evita usar mayormente el suelo.
        u0 = max(0, int(expected_u) - 55)
        u1 = min(width, int(expected_u) + 56)
        v0, v1 = int(height * 0.28), int(height * 0.78)
        roi = self.depth[v0:v1, u0:u1]
        expected_depth = math.hypot(forward, lateral)
        valid = np.isfinite(roi) & (roi > 0.05) & \
            (np.abs(roi - expected_depth) < 0.30)
        rows, cols = np.nonzero(valid)
        if cols.size < 20:
            self.get_logger().warn(
                f"RGBD_V2 SIN_DEPTH: pixeles={cols.size} "
                f"depth_pred={expected_depth:.3f}", throttle_duration_sec=2.0)
            self.good_frames = 0
            # Parada efectiva, sin avanzar a ciegas. Solo tras perdida sostenida
            # podemos considerar entrega geometrica al gestor del joint.
            self.cmd_pub.publish(Twist())
            if self.near_missing_since is None:
                self.near_missing_since = now
            elif ((now - self.near_missing_since).nanoseconds / 1e9
                  >= self.near_depth_loss_s):
                self._nearfield_handoff(now)
            return
        self.near_missing_since = None
        depths = roi[rows, cols]
        near = depths <= np.percentile(depths, 35.0)
        object_u = float(np.median(cols[near] + u0))
        object_depth = float(np.median(depths[near]))
        pixel_error = object_u - width * 0.5
        if (self.near_field_enabled and abs(pixel_error) <= self.pixel_tol
                and object_depth <= self.near_vision_depth_limit
                and abs(object_depth - expected_depth) <= 0.26):
            self.near_vision_history.append(
                (now.nanoseconds, pixel_error, object_depth))
        if self.diag_last_xy is None:
            self.diag_last_xy = (rx, ry)
            self.diag_last_depth = object_depth
            self.diag_last_progress_time = now
        elif (math.hypot(rx - self.diag_last_xy[0], ry - self.diag_last_xy[1]) >= 0.012
              or (self.diag_last_depth - object_depth) >= 0.012):
            self.diag_last_xy = (rx, ry)
            self.diag_last_depth = object_depth
            self.diag_last_progress_time = now
        elif (self.diag_last_progress_time is not None and
              (now - self.diag_last_progress_time).nanoseconds / 1e9 >= self.diag_stall_seconds):
            self.get_logger().warn(
                f"RGBD_V2 ESTANCAMIENTO: depth={object_depth:.3f}m "
                f"error_px={pixel_error:+.1f} cmd_anterior={self.diag_last_command} "
                f"pose_xy=({rx:.3f},{ry:.3f}); comprobar contacto fisico y cmd_vel",
                throttle_duration_sec=1.5)

        if self.last_progress_log is None or \
                (now - self.last_progress_log).nanoseconds / 1e9 >= 1.0:
            self.get_logger().info(
                f"Alineando: lateral={pixel_error:+.1f}px "
                f"profundidad={object_depth:.3f}m "
                f"confirmacion={self.good_frames}/{self.confirm_frames}")
            self.last_progress_log = now

        aligned = abs(pixel_error) <= self.pixel_tol
        # Una pieza algo mas cerca ya esta suficientemente insertada. Exigir
        # error absoluto dejaba el estado bloqueado para siempre si la inercia
        # cruzaba el limite inferior, porque por seguridad este controlador no
        # retrocede durante el agarre.
        at_depth = object_depth <= self.desired_depth + self.depth_tol
        if aligned and at_depth:
            self.good_frames += 1
            self.cmd_pub.publish(Twist())
            if self.good_frames >= self.confirm_frames:
                self.get_logger().info(
                    f"Pieza centrada por RGB-D (u={pixel_error:+.1f}px, z={object_depth:.3f}m)")
                self._finish(1)
            return
        self.good_frames = 0
        cmd = Twist()
        cmd.linear.y = float(np.clip(-0.0015 * pixel_error,
                                   -self.max_strafe, self.max_strafe))
        # No avanzar mientras el error lateral sea grande: evita rozar/tumbar
        # la caja con un dedo. Nunca retrocede por profundidad ruidosa.
        if abs(pixel_error) < 45.0 and object_depth > self.desired_depth + self.depth_tol:
            cmd.linear.x = self._forward_speed(object_depth)
        self.diag_last_command = (float(cmd.linear.x), float(cmd.linear.y))
        self.cmd_pub.publish(cmd)

    def _forward_speed(self, object_depth):
        """Velocidad de avance segun el tramo (lejano rapido / insercion lento).

        En el tramo de insercion (d <= slow_depth) el techo sigue siendo
        max_forward y la ley proporcional es la de siempre; solo se le pone un
        suelo (min_forward) para no reptar en los ultimos centimetros.
        """
        if object_depth > self.slow_depth:
            return min(self.far_forward,
                       self.far_gain * (object_depth - self.slow_depth)
                       + self.max_forward)
        speed = min(self.max_forward, 0.35 * (object_depth - self.desired_depth))
        return max(self.min_forward, speed)


def main():
    rclpy.init()
    node = VisualGrasp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # launch puede haber invalidado ya el contexto al propagar SIGINT.
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
