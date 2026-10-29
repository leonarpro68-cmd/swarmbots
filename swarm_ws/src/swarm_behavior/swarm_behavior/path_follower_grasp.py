#!/usr/bin/env python3
"""Seguidor CBS experimental: navegación, control RGB-D y preorientación local.

Entrada adicional: local_turn/target_yaw (std_msgs/Float64), radianes EN MARCO
DE ODOMETRÍA. Resultado: local_turn/result (std_msgs/Int8): 1 éxito, -1 fallo.
Solo este nodo publica cmd_vel. No modificar el seguidor CBS estable.
"""
from __future__ import annotations
import math
import time

from std_msgs.msg import String
from .swarm_planning.hybrid_refinement.timed_protocol import (
    TimedProtocolError, decode_command, build_status, reference,
)

import rclpy
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64, Int8

from .path_follower import PathFollower


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class GraspPathFollower(PathFollower):
    def __init__(self):
        super().__init__()
        self.declare_parameter('visual_cmd_timeout', 0.35)
        self.declare_parameter('turn_kp', 1.8)
        self.declare_parameter('turn_max_speed', 0.35)
        self.declare_parameter('turn_tolerance_rad', 0.055)
        self.declare_parameter('turn_timeout_s', 15.0)
        self.visual_timeout = float(self.get_parameter('visual_cmd_timeout').value)
        self.turn_kp = float(self.get_parameter('turn_kp').value)
        self.turn_max_speed = float(self.get_parameter('turn_max_speed').value)
        self.turn_tolerance = float(self.get_parameter('turn_tolerance_rad').value)
        self.turn_timeout = float(self.get_parameter('turn_timeout_s').value)
        if min(self.visual_timeout, self.turn_kp, self.turn_max_speed,
               self.turn_tolerance, self.turn_timeout) <= 0:
            raise ValueError('Parámetros de control visual y giro deben ser positivos')
        self.visual_active = False
        self.visual_cmd = Twist()
        self.visual_last_time = None
        self.turn_target = None
        self.turn_started = None
        self.turn_hold_started = None
        self.turn_result_pub = self.create_publisher(Int8, 'local_turn/result', 10)
        self.create_subscription(Float64, 'local_turn/target_yaw', self.on_turn_target, 10)
        self.create_subscription(Bool, 'visual_grasp/active', self.on_visual_active, 10)
        self.create_subscription(Twist, 'visual_grasp/cmd_vel', self.on_visual_cmd, 10)
        # Fase 5: NO creamos un segundo publicador de cmd_vel.
        # Este modo permanece deshabilitado para la misión v1.1 original.
        self.declare_parameter('hybrid_enabled', False)
        self.declare_parameter('hybrid_kp', 1.1)
        # Límite exclusivo para Bézier temporizada; no afecta D8/RGB-D ni el giro.
        self.declare_parameter('hybrid_timed_max_speed', 2.0)
        self.hybrid_enabled = bool(self.get_parameter('hybrid_enabled').value)
        self.hybrid_kp = float(self.get_parameter('hybrid_kp').value)
        self.hybrid_timed_max_speed = float(self.get_parameter('hybrid_timed_max_speed').value)
        if not math.isfinite(self.hybrid_timed_max_speed) or not 0 < self.hybrid_timed_max_speed <= 2.0:
            raise ValueError('hybrid_timed_max_speed debe estar en (0, 2.0]')
        self.timed_command = None
        self.timed_last_time = None
        self.timed_heartbeat_wall = None
        # Ventana breve solo para pausas de /clock mientras estamos ARMED.
        # RUNNING tolera pausas cortas de /clock; conserva STOP y timeout.
        self.timed_armed_clock_grace_s = 3.0
        self.timed_last_velocity = (0., 0.)
        self.timed_goal_since = None
        self.timed_running_reported = False
        self.timed_status_pub = self.create_publisher(String, 'hybrid_trajectory/status', 10)
        self.create_subscription(String, 'hybrid_trajectory/command', self.on_timed_command, 10)
        # Temporizador independiente de /clock: si Gazebo se congela, mandar STOP.
        self.create_timer(0.2, self._timed_wall_watchdog,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        if self.hybrid_kp <= 0 or not math.isfinite(self.hybrid_kp):
            raise ValueError('hybrid_kp inválido')
        self.get_logger().info('Seguidor experimental: RGB-D y preorientación local habilitados; '
                               f'hybrid_enabled={self.hybrid_enabled}')

    def _timed_wall_watchdog(self):
        """Supervisión STEADY_TIME independiente de /clock.

        Una interrupción temporal de /clock antes del inicio no debe consumir
        la trayectoria, pero el seguidor mantiene cmd_vel=0 durante la espera.
        Si /clock sí avanza y el lazo no se ejecuta, cancelamos a los 1 s.
        ARMED y RUNNING toleran pausas breves; bloqueos reales se cancelan.
        """
        cmd = self.timed_command
        hb = self.timed_heartbeat_wall
        if cmd is None or hb is None:
            return
        age = time.monotonic() - hb
        if age <= 1.0:
            return
        now_sim = self.get_clock().now().nanoseconds / 1e9
        last_step_sim = self.timed_last_time
        # El temporizador de control usa tiempo de simulacion y control_rate.
        # Un avance menor que dos periodos no prueba que el lazo este bloqueado:
        # a 20 Hz, 0.044 s de /clock todavia no son ni un ciclo completo.
        control_period_s = 1.0 / float(self.get_parameter('control_rate').value)
        sim_progress_s = (now_sim - last_step_sim
                          if last_step_sim is not None else 0.0)
        sim_advanced = (last_step_sim is not None
                        and sim_progress_s > 2.0 * control_period_s)
        if sim_advanced:
            # Simulación avanzando, pero el control dejó de atender el temporizador.
            self._timed_abort(
                f'CONTROL_LOOP_STALLED: control sin ciclo {age:.2f}s; '
                f'/clock avanzó {now_sim-last_step_sim:.3f}s')
            return
        # Si /clock no avanza, tolerar una pausa breve tanto
        # en ARMED como en RUNNING, manteniendo STOP.
        if age < self.timed_armed_clock_grace_s:
            self.stop()
            return

        self._timed_abort(
            f'CLOCK_STALLED: pausa de /clock/control de {age:.2f}s; '
            f'estado={"running" if self.timed_running_reported else "armed"}')

    def _timed_status(self, cmd, state, detail=''):
        self.timed_status_pub.publish(String(data=build_status(cmd.robot, cmd.session_id, state, detail)))

    def _timed_abort(self, detail, state='failed'):
        cmd = self.timed_command
        self.timed_command = None
        self.timed_last_velocity = (0., 0.)
        self.timed_last_time = None
        self.timed_heartbeat_wall = None
        self.timed_goal_since = None
        self.stop()
        if cmd is not None:
            self._timed_status(cmd, state, detail)
            self.get_logger().error(f'HÍBRIDO {state}: {detail}')

    def on_timed_command(self, msg):
        if not self.hybrid_enabled:
            return
        try:
            cmd = decode_command(msg.data)
        except TimedProtocolError as exc:
            self.get_logger().error(f'Orden híbrida inválida: {exc}')
            self._timed_abort('Mensaje malformado')
            return
        if cmd.robot != self.get_namespace().lstrip('/'):
            self.get_logger().warn('Orden híbrida para otro namespace ignorada')
            return
        if cmd.action == 'cancel':
            if self.timed_command is not None and self.timed_command.session_id == cmd.session_id:
                self._timed_abort('Cancelación de coordinador', state='cancelled')
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if self.timed_command is not None:
            self.get_logger().warn('Ya existe una trayectoria híbrida activa; rechazada')
            return
        if (self.visual_active or self.turn_target is not None or self.pose is None
                or self.odom_frame != cmd.frame_id or self.last_odom_time is None
                or not 0 <= now-self.last_odom_time <= self.odom_timeout
                or not 1 <= cmd.start_time_s-now <= 30):
            self._timed_status(cmd, 'failed', 'Modo ocupado, odometría/frame inválido o inicio fuera de ventana')
            return
        p0 = cmd.control_points[0][0]
        if math.dist(self.pose[:2], p0) > cmd.goal_tolerance_m:
            self._timed_status(cmd, 'failed', 'Robot no alineado con el inicio Bézier')
            return
        self.path=[]
        self.path_index=0
        self.stop()
        self.timed_command=cmd
        self.timed_last_time=now
        self.timed_heartbeat_wall=time.monotonic()
        self.timed_last_velocity=(0., 0.)
        self.timed_goal_since=None
        self.timed_running_reported=False
        self._timed_status(cmd, 'armed')
        self.get_logger().info(f'HÍBRIDO armado: {len(cmd.control_points)} piezas, '
                               f'inicio en {cmd.start_time_s-now:.1f}s')

    def _timed_control_step(self):
        cmd=self.timed_command
        if cmd is None:
            return
        self.timed_heartbeat_wall=time.monotonic()
        now=self.get_clock().now().nanoseconds/1e9
        dt=now-self.timed_last_time
        if (self.pose is None or self.last_odom_time is None
                or not 0 <= now-self.last_odom_time <= self.odom_timeout
                or self.odom_frame != cmd.frame_id or dt < -0.02
                or (now >= cmd.start_time_s and dt > 0.5)):
            self._timed_abort('Odometría vencida, cambio de frame o salto de /clock')
            return
        if now < cmd.start_time_s:
            self.stop()
            self.timed_last_time=now
            return
        if not self.timed_running_reported:
            self.timed_running_reported=True
            self._timed_status(cmd,'running')
        point,velocity,phase=reference(cmd,now)
        ex=point[0]-self.pose[0]
        ey=point[1]-self.pose[1]
        error=math.hypot(ex,ey)
        if error > cmd.max_tracking_error_m:
            self._timed_abort(f'Error de seguimiento {error:.3f}m excede {cmd.max_tracking_error_m:.3f}m')
            return
        if phase=='after':
            if now > cmd.start_time_s+cmd.duration_s+cmd.finish_grace_s:
                self._timed_abort('Tiempo de llegada excedido')
                return
            if error <= cmd.goal_tolerance_m:
                if self.timed_goal_since is None:
                    self.timed_goal_since=now
                if now-self.timed_goal_since >= cmd.goal_settle_s:
                    self._timed_status(cmd,'done')
                    self.timed_command=None
                    self.stop()
                    return
            else:
                self.timed_goal_since=None
        # Feedforward temporal + realimentación P en el plano mundial.
        vx=velocity[0]+self.hybrid_kp*ex
        vy=velocity[1]+self.hybrid_kp*ey
        limit=min(self.hybrid_timed_max_speed,cmd.speed_limit_m_s)
        length=math.hypot(vx,vy)
        if length>limit:
            vx*=limit/length;vy*=limit/length
        # Limitar aceleración COMANDADA, distinta de la cota de referencia.
        dt=max(0.,min(dt,0.2))
        if dt>0:
            dvx=vx-self.timed_last_velocity[0]
            dvy=vy-self.timed_last_velocity[1]
            dv=math.hypot(dvx,dvy)
            allowed=cmd.acceleration_limit_m_s2*dt
            if dv>allowed:
                vx=self.timed_last_velocity[0]+dvx*allowed/dv
                vy=self.timed_last_velocity[1]+dvy*allowed/dv
        yaw=self.pose[2]
        c,s=math.cos(yaw),math.sin(yaw)
        twist=Twist()
        twist.linear.x=c*vx+s*vy
        twist.linear.y=-s*vx+c*vy
        self.cmd_pub.publish(twist)
        self.timed_last_velocity=(vx,vy)
        self.timed_last_time=now

    def on_turn_target(self, msg):
        if self.timed_command is not None:
            self._timed_abort('Se recibió preorientación durante trayectoria')
            self.turn_result_pub.publish(Int8(data=-1))
            return
        if self.visual_active or self.turn_target is not None or not math.isfinite(msg.data):
            self.get_logger().warn('Giro rechazado: control ocupado u objetivo inválido')
            self.turn_result_pub.publish(Int8(data=-1))
            return
        if self.pose is None:
            self.get_logger().warn('Giro rechazado: sin odometría')
            self.turn_result_pub.publish(Int8(data=-1))
            return
        self.path = []  # solo giro en posición, nunca reanudar ruta antigua
        self.path_index = 0
        self.turn_target = wrap_angle(float(msg.data))
        self.turn_started = time.monotonic()
        self.turn_hold_started = None
        self.stop()
        self.get_logger().info(f'PREORIENTAR: yaw objetivo={math.degrees(self.turn_target):.1f}°')

    def on_visual_active(self, msg):
        active = bool(msg.data)
        if active and not self.visual_active:
            if self.timed_command is not None:
                self._timed_abort('Control RGB-D solicitado durante trayectoria')
                return
            if self.turn_target is not None:
                self.turn_target = None
                self.turn_result_pub.publish(Int8(data=-1))
            self.path = []
            self.path_index = 0
            self.visual_cmd = Twist()
            self.visual_last_time = None
            self.stop()
            self.get_logger().info('CONTROL RGB-D: ruta suspendida')
        elif not active and self.visual_active:
            self.visual_cmd = Twist()
            self.visual_last_time = None
            self.stop()
            self.get_logger().info('CONTROL RGB-D terminado: esperando nueva ruta')
        self.visual_active = active

    def on_path(self, msg):
        if self.timed_command is not None:
            if not msg.poses:
                self._timed_abort('Ruta vacía de parada', state='cancelled')
            else:
                self.get_logger().warn('Waypoint rechazado: Bézier está activo')
            return
        if self.visual_active or self.turn_target is not None:
            self.get_logger().warn('Ruta recibida mientras control local está activo; ignorada')
            return
        super().on_path(msg)

    def on_visual_cmd(self, msg):
        if self.visual_active:
            self.visual_cmd = msg
            self.visual_last_time = self.get_clock().now().nanoseconds / 1e9

    def control_step(self):
        if self.timed_command is not None:
            self._timed_control_step()
            return
        if self.visual_active:
            now = self.get_clock().now().nanoseconds / 1e9
            if (self.visual_last_time is not None
                    and 0 <= now - self.visual_last_time <= self.visual_timeout):
                self.cmd_pub.publish(self.visual_cmd)
            else:
                self.stop()
            return
        if self.turn_target is not None:
            now = time.monotonic()
            current_sim_time = self.get_clock().now().nanoseconds / 1e9
            if (self.pose is None or self.last_odom_time is None
                    or current_sim_time - self.last_odom_time > self.odom_timeout
                    or now - self.turn_started > self.turn_timeout):
                self.stop()
                self.turn_target = None
                self.turn_result_pub.publish(Int8(data=-1))
                self.get_logger().error('PREORIENTAR FALLIDO: odometría vencida o timeout')
                return
            error = wrap_angle(self.turn_target - self.pose[2])
            if abs(error) <= self.turn_tolerance:
                self.stop()
                if self.turn_hold_started is None:
                    self.turn_hold_started = now
                elif now - self.turn_hold_started >= 0.4:
                    self.turn_target = None
                    self.turn_result_pub.publish(Int8(data=1))
                    self.get_logger().info('PREORIENTAR CONFIRMADO')
                return
            self.turn_hold_started = None
            cmd = Twist()
            cmd.angular.z = max(-self.turn_max_speed, min(self.turn_max_speed,
                                                           self.turn_kp * error))
            self.cmd_pub.publish(cmd)
            return
        super().control_step()


def main(args=None):
    rclpy.init(args=args)
    node = GraspPathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
