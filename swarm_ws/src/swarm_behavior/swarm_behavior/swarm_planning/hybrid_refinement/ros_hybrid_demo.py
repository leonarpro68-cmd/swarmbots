"""Fase 5: demo ROS 2 de tres Summit con CBS + corredores + Bézier.

No arranca Gazebo ni ejecuta agarres. Úsese SOLO con el launch aislado
swarm_cbs_grasp_base.launch.py hybrid_enabled:=true, sin run_swarmbots.sh.

Etapas: odometría real -> alineación SECUENCIAL segura -> CBS de recogida
(con mapa de escenario) -> refinamiento offline -> envío futuro común por
/clock -> ACK 'armed' de tres controladores -> seguimiento y monitoreo.

Si la optimización o un controlador falla, envía cancel a TODOS.
No se garantiza evasión bajo fallas de hardware, reloj o seguimiento.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path as FilePath
import json
import math
import time
from uuid import uuid4

import rclpy
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from .. import geometry
from ..bridge import PlanningBridge
from ..ros_plan_helpers import AGENTS, RobotSample, planning_starts, snapshot
from .adapter import from_cbs_paths
from .corridors import build_hybrid_corridors
from .bezier_optimizer import optimize_bezier, scale_trajectory_time
from .iterative_refinement import refine_iteratively
from .timed_protocol import TimedCommand, decode_status, TimedProtocolError
from .run_report import save_run_report


class HybridDemo(Node):
    def __init__(self, args):
        super().__init__('swarm_hybrid_demo', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.args = args
        self.bridge = PlanningBridge.from_json(args.scenario)
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.samples: dict[str, RobotSample] = {}
        self.speeds: dict[str, float] = {}
        self.odom_subs = [self.create_subscription(
            Odometry, f'/{a}/odom', lambda msg, a=a: self._on_odom(a,msg), 10)
            for a in AGENTS]
        self.status_subs = [self.create_subscription(
            String, f'/{a}/hybrid_trajectory/status',
            lambda msg, a=a: self._on_status(a,msg), 10) for a in AGENTS]
        self.path_pubs={a:self.create_publisher(Path,f'/{a}/planned_path',10) for a in AGENTS}
        self.timed_pubs={a:self.create_publisher(
            String,f'/{a}/hybrid_trajectory/command',10) for a in AGENTS}
        self.state='WAIT_ODOM'
        self.started_wall=time.monotonic()
        self.align_starts=None
        self.align_agent=0
        self.align_since=0.
        self.align_hold_since=None
        self.statuses={}
        self.session=None
        self.start_time=None
        self.arm_deadline=0.
        self.expected_duration=None
        self.plan_since=None
        self.last_monitor_log=0.
        # Reloj monotónico para vigilancia; referencias siguen /clock simulado.
        self.timer=self.create_timer(.1,self.tick,
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Demo híbrida: esperando /clock, odometría y DDS (NO iniciar misión automática)')

    def _sim_now(self):
        return self.get_clock().now().nanoseconds/1e9

    def _on_odom(self,a,msg):
        p=msg.pose.pose.position
        v=msg.twist.twist.linear
        self.samples[a]=RobotSample(float(p.x),float(p.y),time.monotonic(),msg.header.frame_id)
        self.speeds[a]=math.hypot(v.x,v.y)

    def _on_status(self, agent, msg):
        try:
            robot,session,state=decode_status(msg.data)
        except TimedProtocolError:
            self.get_logger().warn('Estado híbrido inválido ignorado')
            return
        if robot!=agent or session!=self.session:
            return
        self.statuses[agent]=state
        if state in ('failed','cancelled') and self.state not in ('FAILED','DONE'):
            self.fail(f'{agent} reportó {state}: {msg.data[:250]}')
        else:
            self.get_logger().info(f'{agent}: {state}')

    def _fresh(self):
        return snapshot(self.samples,1.5)

    def _path(self,a, xy=None):
        m=Path()
        m.header.frame_id=self.samples[a].frame_id if a in self.samples else ''
        m.header.stamp=self.get_clock().now().to_msg()
        if xy is not None:
            p=PoseStamped()
            p.header=m.header
            p.pose.position.x=float(xy[0]);p.pose.position.y=float(xy[1]);p.pose.orientation.w=1.
            m.poses.append(p)
        self.path_pubs[a].publish(m)

    def _cancel(self):
        for a in AGENTS:
            if self.session is not None:
                self.timed_pubs[a].publish(String(data=TimedCommand(
                    'cancel',self.session,a).to_json()))
            self._path(a)

    def fail(self, reason):
        if self.state in ('FAILED','DONE'):
            return
        self.get_logger().error('DEMO HÍBRIDA ABORTADA: '+str(reason))
        self.state='FAILED'
        self._cancel()

    def _start_align(self):
        a=AGENTS[self.align_agent]
        p=(self.samples[a].x,self.samples[a].y)
        goal=geometry.node_position(self.align_starts[self.align_agent])
        if not self.bridge.graph.segment_clear(p,goal):
            raise ValueError(f'{a}: alineación atraviesa obstáculo')
        # Alineación secuencial: otros robots están inmóviles. Margen
        # de seguridad válido incluso si el seguidor ajusta su velocidad.
        for other in AGENTS:
            if other==a:
                continue
            q=(self.samples[other].x,self.samples[other].y)
            distance=geometry.segment_point_distance(p,goal,q)
            if distance < 2*geometry.CLEARANCE_WALL_M + .08:
                raise ValueError(f'Alineación {a} demasiado cerca de {other}: {distance:.3f} m')
        for other in AGENTS:
            self._path(other,goal if other==a else None)
        self.align_since=time.monotonic()
        self.align_hold_since=None
        self.get_logger().info(f'ALINEAR {a}: {p} -> {goal}')

    def _compute(self,starts):
        # Trabajo CPU fuera del callback ROS: permite procesar odometría/clock.
        plan=self.bridge.plan_pickup(starts, max_time=self.args.cbs_horizon)
        schedule=from_cbs_paths(plan.paths,graph=self.bridge.graph,
                                robot_names=AGENTS,max_speed_m_s=self.args.max_speed)
        corridors=build_hybrid_corridors(schedule,graph=self.bridge.graph)
        initial=optimize_bezier(schedule,corridors,maxiter=self.args.solver_maxiter)
        result=refine_iteratively(schedule,corridors,initial_trajectory=initial,
                                   iterations=self.args.iterations,
                                   maxiter=self.args.solver_maxiter,
                                   samples_per_piece=32,
                                   max_speed_m_s=self.args.max_speed,
                                   max_acceleration_m_s2=self.args.max_accel)
        scaled=scale_trajectory_time(result.final_trajectory,
                 max_speed_m_s=self.args.max_speed,
                 max_acceleration_m_s2=self.args.max_accel,
                 safety_factor=self.args.time_safety_factor)
        if not all(r.solver_success for r in result.final_trajectory.robots):
            raise ValueError('Optimizador no convergió: no ejecutar una solución de respaldo')
        if any(not math.isfinite(x) for r in scaled.trajectory.robots
               for piece in r.control_points for point in piece for x in point):
            raise ValueError('Curva no finita')
        return scaled,plan,result

    def _arm(self,scaled,plan,result):
        if self._fresh() is None:
            raise ValueError('No hay odometría fresca al publicar Bézier')
        for i,a in enumerate(AGENTS):
            point=scaled.trajectory.robots[i].control_points[0][0]
            if math.dist((self.samples[a].x,self.samples[a].y),point)>self.args.goal_tolerance:
                raise ValueError(f'{a} dejó la posición inicial durante optimización')
            if not self.samples[a].frame_id:
                raise ValueError(f'{a} no informa marco de odometría')
            if self.timed_pubs[a].get_subscription_count()<1:
                raise ValueError(f'{a} sin suscriptor hybrid_trajectory/command; habilita hybrid_enabled:=true')
        self.session='hybrid-'+uuid4().hex[:16]
        self.start_time=self._sim_now()+8.
        self.statuses={}
        # Exportar la misma planificación que se enviará a los robots.
        # No se modifica el plan ni se reoptimiza para el gráfico.
        if self.args.output_dir:
            directory=FilePath(self.args.output_dir).expanduser()
            paths=save_run_report(directory,self.session,self.bridge,plan,result,scaled,
                                  max_speed=self.args.max_speed,max_accel=self.args.max_accel)
            self.get_logger().info(f'PLAN HÍBRIDO EXPORTADO: {paths}')
        for i,a in enumerate(AGENTS):
            cmd=TimedCommand('start',self.session,a,frame_id=self.samples[a].frame_id,
                  start_time_s=self.start_time,dt_s=scaled.dt_s,
                  control_points=scaled.trajectory.robots[i].control_points,
                  speed_limit_m_s=self.args.max_speed,
                  acceleration_limit_m_s2=self.args.max_accel,
                  max_tracking_error_m=self.args.tracking_error,
                  goal_tolerance_m=self.args.goal_tolerance,
                  goal_settle_s=.6,finish_grace_s=4.)
            # El parser puro revalida el mismo JSON que recibirá el seguidor.
            from .timed_protocol import decode_command
            decode_command(cmd.to_json())
            self.timed_pubs[a].publish(String(data=cmd.to_json()))
        self.expected_duration=scaled.duration_s
        self.arm_deadline=time.monotonic()+5.
        self.state='ARMING'
        self.get_logger().info(f'CBS costo={plan.cost}; piezas={len(scaled.trajectory.robots[0].control_points)} '
            f'por robot; refinamientos={result.iterations_completed}; '
            f'duración={scaled.duration_s:.2f}s; sesión={self.session}')

    def _monitor(self):
        if self._fresh() is None:
            self.fail('Odometría de al menos un robot vencida')
            return False
        # El modelo nominal asegura 1.523m; se vigila la separación física
        # observada con una reserva, incluso durante un eventual desfase.
        guard=2*geometry.ROBOT_RADIUS_M + .18
        for i,a in enumerate(AGENTS):
            p=(self.samples[a].x,self.samples[a].y)
            if not self.bridge.graph.free_position(p):
                self.fail(f'{a} entró en zona inflada de obstáculo')
                return False
            for b in AGENTS[i+1:]:
                q=(self.samples[b].x,self.samples[b].y)
                d=math.dist(p,q)
                if d<guard:
                    self.fail(f'Distancia entre {a} y {b}: {d:.3f}m (<{guard:.3f}m)')
                    return False
        return True

    def tick(self):
        if self.state in ('DONE','FAILED'):
            return
        try:
            if self.state=='WAIT_ODOM':
                if self._sim_now() <= 0 or self._fresh() is None:
                    if time.monotonic()-self.started_wall>45:
                        self.fail('Timeout esperando /clock y odometrías')
                    return
                if any(self.path_pubs[a].get_subscription_count()<1 or
                       self.timed_pubs[a].get_subscription_count()<1 for a in AGENTS):
                    if time.monotonic()-self.started_wall>45:
                        self.fail('Timeout descubrimiento DDS')
                    return
                samples=self._fresh()
                self.align_starts=planning_starts(self.bridge.graph,samples)
                self.state='ALIGN'
                self._start_align()
            elif self.state=='ALIGN':
                if self._fresh() is None:
                    self.fail('Odometría vencida durante ALIGN')
                    return
                a=AGENTS[self.align_agent]
                goal=geometry.node_position(self.align_starts[self.align_agent])
                d=math.dist((self.samples[a].x,self.samples[a].y),goal)
                if time.monotonic()-self.align_since>40:
                    self.fail(f'Timeout alinear {a}: error={d:.3f}m')
                    return
                if d <= self.args.goal_tolerance and self.speeds.get(a,math.inf)<.10:
                    if self.align_hold_since is None:
                        self.align_hold_since=time.monotonic()
                    elif time.monotonic()-self.align_hold_since>.6:
                        self._path(a)
                        self.align_agent+=1
                        if self.align_agent==len(AGENTS):
                            self.state='PLANNING'
                            self.plan_since=time.monotonic()
                            self.future=self.pool.submit(self._compute,self.align_starts)
                            self.get_logger().info('Todos alineados: CBS + refinamiento trabajando')
                        else:
                            self._start_align()
                else:
                    self.align_hold_since=None
            elif self.state=='PLANNING':
                if self.plan_since is not None and time.monotonic()-self.plan_since>180:
                    self.fail('Timeout CBS + optimización (>180 s)')
                    return
                if not self._monitor():return
                if not self.future.done():return
                scaled,plan,result=self.future.result()
                self._arm(scaled,plan,result)
            elif self.state=='ARMING':
                if not self._monitor():return
                if all(self.statuses.get(a)=='armed' for a in AGENTS):
                    self.state='RUNNING'
                    self.get_logger().info('Tres ACK armados. Esperando inicio simultáneo por /clock')
                elif time.monotonic()>self.arm_deadline or self._sim_now()>self.start_time-1:
                    self.fail(f'No llegaron los tres ACK antes del inicio: {self.statuses}')
            elif self.state=='RUNNING':
                if not self._monitor():return
                if all(self.statuses.get(a)=='done' for a in AGENTS):
                    self._cancel()
                    self.state='DONE'
                    self.get_logger().info('DEMO HÍBRIDA FINALIZADA: 3 Summit completaron trayectorias')
                elif self._sim_now()>self.start_time+self.expected_duration+10:
                    self.fail('Timeout global de ejecución')
        except Exception as exc:
            self.fail(f'{type(exc).__name__}: {exc}')

    def close(self):
        self._cancel()
        # Dar a DDS una oportunidad de enviar las órdenes de cancelación.
        time.sleep(.25)
        self.pool.shutdown(wait=False,cancel_futures=True)


def main(args=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario',required=True,help='JSON del MISMO mundo Gazebo')
    parser.add_argument('--iterations',type=int,default=2)
    parser.add_argument('--solver-maxiter',type=int,default=150)
    parser.add_argument('--cbs-horizon',type=int,default=65)
    parser.add_argument('--max-speed',type=float,default=.60)
    parser.add_argument('--max-accel',type=float,default=.50)
    parser.add_argument('--tracking-error',type=float,default=.16)
    parser.add_argument('--time-safety-factor',type=float,default=1.08,help='>=1.0; margen temporal analítico')
    parser.add_argument('--output-dir',default='/tmp/swarm_hybrid_results',help='Directorio para JSON y PNG de esta ejecución')
    parser.add_argument('--goal-tolerance',type=float,default=.10)
    cli,rosargs=parser.parse_known_args(args)
    if not (0<=cli.iterations<=5 and cli.solver_maxiter>=1 and 1<=cli.cbs_horizon<=120
            and 0<cli.max_speed<=1.20 and 0<cli.max_accel<=1.00
            and 1.0<=cli.time_safety_factor<=2.0
            and .10<=cli.tracking_error<=.20 and .05<=cli.goal_tolerance<=cli.tracking_error):
        parser.error('Parámetros de la demo fuera de límites')
    rclpy.init(args=rosargs)
    node=None
    try:
        node=HybridDemo(cli)
        while rclpy.ok() and node.state not in ('DONE','FAILED'):
            rclpy.spin_once(node,timeout_sec=.2)
        return 0 if node.state=='DONE' else 1
    except KeyboardInterrupt:
        return 130
    finally:
        if node is not None:
            node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':
    raise SystemExit(main())
