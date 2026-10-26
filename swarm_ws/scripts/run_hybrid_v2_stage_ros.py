#!/usr/bin/env python3
"""Hybrid V2 D.4 guarded executor: ONE pre-certified stage, no alignment or replanning.

Deliberately refuses mismatched starts. No extra cmd_vel publisher.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import math
import time
from uuid import uuid4
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from swarm_behavior.swarm_planning.hybrid_v2.ros_protocol_adapter import build_commands
from swarm_behavior.swarm_planning.hybrid_refinement.timed_protocol import (
    TimedCommand, decode_command, decode_status, TimedProtocolError, reference)

ROBOTS = ('summit0','summit1','summit2')

class GuardedExecutor(Node):
    def __init__(self, args):
        super().__init__('hybrid_v2_guarded_executor', parameter_overrides=[Parameter('use_sim_time',Parameter.Type.BOOL, True)])
        self.args=args; self.mode='WAIT'; self.session='v2-'+uuid4().hex[:18]
        self.odoms={}; self.states={}; self.commands={}; self.parsed={}; self.arm_wall=0.; self.start_s=None
        self.started=time.monotonic(); self.last_log=0.; self.graph_wait_started=None
        self.dds_ready_since=None
        self.subs=[]; self.pubs={}
        for a in ROBOTS:
            self.subs.append(self.create_subscription(Odometry, f'/{a}/odom', lambda m,a=a:self.on_odom(a,m),10))
            self.subs.append(self.create_subscription(String, f'/{a}/hybrid_trajectory/status', lambda m,a=a:self.on_status(a,m),10))
            self.pubs[a]=self.create_publisher(String,f'/{a}/hybrid_trajectory/command',10)
        self.timer=self.create_timer(.1,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('V2 D.4: preflight de odometría; NO alineación automática; etapa única')

    def on_odom(self,a,msg):
        p=msg.pose.pose.position; v=msg.twist.twist.linear
        self.odoms[a]=(float(p.x),float(p.y),msg.header.frame_id,math.hypot(v.x,v.y),time.monotonic())

    def on_status(self,a,msg):
        try: robot,sid,state=decode_status(msg.data)
        except TimedProtocolError:return
        if robot==a and sid==self.session:
            self.states[a]=state
            self.get_logger().info(f'{a}: {state}')
            if state in ('failed','cancelled') and self.mode not in ('FAILED','DONE'):
                self.fail(f'{a} informó {state}')

    def cancel(self):
        if self.mode not in ('ARMED','RUNNING','FAILED','DONE'):return
        for a in ROBOTS:
            self.pubs[a].publish(String(data=TimedCommand('cancel',self.session,a).to_json()))

    def fail(self,why):
        if self.mode in ('FAILED','DONE'): return
        prev=self.mode; self.mode='FAILED'; self.get_logger().error('ABORT: '+str(why))
        if prev in ('ARMED','RUNNING'):self.cancel()

    def fresh(self):
        return len(self.odoms)==3 and all(time.monotonic()-p[4]<1.5 and p[2] for p in self.odoms.values())

    def followers_ready(self):
        """Nonblocking ROS graph discovery; never mistake an observer for a controller."""
        if self.graph_wait_started is None:
            self.graph_wait_started=time.monotonic()
        missing=[]
        for robot in ROBOTS:
            topic=f'/{robot}/hybrid_trajectory/command'
            endpoints=self.get_subscriptions_info_by_topic(topic)
            followers=[e for e in endpoints if e.node_name=='path_follower'
                       and e.node_namespace.rstrip('/')==f'/{robot}'
                       and e.topic_type=='std_msgs/msg/String']
            if len(followers)>1:
                raise ValueError(f'{robot}: múltiples seguidores autorizados en {topic}')
            if not followers:
                missing.append(robot)
        if missing and time.monotonic()-self.graph_wait_started>=8.0:
            raise ValueError(f'No se descubrieron path_follower después de 8 s: {missing}')
        return not missing

    def connections_ready(self):
        """Wait for DDS matching, not only discovery of graph endpoints.

        A subscriber can appear in get_subscriptions_info_by_topic before
        our just-created publisher has actually matched it. This function
        requires the authorized follower endpoint and a publisher match for
        each robot; an observer alone is never sufficient.
        """
        if not self.followers_ready():
            self.dds_ready_since=None
            return False
        missing=[]
        for robot in ROBOTS:
            count=self.pubs[robot].get_subscription_count()
            if count < 1:
                missing.append(robot)
        if missing:
            self.dds_ready_since=None
            if time.monotonic()-self.started>self.args.wait_timeout:
                raise ValueError(f'DDS no conectado con seguidores: {missing}')
            return False
        if self.dds_ready_since is None:
            self.dds_ready_since=time.monotonic()
            return False
        return time.monotonic()-self.dds_ready_since >= 1.0

    def check_start(self):
        if not self.fresh():raise ValueError('Falta odometría fresca')
        frames=[self.odoms[a][2] for a in ROBOTS]
        now=self.get_clock().now().nanoseconds/1e9
        if now<=0:raise ValueError('Reloj simulado no disponible')
        cmds=build_commands(self.args.report,self.args.stage, session_id=self.session,
            start_time_s=now+self.args.lead_time,frame_ids=frames,
            max_speed=self.args.max_speed,max_accel=self.args.max_accel,
            tracking_error=self.args.tracking_error,goal_tolerance=self.args.goal_tolerance,
            controls_path=self.args.controls)
        parsed={a:decode_command(cmds[a]) for a in ROBOTS}
        for a in ROBOTS:
            x,y,_,speed,_=self.odoms[a]
            initial=parsed[a].control_points[0][0]
            dist=math.dist((x,y),initial)
            if dist>self.args.start_tolerance:
                raise ValueError(f'{a} inicio diferente: odom={(round(x,3),round(y,3))}, Bézier={tuple(round(z,3) for z in initial)}, error={dist:.3f} m > {self.args.start_tolerance:.3f} m. NO ejecutar; regenerar con inicios reales o alinear de forma validada')
            if speed>.08:raise ValueError(f'{a} en movimiento: {speed:.3f} m/s')
        # A live plan's captured positions must correspond to control starting points.
        report=json.loads(Path(self.args.report).read_text(encoding='utf-8'))
        stage=report['stages'][self.args.stage]['hybrid_v2']
        if stage.get('odom_frame_policy') == 'verified_namespaced_world_coordinates':
            for a in ROBOTS:
                if self.odoms[a][2] != f'{a}/odom':
                    raise ValueError(f'{a} ya no utiliza su frame verificado')
        if True:  # D.4.2: only live D.4.1 report is allowed
            if stage.get('stage') != 'D.4.1' or stage.get('mission_stage') != self.args.stage:
                raise ValueError('D.4.2 exige informe generado desde odometría D.4.1')
            captured=stage.get('odom_xy')
            if not isinstance(captured,list) or len(captured)!=3:
                raise ValueError('Falta snapshot de odometría original')
            for i,a in enumerate(ROBOTS):
                if math.dist(captured[i],parsed[a].control_points[0][0]) > 1e-5:
                    raise ValueError(f'{a}: controles no coinciden con snapshot certificado')
            if abs(float(stage.get('experimental_robot_radius_m',-1))-self.args.robot_radius) > 1e-6:
                raise ValueError('Radio experimental distinto al del informe')
            if abs(float(stage.get('operational_margin_m',-1))-self.args.operational_margin) > 1e-6:
                raise ValueError('Reserva distinta al informe')
            scenario=Path(stage.get('scenario_json',''))
            if not scenario.is_file():
                raise ValueError('Escenario del reporte no encontrado: '+str(scenario))
            if self.args.scenario and scenario.resolve()!=Path(self.args.scenario).resolve():
                raise ValueError('Ruta de escenario no coincide con informe')
            # A certificate authorizes nominal trajectories, not the Gazebo world model.
        # Detect external motion command publisher (other than controller). Fails closed.
        for a in ROBOTS:
            pubs=self.get_publishers_info_by_topic(f'/{a}/cmd_vel')
            if len(pubs)>1:
                raise ValueError(f'{a} tiene {len(pubs)} publishers /cmd_vel: arriesga conflicto')
        # Physical distances monitored independently; this is NOT a proof of collision-free motion.
        self.commands=cmds; self.parsed=parsed; self.start_s=now+self.args.lead_time
        self.end_s=self.start_s+parsed[ROBOTS[0]].duration_s
        self.get_logger().info(f'PRECHECK OK: etapa={self.args.stage}, duración={parsed[ROBOTS[0]].duration_s:.2f}s, inicio /clock={self.start_s:.3f}')

    def monitor(self):
        if not self.fresh():raise ValueError('Odometría caducada')
        for i,a in enumerate(ROBOTS):
            x,y,_,_,_=self.odoms[a]
            for b in ROBOTS[i+1:]:
                xb,yb,_,_,_=self.odoms[b]
                d=math.dist((x,y),(xb,yb))
                if d<self.args.min_separation:
                    raise ValueError(f'Distancia {a}-{b} {d:.3f}m < {self.args.min_separation:.3f}m')
        # Independently verify tracking of the temporal reference using fresh odometry.
        now=self.get_clock().now().nanoseconds/1e9
        for a in ROBOTS:
            cmd=self.parsed.get(a)
            if cmd is None: continue
            p,_,_=reference(cmd,now)
            observed=self.odoms[a][:2]
            error=math.dist(p,observed)
            if now>=cmd.start_time_s and error > min(self.args.tracking_error,self.args.abort_tracking_error):
                raise ValueError(f'{a}: error de seguimiento {error:.3f} m')
        # Full Gazebo obstacle/footprint verification remains external to this monitor.

    def tick(self):
        if self.mode in ('DONE','FAILED'):return
        try:
            if self.mode=='WAIT':
                if time.monotonic()-self.started>self.args.wait_timeout:
                    raise ValueError('Timeout esperando odometría y /clock')
                if not self.fresh() or self.get_clock().now().nanoseconds<=0:return
                if not self.connections_ready():return
                self.check_start()
                if not self.args.execute:
                    self.get_logger().info('DRY RUN OK. Ningún movimiento ordenado. Usar --execute solamente tras verificar Gazebo y parámetros.')
                    self.mode='DONE'; return
                if not self.args.world_verified:
                    raise ValueError('Bloqueado: falta --world-verified (verificación externa de escenario y geometría)')
                # Final re-check immediately before issuing the start orders.
                self.check_start()
                for a in ROBOTS:self.pubs[a].publish(String(data=self.commands[a]))
                self.mode='ARMED';self.arm_wall=time.monotonic()
                self.get_logger().info('Órdenes enviadas: esperando tres ACK armed')
            elif self.mode=='ARMED':
                self.monitor()
                if all(self.states.get(a) in ('armed','running','done') for a in ROBOTS):
                    self.mode='RUNNING'; self.get_logger().info('Tres seguidores reconocieron la sesión')
                elif time.monotonic()-self.arm_wall>min(5,self.args.lead_time-1):
                    raise ValueError(f'ACK incompleto: {self.states}')
            elif self.mode=='RUNNING':
                self.monitor()
                if all(self.states.get(a)=='done' for a in ROBOTS):
                    self.mode='DONE'; self.get_logger().info('Etapa completada (según estados de controladores)')
                elif self.get_clock().now().nanoseconds/1e9>self.end_s+10:
                    raise ValueError('Timeout trayectoria')
        except Exception as exc:self.fail(str(exc))

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--report',required=True);ap.add_argument('--controls',required=True)
    ap.add_argument('--stage',default='pickup1',choices=('pickup1','delivery1','pickup2','delivery2'))
    ap.add_argument('--execute',action='store_true',help='AUTORIZA movimiento: default es solo preflight')
    ap.add_argument('--world-verified',action='store_true',help='Confirmación explícita de que mundo y escenario, incluyendo huellas, fueron revisados')
    ap.add_argument('--scenario',help='Mismo escenario JSON registrado en informe D.4.1')
    ap.add_argument('--robot-radius',type=float,default=.55)
    ap.add_argument('--operational-margin',type=float,default=.05)
    ap.add_argument('--abort-tracking-error',type=float,default=.16)
    ap.add_argument('--start-tolerance',type=float,default=.10)
    ap.add_argument('--min-separation',type=float,default=1.15)
    ap.add_argument('--lead-time',type=float,default=8.)
    ap.add_argument('--tracking-error',type=float,default=.16)
    ap.add_argument('--goal-tolerance',type=float,default=.10)
    ap.add_argument('--max-speed',type=float,default=1.2)
    ap.add_argument('--max-accel',type=float,default=1.)
    ap.add_argument('--wait-timeout',type=float,default=45.)
    args,rosargs=ap.parse_known_args()
    if not (.05<=args.start_tolerance<=.20 and args.min_separation>=1.15 and args.lead_time>=7
            and .10<=args.tracking_error<=.20 and .05<=args.goal_tolerance<=args.tracking_error
            and 0<args.max_speed<=1.2 and 0<args.max_accel<=1.
            and .03<=args.abort_tracking_error<=args.tracking_error):ap.error('Parámetros fuera de límites D4')
    rclpy.init(args=rosargs); n=GuardedExecutor(args)
    try:
        while rclpy.ok() and n.mode not in ('DONE','FAILED'):
            rclpy.spin_once(n,timeout_sec=.2)
        return 0 if n.mode=='DONE' else 2
    except KeyboardInterrupt:
        n.fail('Interrumpido por operador');return 130
    finally:
        if n.mode in ('ARMED','RUNNING','FAILED'):
            n.cancel()
            for _ in range(3):rclpy.spin_once(n,timeout_sec=.1)
        n.destroy_node();rclpy.shutdown()
if __name__=='__main__':raise SystemExit(main())
