#!/usr/bin/env python3
"""Recogida CBS experimental: discovery DDS y reenvio acotado de ALIGN.

No modifica A* ni CBS, no agarra ni elimina modelos. Solo ejecuta rutas.
"""
from __future__ import annotations
import argparse
import math
import time
import rclpy
from swarm_behavior.swarm_planning.ros_pickup_executor import PickupExecutor
from swarm_behavior.swarm_planning.ros_plan_helpers import AGENTS

class PickupDDS(PickupExecutor):
    def __init__(self, scenario, *, discovery_timeout=20., diagnostic_interval=2.,
                 retry_interval=5., max_retries=3, **kwargs):
        self.discovery_timeout=discovery_timeout
        self.diagnostic_interval=diagnostic_interval
        self.retry_interval=retry_interval
        self.max_retries=max_retries
        self.discovery_started=time.monotonic()
        self.last_discovery_log=0.
        self.last_diagnostic=0.
        self.last_retry=0.
        self.retry_count=0
        super().__init__(scenario, **kwargs)
        self.node.get_logger().info('RECOGIDA V2: esperando 3 suscriptores DDS antes de planificar')

    def _connections_ready(self):
        counts={a:self.pubs[a].get_subscription_count() for a in AGENTS}
        if all(x >= 1 for x in counts.values()):
            return True
        now=time.monotonic()
        if now-self.last_discovery_log >= 2.:
            self.last_discovery_log=now
            self.node.get_logger().warn(f'RECOGIDA V2 DDS: {counts}')
        if now-self.discovery_started >= self.discovery_timeout:
            self.fail(f'RECOGIDA V2: timeout de DDS: {counts}')
        return False

    def start_barrier(self, name, targets):
        counts={a:self.pubs[a].get_subscription_count() for a in AGENTS}
        if not all(x >= 1 for x in counts.values()):
            self.fail(f'RECOGIDA V2: falta DDS antes de {name}: {counts}')
            return
        super().start_barrier(name, targets)
        self.last_diagnostic=0.
        self.last_retry=time.monotonic()
        self.retry_count=0
        self.node.get_logger().info(f'RECOGIDA V2 PUBLICADO {name}: DDS={counts}')

    def tick(self):
        if self.state == 'WAIT_ODOM' and not self._connections_ready():
            return
        if self.state == 'MOVING' and self.barrier is not None:
            now=time.monotonic()
            name=self.barrier.name
            if now-self.last_diagnostic >= self.diagnostic_interval:
                self.last_diagnostic=now
                parts=[]
                for a in AGENTS:
                    sample=self.samples.get(a)
                    if sample is None:
                        parts.append(f'{a}: sin odometria')
                        continue
                    d=math.dist((sample.x,sample.y),self.barrier.targets[a])
                    v=self.velocities.get(a,float('nan'))
                    n=self.pubs[a].get_subscription_count()
                    parts.append(f'{a}: d={d:.3f}m v={v:.3f}m/s DDS={n}')
                self.node.get_logger().info(f'RECOGIDA V2 DIAGNOSTICO {name} | '+ ' | '.join(parts))
            # Limitar reenvios a ALIGN: no alterar pasos CBS temporales.
            if name == 'ALIGN' and now-self.last_retry >= self.retry_interval and self.retry_count < self.max_retries:
                self.last_retry=now
                self.retry_count+=1
                for a in AGENTS:
                    sample=self.samples.get(a)
                    if sample is None:
                        continue
                    count=self.pubs[a].get_subscription_count()
                    if count < 1:
                        self.fail(f'RECOGIDA V2: enlace DDS perdido en ALIGN para {a}')
                        return
                    d=math.dist((sample.x,sample.y),self.barrier.targets[a])
                    if d>self.arrival_tolerance:
                        self.publish(a,self.barrier.targets[a])
                        self.node.get_logger().warn(f'RECOGIDA V2 REENVIO {self.retry_count}/{self.max_retries}: {a} d={d:.3f}m')
        super().tick()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scenario',required=True)
    p.add_argument('--step-timeout',type=float,default=90.)
    p.add_argument('--discovery-timeout',type=float,default=20.)
    p.add_argument('--diagnostic-interval',type=float,default=2.)
    p.add_argument('--retry-interval',type=float,default=5.)
    p.add_argument('--max-retries',type=int,default=3)
    p.add_argument('--max-ct',type=int,default=20000)
    p.add_argument('--max-time',type=int,default=65)
    args,rosargs=p.parse_known_args()
    if args.step_timeout<20 or args.discovery_timeout<3 or args.diagnostic_interval<.5 or args.retry_interval<3 or not 0<=args.max_retries<=5:
        p.error('Parametros fuera de los limites de prueba')
    rclpy.init(args=rosargs)
    exe=None
    try:
        exe=PickupDDS(args.scenario,step_timeout=args.step_timeout,
                      discovery_timeout=args.discovery_timeout,
                      diagnostic_interval=args.diagnostic_interval,
                      retry_interval=args.retry_interval,max_retries=args.max_retries,
                      max_ct=args.max_ct,max_time=args.max_time)
        while rclpy.ok() and exe.state not in ('DONE','FAILED'):
            rclpy.spin_once(exe.node,timeout_sec=.2)
        return 0 if exe.state=='DONE' else 1
    except KeyboardInterrupt:
        return 130
    finally:
        if exe is not None:
            exe.close()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=='__main__':
    raise SystemExit(main())
