#!/usr/bin/env python3
"""Recoge 3 basuras y conduce los 3 Summit a depósitos del color correcto.

La recogida solo se confirma si Gazebo elimina cada modelo; el viaje de
entrega comienza únicamente si las tres eliminaciones fueron confirmadas.
No modifica ninguno de los ejecutores anteriores.
"""
from __future__ import annotations
import argparse
import json
import math
import time

import grafo_cbs_125 as geom
import cbs_grafo_125 as cbs
from cbs_asignacion_basuras import solve_multi
from cbs_primera_recogida_ros import choose_safe_alignment
from cbs_recogida_efectiva_ros import PickupAndRemoveExecutor
from cbs_ros_executor import pad_paths, extract_step


def deposit_goals(graph, color):
    """Vértices vecinos transitables alrededor del disco del depósito.

    Se excluye el centro: permite múltiples robots sin entrar en el disco.
    """
    deposits = [d for d in graph.deposits.values() if d.color == color]
    if len(deposits) != 1:
        raise ValueError(f'Se esperaba un único depósito para color {color}')
    i, j = deposits[0].node
    candidates = ((i+di,j+dj) for di in (-1,0,1) for dj in (-1,0,1) if di or dj)
    return tuple(sorted(v for v in candidates if v in graph.vertices))


def plan_delivery(graph, starts, picked_names, max_ct=20000, max_time=65):
    """CBS multi-meta: robots del mismo color pueden terminar en vecinos diferentes."""
    if len(starts) != 3 or len(picked_names) != 3:
        raise ValueError('Se requieren exactamente 3 robots y 3 basuras recogidas')
    if len(set(picked_names)) != 3:
        raise ValueError('No se puede entregar una basura dos veces')
    if any(n not in graph.trash or n in graph.pending for n in picked_names):
        raise ValueError('Hay basuras desconocidas o no confirmadas como recogidas')
    starts = tuple(starts)
    if any(s not in graph.vertices for s in starts):
        raise ValueError('Algún inicio está bloqueado')
    goals = tuple(deposit_goals(graph, graph.trash[n].color) for n in picked_names)
    if any(not options for options in goals):
        raise ValueError('Hay un depósito sin posiciones accesibles')
    result = solve_multi(graph, starts, goals, max_ct=max_ct, max_time=max_time)
    if cbs.first_conflict(result['paths']) is not None:
        raise ValueError('CBS devolvió conflictos')
    for i, path in enumerate(result['paths']):
        if path[0] != starts[i] or path[-1] not in goals[i]:
            raise ValueError('Inicio o final incorrecto')
        if any(v not in graph.vertices for v in path):
            raise ValueError('Ruta atraviesa obstáculo pendiente')
        if any(a != b and b not in graph.neighbors[a] for a,b in zip(path,path[1:])):
            raise ValueError('Ruta utiliza arista prohibida')
    return goals, result


class PickupDeliveryExecutor(PickupAndRemoveExecutor):
    def __init__(self, **kwargs):
        self.stage = 'PICKUP'
        self.delivery_goals = None
        self.delivery_odom_wait_started = None
        self.delivery_odom_wait_timeout = 10.0
        self.delivery_align_targets = None
        self.delivery_plan_finished_at = None
        super().__init__(**kwargs)
        # Impedir reutilizar un estado de recogida de una ejecución anterior.
        try:
            self.state_file.unlink(missing_ok=True)
        except OSError as exc:
            self.node.get_logger().warn(f'No se pudo limpiar estado anterior: {exc}')
        self.node.get_logger().info('ENTREGA: tras recoger 3 basuras, CBS enviará los Summit a sus depósitos')

    def tick(self):
        if self.stage in ('COMPLETE', 'FAILED'):
            return
        if self.stage == 'PICKUP':
            super().tick()
            if self.phase == 'ABORT':
                self.stage = 'FAILED'
                return
            if self.finished_removal:
                try:
                    state = json.loads(self.state_file.read_text(encoding='utf-8'))
                    names = [name for name,_ in self.assignments]
                    if set(state['removed_confirmed']) != set(names) or state['failed']:
                        raise RuntimeError('Recogida incompleta: no iniciaremos entrega')
                    # Las llamadas síncronas a `gz service` impiden atender /odom
                    # durante la eliminación. Esperar nuevos mensajes en otro tick.
                    self.stage = 'WAIT_DELIVERY_ODOM'
                    self.delivery_odom_wait_started = time.monotonic()
                    self.node.get_logger().info(
                        'RECOGIDA COMPLETA: esperando odometría nueva antes de planificar entrega')
                except (RuntimeError, ValueError, KeyError, OSError) as exc:
                    self.node.get_logger().error(f'ENTREGA CANCELADA: {exc}')
                    self.stop_all()
                    self.stage = 'FAILED'
            return
        if self.stage == 'WAIT_DELIVERY_ODOM':
            # Dejar al spin procesar mensajes entrantes: no bloquear ni planificar
            # con odometría caducada inmediatamente después de usar gz service.
            if not self.fresh():
                if time.monotonic() - self.delivery_odom_wait_started > self.delivery_odom_wait_timeout:
                    self.node.get_logger().error(
                        'ENTREGA CANCELADA: no llegó odometría fresca de los tres Summit '
                        'en 10 segundos')
                    self.stop_all()
                    self.stage = 'FAILED'
                return
            try:
                names = [name for name, _ in self.assignments]
                starts = tuple(choose_safe_alignment(self.dynamic, self.odom[a][:2])
                               for a in self.agents)
                goals, result = plan_delivery(self.dynamic, starts, names,
                                              self.max_ct, self.max_time)
                self.delivery_goals = goals
                self.planned = pad_paths(result['paths'])
                self.step_idx = 0
                log = self.node.get_logger()
                for i, a in enumerate(self.agents):
                    color = self.dynamic.trash[names[i]].color
                    log.info(f'{a}: entrega color={color}; deposito vecinos={goals[i]}; '
                             f'meta={result["paths"][i][-1]}; ruta={result["paths"][i]}')
                log.info(f'CBS ENTREGA OK: costo={result["cost"]}, CT={result["expanded"]}, '
                         f'pasos={len(self.planned[0])-1}')
                # El CBS puede tardar más que odom_timeout dentro de tick().
                # Esperar odometría recibida DESPUÉS de finalizar el plan.
                self.delivery_align_targets = {
                    a: geom.node_position(starts[i]) for i, a in enumerate(self.agents)
                }
                self.delivery_plan_finished_at = time.monotonic()
                self.delivery_odom_wait_started = self.delivery_plan_finished_at
                self.stage = 'WAIT_DELIVERY_ALIGN_ODOM'
                log.info('CBS listo: esperando odometría posterior a la planificación')
            except (RuntimeError, ValueError, KeyError, OSError) as exc:
                self.node.get_logger().error(f'ENTREGA CANCELADA: {exc}')
                self.stop_all()
                self.stage = 'FAILED'
            return
        if self.stage == 'WAIT_DELIVERY_ALIGN_ODOM':
            # A diferencia de fresh(), exigimos odometría posterior al cálculo CBS.
            updated = all(
                a in self.odom and self.odom[a][3] > self.delivery_plan_finished_at
                for a in self.agents
            )
            if updated and self.fresh():
                self.stage = 'DELIVERY'
                self.start_phase('DELIVERY_ALIGN', self.delivery_align_targets)
            elif time.monotonic() - self.delivery_odom_wait_started > self.delivery_odom_wait_timeout:
                self.node.get_logger().error(
                    'ENTREGA CANCELADA: no llegó odometría posterior a CBS en 10 segundos')
                self.stop_all()
                self.stage = 'FAILED'
            return
        # Navegación sincronizada a los depósitos usando el mismo contrato
        # de tolerancia, estabilidad, odometría y timeout que la recogida.
        if not self.fresh():
            self.fail('Odometría vencida durante entrega')
            self.stage = 'FAILED'
            return
        now = time.monotonic()
        if now-self.started > self.timeout:
            self.fail(f'Tiempo agotado en {self.phase}')
            self.stage = 'FAILED'
            return
        for agent in self.agents:
            x,y,speed,_,_ = self.odom[agent]
            gx,gy = self.target[agent]
            reached = math.hypot(x-gx,y-gy) <= self.tol and speed <= self.speed_limit
            self.holding_since[agent] = (
                now if reached and self.holding_since[agent] is None
                else self.holding_since[agent] if reached else None
            )
        if not all(self.holding_since[a] is not None and
                   now-self.holding_since[a] >= self.hold for a in self.agents):
            return
        self.node.get_logger().info(f'BARRERA superada: {self.phase}')
        next_step = 1 if self.phase == 'DELIVERY_ALIGN' else self.step_idx+1
        if next_step >= len(self.planned[0]):
            self.stop_all()
            self.stage = 'COMPLETE'
            self.phase = 'DONE'
            self.node.get_logger().info('ENTREGA COMPLETADA: los tres Summit llegaron a sus depósitos; quedan 3 basuras')
            return
        self.step_idx = next_step
        nodes = extract_step(self.planned, next_step)
        waits = [a for i,a in enumerate(self.agents)
                 if self.planned[i][next_step] == self.planned[i][next_step-1]]
        self.node.get_logger().info(f'ENTREGA_STEP_{next_step}: {nodes} | WAIT={waits}')
        self.start_phase(f'ENTREGA_STEP_{next_step}', {
            a: geom.node_position(nodes[a]) for a in self.agents
        })


def main():
    import rclpy
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--world', default=None)
    ap.add_argument('--state-file', default='/tmp/swarm_recogida_estado.json')
    ap.add_argument('--max-ct', type=int, default=20000)
    ap.add_argument('--max-time', type=int, default=65)
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    executor = PickupDeliveryExecutor(world=args.world, state_file=args.state_file,
                                      max_ct=args.max_ct, max_time=args.max_time)
    try:
        rclpy.spin(executor.node)
    except KeyboardInterrupt:
        pass
    finally:
        executor.stop_all()
        executor.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
