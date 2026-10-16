#!/usr/bin/env python3
"""Primera recogida efectiva: navegación CBS validada + eliminación confirmada en Gazebo.

No implementa aún viaje a depósitos. No altera scripts existentes.
Los tres robots deben llegar antes de retirar cualquier basura.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import subprocess

from cbs_primera_recogida_ros import FirstPickupExecutor
from grafo_basuras_dinamico import DynamicTrashGraph


def find_remove_service(run=subprocess.run, requested_world=None):
    """Encuentra el servicio real de eliminación del mundo activo."""
    if requested_world:
        return f'/world/{requested_world}/remove'
    p = run(['gz', 'service', '-l'], capture_output=True, text=True, timeout=12)
    if p.returncode != 0:
        raise RuntimeError('No se pudieron listar servicios Gazebo: ' + (p.stderr or p.stdout).strip())
    names = re.findall(r'^\s*(/world/[^\s/]+/remove)\s*$', p.stdout, re.MULTILINE)
    if len(names) != 1:
        raise RuntimeError(f'Se esperaba un solo servicio /world/.../remove; encontrados: {names}. Usa --world NOMBRE')
    return names[0]


def remove_gazebo_model(name, service, run=subprocess.run):
    """Solicita eliminar modelo por nombre. Requiere respuesta explícita true."""
    if not re.fullmatch(r'[A-Za-z0-9_]+', name):
        raise ValueError('Nombre de modelo inválido')
    cmd = ['gz', 'service', '-s', service,
           '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
           '--timeout', '15000', '--req', f'name: "{name}" type: MODEL']
    p = run(cmd, capture_output=True, text=True, timeout=12)
    output = (p.stdout or '') + '\n' + (p.stderr or '')
    # gz suele imprimir "data: true" o "data: false" para Boolean.
    if p.returncode != 0 or not re.search(r'\bdata\s*:\s*true\b', output, re.I):
        raise RuntimeError(f'Gazebo no confirmó la eliminación de {name}: {output.strip()}')
    return True


def apply_confirmed_pickups(graph, names, service, remover=remove_gazebo_model):
    """Elimina modelos uno a uno y actualiza grafo solo tras confirmación."""
    if len(names) != len(set(names)):
        raise ValueError('Hay basuras duplicadas')
    if any(name not in graph.pending for name in names):
        raise ValueError('La lista contiene basuras inexistentes o ya recogidas')
    removed, failures = [], {}
    for name in names:
        try:
            remover(name, service)
        except Exception as exc:
            failures[name] = str(exc)
        else:
            graph.collect(name)
            removed.append(name)
    return removed, failures


class PickupAndRemoveExecutor(FirstPickupExecutor):
    def __init__(self, world=None, state_file='/tmp/swarm_recogida_estado.json', **kwargs):
        self.remove_world = world
        self.state_file = Path(state_file)
        self.finished_removal = False
        super().__init__(**kwargs)
        self.node.get_logger().info('RECOGIDA: se retirarán modelos solamente tras completar la última barrera')

    def tick(self):
        if self.finished_removal:
            return
        super().tick()
        if self.phase != 'DONE':
            return
        self.finished_removal = True
        log = self.node.get_logger()
        self.stop_all()
        names = [name for name, _ in self.assignments]
        try:
            service = find_remove_service(requested_world=self.remove_world)
            log.info(f'Servicio de eliminación: {service}')
            removed, failures = apply_confirmed_pickups(self.dynamic, names, service)
        except Exception as exc:
            removed, failures = [], {'servicio': str(exc)}
        for name in removed:
            log.info(f'RECOGIDA CONFIRMADA: {name}; vértice liberado={self.dynamic.trash[name].node}')
        for name, reason in failures.items():
            log.error(f'NO SE RECOGIÓ {name}: {reason}')
        payload = {
            'removed_confirmed': removed,
            'failed': failures,
            'pending': sorted(self.dynamic.pending),
            'vertices_free': len(self.dynamic.vertices),
            'edges_free': len(self.dynamic.edges),
            'next_step': 'planificar entrega a depósitos; aún no implementada',
        }
        try:
            self.state_file.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
            log.info(f'Estado guardado en {self.state_file}')
        except OSError as exc:
            log.error(f'No se pudo guardar el estado: {exc}')
        log.info(f'RECOGIDA FINALIZADA: eliminadas={len(removed)}/3, pendientes={len(self.dynamic.pending)}; robots detenidos')


def main():
    import rclpy
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--world', default=None, help='Nombre del mundo Gazebo; autodetección si se omite')
    ap.add_argument('--state-file', default='/tmp/swarm_recogida_estado.json')
    ap.add_argument('--max-ct', type=int, default=20000)
    ap.add_argument('--max-time', type=int, default=65)
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    executor = PickupAndRemoveExecutor(world=args.world, state_file=args.state_file,
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
