#!/usr/bin/env python3
"""Misión completa: 3 Summit recogen y entregan 6 basuras en 2 rondas.

Reutiliza el ejecutor de entrega ya probado. Cada ronda vuelve a leer odometría,
asigna tres basuras pendientes, calcula CBS y elimina en Gazebo solo las recogidas.
No modifica los seguidores ni los algoritmos previos.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from cbs_entrega_depositos_ros import PickupDeliveryExecutor


class FullMissionExecutor(PickupDeliveryExecutor):
    def __init__(self, summary_file='/tmp/swarm_mision_resumen.json', **kwargs):
        self.round_index = 1
        self.history = []
        self.summary_file = Path(summary_file)
        super().__init__(**kwargs)
        self.node.get_logger().info('MISIÓN: 2 rondas, 6 basuras, 3 Summit; asignación euclidiana')

    def completed_round(self):
        """Registrar entrega completada antes de reutilizar la misma instancia."""
        if not self.assignments or len(self.assignments) != 3:
            raise RuntimeError('La ronda finalizó sin tres asignaciones')
        names = [n for n, _ in self.assignments]
        if len(set(names)) != 3 or any(n in self.dynamic.pending for n in names):
            raise RuntimeError('La ronda contiene basuras duplicadas o no recogidas')
        self.history.append({
            'ronda': self.round_index,
            'entregadas': [
                {'robot': a, 'basura': names[i],
                 'color': self.dynamic.trash[names[i]].color}
                for i, a in enumerate(self.agents)
            ],
            'pendientes_tras_entrega': sorted(self.dynamic.pending),
        })

    def next_round(self):
        """Reiniciar solamente el estado de la ronda; conservar el grafo dinámico."""
        if len(self.dynamic.pending) != 3 or self.round_index != 1:
            raise RuntimeError('Se esperaban 3 basuras para iniciar la segunda ronda')
        self.round_index = 2
        self.assignments = None
        self.planned = ()
        self.delivery_goals = None
        self.finished_removal = False
        self.delivery_odom_wait_started = None
        self.step_idx = 0
        self.target = {}
        self.holding_since = {a: None for a in self.agents}
        self.phase = 'WAIT_ODOM'
        self.stage = 'PICKUP'
        # El estado previo no debe confundirse con la segunda recogida.
        self.state_file.unlink(missing_ok=True)
        self.node.get_logger().info(
            'RONDA 2: nuevas posiciones desde odometría; 3 basuras pendientes; '
            'CBS usará el grafo actualizado')

    def finish_mission(self):
        if self.dynamic.pending or len(self.history) != 2:
            raise RuntimeError('No se completaron las seis recogidas y entregas')
        picked = [e['basura'] for rnd in self.history for e in rnd['entregadas']]
        if len(picked) != 6 or len(set(picked)) != 6:
            raise RuntimeError('Registro de misión incompleto o con duplicados')
        payload = {
            'status': 'completed',
            'robots': list(self.agents),
            'rounds': self.history,
            'collected_and_delivered': len(picked),
            'pending': sorted(self.dynamic.pending),
            'vertices_free': len(self.dynamic.vertices),
            'edges_free': len(self.dynamic.edges),
        }
        self.summary_file.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        self.node.get_logger().info(
            'MISIÓN COMPLETADA: 6/6 basuras recogidas y entregadas; '
            '0 pendientes; resumen=' + str(self.summary_file))
        self.stop_all()
        self.stage = 'MISSION_COMPLETE'
        self.phase = 'DONE'

    def tick(self):
        if self.stage in ('MISSION_COMPLETE', 'FAILED'):
            return
        # Ejecuta la misma recogida, eliminación confirmada y entrega que ya funcionó.
        super().tick()
        if self.stage != 'COMPLETE':
            return
        try:
            self.completed_round()
            self.node.get_logger().info(
                f'RONDA {self.round_index} ENTREGADA: '
                f'3 basuras; quedan {len(self.dynamic.pending)}')
            if self.round_index == 1:
                self.next_round()
            else:
                self.finish_mission()
        except (RuntimeError, OSError, ValueError) as exc:
            self.node.get_logger().error(f'MISIÓN INTERRUMPIDA: {exc}')
            self.stop_all()
            self.stage = 'FAILED'
            self.phase = 'ABORT'


def main():
    import rclpy
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--world', default=None)
    ap.add_argument('--state-file', default='/tmp/swarm_recogida_estado.json')
    ap.add_argument('--summary-file', default='/tmp/swarm_mision_resumen.json')
    ap.add_argument('--max-ct', type=int, default=20000)
    ap.add_argument('--max-time', type=int, default=65)
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    mission = FullMissionExecutor(world=args.world, state_file=args.state_file,
                                  summary_file=args.summary_file,
                                  max_ct=args.max_ct, max_time=args.max_time)
    try:
        rclpy.spin(mission.node)
    except KeyboardInterrupt:
        pass
    finally:
        mission.stop_all()
        mission.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
