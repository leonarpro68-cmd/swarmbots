#!/usr/bin/env python3
"""CBS recogida ronda 2: marca como recogidas lógicamente las 3 basuras de ronda 1.

No borra modelos Gazebo. NO lanza agarres. Reutiliza PickupExecutor original.
"""
import argparse
import json
from pathlib import Path
import rclpy
from swarm_behavior.swarm_planning.ros_pickup_executor import PickupExecutor


def previous_names(state_file, scenario):
    state = json.loads(Path(state_file).read_text(encoding='utf-8'))
    if state.get('status') != 'completed_pending_visual_check' or state.get('stage') != 'round_1_finished':
        raise ValueError('Ronda 1 no finalizada según el archivo de estado')
    assigned = state.get('assigned', {})
    done = state.get('completed_releases', [])
    names = tuple(assigned.get(r) for r in ('summit0','summit1','summit2'))
    if len(set(names)) != 3 or set(done) != set(names) or len(done) != 3:
        raise ValueError('Se requieren tres entregas ACK distintas de la ronda 1')
    items = {e['name'] for e in json.loads(Path(scenario).read_text(encoding='utf-8'))['elements'] if e['type']=='trash'}
    if not set(names) <= items:
        raise ValueError('El estado de ronda 1 no coincide con este escenario')
    if Path(state.get('scenario','')).resolve() != Path(scenario).resolve():
        raise ValueError('Ruta de escenario diferente a la de la ronda 1')
    return names


class Round2Pickup(PickupExecutor):
    def __init__(self, scenario, previous, **kwargs):
        super().__init__(scenario, **kwargs)
        for name in previous:
            self.coordinator.bridge.confirm_collected(name)
        if len(self.coordinator.pending) != 3:
            raise ValueError('Se esperaban exactamente tres basuras pendientes')
        self.node.get_logger().info(f'RONDA 2: ignorando basuras entregadas {previous}; pendientes={sorted(self.coordinator.pending)}')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--previous-state', required=True)
    ap.add_argument('--step-timeout', type=float, default=90.)
    ap.add_argument('--max-ct', type=int, default=20000)
    ap.add_argument('--max-time', type=int, default=65)
    args, ros_args = ap.parse_known_args()
    prev = previous_names(args.previous_state, args.scenario)
    rclpy.init(args=ros_args)
    exe = None
    try:
        exe = Round2Pickup(args.scenario, prev, step_timeout=args.step_timeout,
                           max_ct=args.max_ct, max_time=args.max_time)
        while rclpy.ok() and exe.state not in ('DONE','FAILED'):
            rclpy.spin_once(exe.node, timeout_sec=.2)
        return 0 if exe.state == 'DONE' else 1
    except KeyboardInterrupt:
        return 130
    finally:
        if exe is not None:
            exe.close()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    raise SystemExit(main())
