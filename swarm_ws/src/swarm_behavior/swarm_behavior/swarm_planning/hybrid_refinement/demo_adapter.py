"""Prueba real y reproducible de CBS -> calendario nominal, sin ROS/Gazebo.

Con el workspace en PYTHONPATH o después de un colcon build:
python3 -m swarm_behavior.swarm_planning.hybrid_refinement.demo_adapter \\
    --out /tmp/swarm_cbs_hybrid_schedule.json
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path

from .. import cbs
from .adapter import from_cbs_paths


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=Path('/tmp/swarm_cbs_hybrid_schedule.json'))
    parser.add_argument('--max-speed', type=float, default=0.60)
    args = parser.parse_args()

    # Mismo escenario de demostración definido en cbs.main(). No sustituye
    # rutas de recogida ni depende de un escenario de basuras particular.
    starts = ((8, 18), (14, 18), (11, 14))
    goals = ((14, 18), (8, 18), (11, 22))
    graph = cbs.Graph()
    result = cbs.solve(starts, goals, graph=graph)
    schedule = from_cbs_paths(result['paths'], graph=graph, max_speed_m_s=args.max_speed)
    payload = schedule.to_dict()
    payload['cbs'] = {'source': 'cbs.solve', 'cost': result['cost'],
                      'expanded': result['expanded'], 'generated': result['generated']}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(f"CBS: coste={result['cost']} ticks, CT expandidos={result['expanded']}")
    print(f"Adaptador: {len(schedule.robot_names)} Summit, {schedule.ticks} ticks sincronizados")
    print(f"Mayor arista: {schedule.max_step_m:.4f} m; separación: {schedule.min_separation_m:.4f} m")
    print(f"Tiempo NOMINAL por tick: {schedule.tick_seconds:.4f} s; duración: {schedule.duration_s:.4f} s")
    print(f"WAIT añadidos al final de cada ruta: {schedule.was_padded}")
    print('JSON:', args.out)
    print('ATENCIÓN: este JSON NO es ejecutable directamente en Gazebo')


if __name__ == '__main__':
    main()
