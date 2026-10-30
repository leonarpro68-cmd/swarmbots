#!/usr/bin/env python3
"""Reanuda SOLO las oleadas de entrega V4 tras transporte físico recuperado.

No vuelve a ejecutar CBS ni agarres; exige confirmación visual humana.
"""
from __future__ import annotations
import argparse
import json
import sys
from pathlib import Path
from entregas_paralelas_cbs import release_waves

ROBOTS = {'summit0', 'summit1', 'summit2'}

def load(path):
    return json.loads(path.read_text(encoding='utf-8'))

def save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(data, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    temp.replace(path)

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scenario', type=Path, required=True)
    p.add_argument('--transport', type=Path, default=Path('/tmp/transporte_v4_recuperado.json'))
    p.add_argument('--mission-state', type=Path, default=Path('/tmp/mision_fisica_ronda1_v4_estado.json'))
    p.add_argument('--state', type=Path, default=Path('/tmp/entregas_v4_recuperadas_estado.json'))
    p.add_argument('--log', type=Path, default=Path('/tmp/entregas_v4_recuperadas.log'))
    p.add_argument('--max-start-distance', type=float, default=2.0)
    p.add_argument('--execute', action='store_true')
    args = p.parse_args()
    try:
        scenario = load(args.scenario)
        mission = load(args.mission_state)
        transport = load(args.transport)
        assignments = mission.get('assigned', {})
        trash = {e['name']: e for e in scenario['elements'] if e['type'] == 'trash'}
        deposits = {e['color']: e['name'] for e in scenario['elements'] if e['type'] == 'deposit'}
        if set(assignments) != ROBOTS or len(set(assignments.values())) != 3:
            raise ValueError(f'Asignaciones incompletas en misión: {assignments}')
        if any(v not in trash for v in assignments.values()):
            raise ValueError('Hay basuras asignadas ajenas al escenario')
        if transport.get('status') != 'arrived_not_delivered':
            raise ValueError(f'Estado de transporte incorrecto: {transport.get("status")}')
        if transport.get('carried') != assignments:
            raise ValueError('Las cargas del transporte NO coinciden con las de misión')
        if mission.get('completed_releases'):
            raise ValueError('El estado de misión ya registra descargas; no iniciar una entrega completa')
        if set(transport.get('reference_xy', {})) != ROBOTS:
            raise ValueError('Faltan las posiciones finales de algún robot')
        if not 1.8 <= args.max_start_distance <= 2.2:
            raise ValueError('max-start-distance debe estar entre 1.8 y 2.2 m')
        if args.state.exists():
            raise ValueError(f'Ya existe {args.state}; no repetir sobre la misma simulación')
        print('ENTREGAS V4 RECUPERADAS: SIN CBS, SIN NUEVOS AGARRES')
        for r in sorted(ROBOTS):
            color = trash[assignments[r]]['color']
            print(f'  {r}: {assignments[r]} -> {deposits[color]}, '
                  f'vertice={transport["reference_xy"][r]}')
        print('Se requiere Gazebo activo, pinzas aún sujetando las tres basuras y rutas locales despejadas.')
        if not args.execute:
            print('DRY RUN: sin órdenes. Añade --execute tras comprobar Gazebo.')
            return 0
        state = {'status': 'running', 'stage': 'release_waves',
                 'scenario': str(args.scenario), 'transport': str(args.transport),
                 'assigned': assignments, 'completed_releases': []}
        save(args.state, state)
        def ack(robot, trash_name):
            if trash_name in state['completed_releases']:
                raise RuntimeError(f'ACK repetido: {trash_name}')
            state['completed_releases'].append(trash_name)
            save(args.state, state)
        try:
            release_waves(scenario=args.scenario, assignments=assignments, trash=trash,
                          deposits=deposits, transport=transport,
                          script=Path(__file__).resolve().parent/'prueba_entrega_local_cbs.py',
                          logfile=args.log, state_callback=ack,
                          max_start_distance=args.max_start_distance)
            state.update(status='completed_pending_visual_check', stage='round_1_delivered')
            save(args.state, state)
            print('ENTREGAS V4 RECUPERADAS COMPLETAS: 3 DETACH + 3 REGRESOS.')
            print('Confirma en Gazebo que las tres basuras estén en los depósitos.')
            return 0
        except (RuntimeError, OSError, EOFError) as exc:
            state.update(status='stopped_requires_inspection', error=str(exc))
            save(args.state, state)
            print(f'ENTREGA INTERRUMPIDA: {exc}. No reintentar sin inspección.', file=sys.stderr)
            return 1
        except KeyboardInterrupt:
            state.update(status='interrupted_requires_inspection')
            save(args.state, state)
            print('Interrumpido: inspeccionar Gazebo y estado.',file=sys.stderr)
            return 130
    except (ValueError, OSError, KeyError, TypeError, json.JSONDecodeError) as exc:
        p.error(str(exc))

if __name__ == '__main__':
    raise SystemExit(main())
