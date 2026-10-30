#!/usr/bin/env python3
"""Orquestador V4.1 de RONDA 2 con agarres y entregas paralelos (versión de diagnóstico ALIGN), ROS 2 Jazzy + SwarmBots.

Reutiliza exclusivamente los ejecutores validados. No modifica A*/CBS,
no elimina basuras de Gazebo y no marca entrega lógica con ACK ficticios.
La confirmación humana antes de la primera liberación es deliberada.
Requiere Gazebo y /clock activos; ejecutar desde ~/swarm_ws en Docker.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

from agarres_paralelos_cbs import parallel_grasps
from entregas_paralelas_cbs_auto import release_waves

ROBOTS = ('summit0', 'summit1', 'summit2')
PICKUP_SUCCESS = '6B COMPLETADA'
GRASP_SUCCESS = ('ALINEAR CONFIRMADO', 'JOINT CONFIRMADO', 'REGRESO CONFIRMADO')
TRANSPORT_SUCCESS = 'TRANSPORTE COMPLETADO'
RELEASE_SUCCESS = ('DETACH CONFIRMADO', 'REGRESO CONFIRMADO')


def run_stage(label: str, cmd: list[str], checks: tuple[str, ...], logfile: Path):
    print(f'\n{"="*70}\nETAPA: {label}\nCOMANDO: {" ".join(cmd)}\n{"="*70}', flush=True)
    captured = []
    with logfile.open('a', encoding='utf-8') as log:
        log.write(f'\n\n### {label}\n$ {" ".join(cmd)}\n')
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, bufsize=1, env={**os.environ, 'PYTHONUNBUFFERED': '1'})
        try:
            assert proc.stdout is not None
            for line in proc.stdout:
                print(line, end='', flush=True)
                log.write(line)
                log.flush()
                captured.append(line)
            code = proc.wait()
        except KeyboardInterrupt:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
            raise
    output = ''.join(captured)
    if code != 0 or any(marker not in output for marker in checks):
        raise RuntimeError(f'{label}: fallo (codigo={code}; marcadores no encontrados='
                           f'{[x for x in checks if x not in output]}); NO continuar')
    print(f'ETAPA CONFIRMADA: {label}', flush=True)
    return output


def save(path: Path, payload: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    tmp.replace(path)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scenario', required=True, type=Path)
    p.add_argument('--previous-state', type=Path, default=Path('/tmp/mision_fisica_ronda1_v41_estado.json'), help='Estado completado de ronda 1')
    p.add_argument('--execute', action='store_true', help='Autorizar inicio de la ronda real')
    p.add_argument('--state', type=Path, default=Path('/tmp/mision_fisica_ronda2_v41_estado.json'))
    p.add_argument('--log', type=Path, default=Path('/tmp/mision_fisica_ronda2_v41.log'))
    p.add_argument('--transport-state', type=Path, default=Path('/tmp/mision_fisica_ronda2_v41_transporte.json'))
    p.add_argument('--max-start-distance', type=float, default=2.0)
    args = p.parse_args()
    from ros_recogida_ronda2_cbs import previous_names
    try:
        previous = previous_names(args.previous_state, args.scenario)
    except (OSError, ValueError, KeyError) as exc:
        p.error(f'Archivo de estado de ronda 1 inválido: {exc}')
    data = json.loads(args.scenario.read_text(encoding='utf-8'))
    trash = {e['name']: e for e in data['elements'] if e['type'] == 'trash'}
    deposits = {e['color']: e['name'] for e in data['elements'] if e['type'] == 'deposit'}
    if len(trash) != 6 or len(set(trash) - set(previous)) != 3:
        p.error('Se requieren exactamente 6 basuras, 3 previas y 3 pendientes')
    if len(deposits) != 3:
        p.error('Se requieren tres depósitos de colores diferentes')
    scripts = Path(__file__).resolve().parent
    for fn in ('prueba_alinear_agarrar_cbs.py', 'ros_transporte_fisico_cbs_v2.py', 'prueba_entrega_local_cbs.py', 'ros_recogida_ronda2_cbs_v2.py', 'ros_transporte_ronda2_cbs_v2.py', 'agarres_paralelos_cbs.py', 'entregas_paralelas_cbs.py'):
        if not (scripts / fn).is_file():
            p.error(f'Falta {scripts/fn}')
    if args.max_start_distance < 1.8 or args.max_start_distance > 2.2:
        p.error('--max-start-distance debe estar entre 1.8 y 2.2 m en esta prueba')
    if args.state.exists():
        p.error(f'Ya existe {args.state}. Evitamos un segundo arranque accidental. '
                'Revísalo y utiliza una ruta --state diferente únicamente con una simulación nueva.')
    print('MISION FISICA: RONDA 2, MODO SUPERVISADO')
    print('Ya entregadas en ronda 1:', previous)
    print('Escenario:', args.scenario)
    print('El orden de las basuras se obtendrá del plan real de CBS (no se presupone seed=123).')
    print('No reinicies Gazebo: esta ronda parte del estado físico FINAL de la ronda 1.')
    print('AUTO: las liberaciones siguen exigiendo los ACK físicos y el preflight geométrico.')
    if not args.execute:
        print('DRY RUN: no se ha publicado ninguna orden. Añade --execute para comenzar.')
        return 0
    status = {'status': 'running', 'stage': 'init', 'scenario': str(args.scenario),
              'assigned': {}, 'completed_releases': [], 'previous_delivered': list(previous)}
    args.log.parent.mkdir(parents=True, exist_ok=True)
    save(args.state, status)
    try:
        output = run_stage('CBS RECOGIDA', [sys.executable, str(scripts/'ros_recogida_ronda2_cbs_v2.py'), '--scenario', str(args.scenario),
            '--previous-state', str(args.previous_state)],
            (PICKUP_SUCCESS,), args.log)
        matches = re.findall(r'\b(summit[012]): basura=([^,\s]+), meta=', output)
        assignments = dict(matches)
        if set(assignments) != set(ROBOTS) or len(set(assignments.values())) != 3:
            raise RuntimeError(f'No se pudieron obtener tres asignaciones CBS únicas: {assignments}')
        if set(assignments.values()) & set(previous):
            raise RuntimeError('CBS seleccionó basura previamente entregada; secuencia detenida')
        if any(name not in trash for name in assignments.values()):
            raise RuntimeError('Asignación CBS contiene una basura ajena al escenario')
        status.update(stage='pickup_arrived', assigned=assignments)
        save(args.state, status)
        status['stage'] = 'parallel_grasps'
        status['confirmed_grasps'] = {}
        save(args.state, status)

        def acknowledge_grasp(robot, trash_name):
            status['confirmed_grasps'][robot] = trash_name
            save(args.state, status)

        parallel_grasps(
            scenario=args.scenario, assignments=assignments, trash=trash,
            script=scripts/'prueba_alinear_agarrar_cbs.py', logfile=args.log,
            on_success=acknowledge_grasp,
        )
        status['stage'] = 'all_grasped'
        save(args.state, status)
        run_stage('CBS TRANSPORTE', [sys.executable, str(scripts/'ros_transporte_ronda2_cbs_v2.py'),
            '--scenario', str(args.scenario), '--carried', *(assignments[r] for r in ROBOTS),
            '--output', str(args.transport_state), '--previous-state', str(args.previous_state)], (TRANSPORT_SUCCESS,), args.log)
        delivery = json.loads(args.transport_state.read_text(encoding='utf-8'))
        if delivery.get('status') != 'arrived_not_delivered' or delivery.get('carried') != assignments:
            raise RuntimeError('Archivo de transporte incompatible con cargas asignadas')
        status['stage'] = 'transport_arrived'
        save(args.state, status)
        print('\nTRANSPORTE CONFIRMADO: todos los robots mantienen las cargas.', flush=True)
        print('La geometría local aún requiere inspección visual; no existe evitación local de obstáculos.',flush=True)
        if False:  # AUTO: transporte confirmado y estado de carga revisado
            status['stage'] = 'waiting_manual_release'
            save(args.state,status)
            print('Entrega no autorizada; no se ha soltado ningún objeto.')
            return 2
        status['stage'] = 'parallel_releases'
        save(args.state,status)

        def acknowledge_release(robot, trash_name):
            if trash_name in status['completed_releases']:
                raise RuntimeError(f'ACK de release duplicado: {trash_name}')
            status['completed_releases'].append(trash_name)
            save(args.state,status)

        release_waves(
            scenario=args.scenario, assignments=assignments, trash=trash,
            deposits=deposits, transport=delivery,
            script=scripts/'prueba_entrega_local_cbs.py', logfile=args.log,
            state_callback=acknowledge_release,
            max_start_distance=args.max_start_distance,
        )
        status['status'] = 'completed_pending_visual_check'
        status['stage'] = 'round_2_finished'
        save(args.state, status)
        print('\nRONDA 2 V4.1 TERMINADA: 3 ACK de agarre y 3 ACK de liberación, todos con regreso.')
        print('Verifica en Gazebo las seis basuras entregadas. DOS RONDAS FINALIZADAS.')
        return 0
    except (RuntimeError, OSError, EOFError) as exc:
        status['status'] = 'stopped_requires_inspection'
        status['error'] = str(exc)
        save(args.state, status)
        print(f'\nMISION DETENIDA: {exc}\nNo relanzar automáticamente: revisar Gazebo y el archivo de estado.', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        status['status'] = 'interrupted_requires_inspection'
        save(args.state, status)
        print('\nInterrumpido. Inspecciona Gazebo antes de cualquier reintento.', file=sys.stderr)
        return 130


if __name__ == '__main__':
    raise SystemExit(main())
