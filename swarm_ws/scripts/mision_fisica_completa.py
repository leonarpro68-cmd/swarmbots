#!/usr/bin/env python3
"""SwarmBots: dos rondas físicas supervisadas usando coordinadores ya validados.

NO reinicia Gazebo ni retoma una misión parcial. No modifica A*/CBS.
Ronda 1: mision_fisica_ronda1_v411.py
Ronda 2: mision_fisica_ronda2_v41.py
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_PREFIX = Path('/tmp/swarm_mision_completa_v411')


def load_completed(path: Path, stage: str) -> dict:
    try:
        state = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError) as exc:
        raise RuntimeError(f'No se pudo leer estado de {stage}: {exc}') from exc
    if state.get('status') != 'completed_pending_visual_check' or state.get('stage') != stage:
        raise RuntimeError(f'Estado final no confirmado en {path}: {state.get("status")}, {state.get("stage")}')
    assigned = state.get('assigned', {})
    releases = state.get('completed_releases', [])
    if (not isinstance(assigned, dict) or len(assigned) != 3 or
            len(set(assigned.values())) != 3 or not isinstance(releases, list) or
            len(releases) != 3 or set(releases) != set(assigned.values())):
        raise RuntimeError(f'Estado inconsistente en {path}: asignaciones y liberaciones no coinciden')
    return state


def authorized(message: str, keyword: str) -> bool:
    try:
        return input(message).strip() == keyword
    except (EOFError, KeyboardInterrupt):
        return False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', required=True, type=Path)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--prefix', type=Path, default=DEFAULT_PREFIX,
                        help='Prefijo de estados y logs; usar uno nuevo SOLO con Gazebo recién iniciado')
    args = parser.parse_args()
    scenario = args.scenario.resolve()
    if not scenario.is_file():
        parser.error(f'Escenario inexistente: {scenario}')
    programs = (HERE / 'mision_fisica_ronda1_v411.py', HERE / 'mision_fisica_ronda2_v41.py')
    for item in programs:
        if not item.is_file():
            parser.error(f'Falta script: {item}')
    prefix = args.prefix
    state1 = Path(f'{prefix}_ronda1_estado.json')
    state2 = Path(f'{prefix}_ronda2_estado.json')
    log1 = Path(f'{prefix}_ronda1.log')
    log2 = Path(f'{prefix}_ronda2.log')
    transport1 = Path(f'{prefix}_ronda1_transporte.json')
    transport2 = Path(f'{prefix}_ronda2_transporte.json')
    outputs = (state1, state2, log1, log2, transport1, transport2)

    print('SWARMBOTS: DOS RONDAS SUPERVISADAS', flush=True)
    print(f'Escenario: {scenario}', flush=True)
    print('Ronda 1: V4.1.1 | Ronda 2: V4.1', flush=True)
    print('Agarres paralelos, transporte CBS V2, entregas por oleadas.', flush=True)
    if not args.execute:
        print('DRY RUN: no se inicia ROS ni se ordena mover robots.', flush=True)
        print('Archivos que se crearán al ejecutar:')
        print(f'  {state1}\n  {state2}')
        return 0

    occupied = [str(p) for p in outputs if p.exists()]
    if occupied:
        parser.error('Existen archivos de esta ejecución; NO sobrescribir ni relanzar sobre misión parcial:\n' +
                     '\n'.join(occupied))
    if not authorized('Confirma Gazebo NUEVO (6 basuras, 3 pinzas vacías). Escribe INICIAR: ', 'INICIAR'):
        print('No autorizado: ningún robot recibió órdenes.', flush=True)
        return 2

    prefix.parent.mkdir(parents=True, exist_ok=True)
    jobs = [
        [sys.executable, str(programs[0]), '--scenario', str(scenario), '--state', str(state1),
         '--log', str(log1), '--transport-state', str(transport1), '--execute'],
        [sys.executable, str(programs[1]), '--scenario', str(scenario), '--previous-state', str(state1),
         '--state', str(state2), '--log', str(log2), '--transport-state', str(transport2), '--execute'],
    ]
    try:
        print('\n========= RONDA 1 =========', flush=True)
        code = subprocess.call(jobs[0])
        if code != 0:
            print(f'RONDA 1 falló (código {code}). NO iniciar ronda 2.', file=sys.stderr)
            return 1
        first = load_completed(state1, 'round_1_finished')
        print('RONDA 1: 3 recogidas y 3 liberaciones registradas.', flush=True)
        if not authorized('Inspecciona Gazebo: basuras en depósitos, 3 pinzas vacías. Escribe RONDA2: ', 'RONDA2'):
            print('Ronda 2 no autorizada. Conserva Gazebo y estados; no reinicies esta misión.', flush=True)
            return 2
        print('\n========= RONDA 2 =========', flush=True)
        code = subprocess.call(jobs[1])
        if code != 0:
            print(f'RONDA 2 falló (código {code}). No relanzar automáticamente.', file=sys.stderr)
            return 1
        second = load_completed(state2, 'round_2_finished')
        delivered1 = set(first['completed_releases'])
        delivered2 = set(second['completed_releases'])
        if delivered1 & delivered2 or len(delivered1 | delivered2) != 6:
            raise RuntimeError('Entrega total inconsistente: se esperan seis nombres distintos')
        print('\nDOS RONDAS FINALIZADAS: 6 AGARRES + 6 DETACH REGISTRADOS.', flush=True)
        print('Comprueba visualmente la colocación final en Gazebo.', flush=True)
        print(f'Estados: {state1} | {state2}', flush=True)
        return 0
    except KeyboardInterrupt:
        print('\nInterrupción: inspeccionar Gazebo y estados antes de continuar.', file=sys.stderr)
        return 130
    except RuntimeError as exc:
        print(f'ERROR DE VALIDACIÓN: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
