#!/usr/bin/env python3
"""Lanzador supervisado de ambas rondas V4.1 en el MISMO Gazebo.

No reinicia mundos, no reanuda estados fallidos y conserva los prompts humanos
ENTREGAR/CONTINUAR de cada ronda. Ejecutar SOLO desde simulación nueva.
"""
from __future__ import annotations
import argparse
import json
import subprocess
import sys
from pathlib import Path

DIR = Path(__file__).resolve().parent
STATE1 = Path('/tmp/mision_fisica_ronda1_v41_estado.json')
STATE2 = Path('/tmp/mision_fisica_ronda2_v41_estado.json')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scenario', required=True, type=Path)
    ap.add_argument('--execute', action='store_true')
    args = ap.parse_args()
    required = [DIR/'mision_fisica_ronda1_v41.py', DIR/'mision_fisica_ronda2_v41.py',
                DIR/'ros_transporte_fisico_cbs_v2.py', DIR/'ros_transporte_ronda2_cbs_v2.py']
    absent = [str(p) for p in required if not p.is_file()]
    if absent:
        ap.error('Faltan scripts: ' + ', '.join(absent))
    if not args.scenario.is_file():
        ap.error(f'Escenario inexistente: {args.scenario}')
    if STATE1.exists() or STATE2.exists():
        ap.error('Existe un estado V4.1 anterior: NO relanzar sobre una simulación parcial. '
                 f'Comprueba {STATE1} y {STATE2}.')
    print('V4.1: 3 agarres paralelos + CBS + entregas por oleadas; dos rondas.', flush=True)
    print('Se pedirán autorizaciones humanas antes de las liberaciones.', flush=True)
    if not args.execute:
        print('DRY RUN: ninguna orden enviada. Usa --execute solo en un Gazebo recién iniciado.')
        return 0
    for number in (1, 2):
        cmd = [sys.executable, str(DIR/f'mision_fisica_ronda{number}_v41.py'),
               '--scenario', str(args.scenario), '--execute']
        if number == 2:
            previous = json.loads(STATE1.read_text(encoding='utf-8'))
            if previous.get('status') != 'completed_pending_visual_check' or previous.get('stage') != 'round_1_finished' or len(previous.get('completed_releases', [])) != 3:
                print('Ronda 1 no confirmada: NO ejecutar ronda 2', file=sys.stderr)
                return 1
            print('\nInspecciona Gazebo: tres basuras en depósitos, tres pinzas vacías.', flush=True)
            try:
                if input('Escribe RONDA2 para autorizar la segunda ronda: ').strip() != 'RONDA2':
                    print('Segunda ronda no iniciada. Estado de ronda 1 conservado.')
                    return 2
            except EOFError:
                print('Entrada no disponible: NO iniciar ronda 2.', file=sys.stderr)
                return 2
        print(f'\n=== V4.1 RONDA {number} ===', flush=True)
        try:
            code = subprocess.call(cmd)
        except KeyboardInterrupt:
            print('Interrumpido: inspecciona estados y Gazebo; NO relanzar automáticamente.', file=sys.stderr)
            return 130
        if code != 0:
            print(f'Ronda {number} terminada con código {code}; NO continuar.', file=sys.stderr)
            return code
    print('\nV4.1: DOS RONDAS TERMINADAS. Verificar visualmente seis depósitos de basura.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
