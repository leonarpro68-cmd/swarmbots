#!/usr/bin/env python3
"""Validación supervisada SwarmBots: preparar, ejecutar y resumir por semilla.

No inicia/detiene Gazebo automáticamente, ni modifica CBS. Cada ejecución física
requiere Gazebo nuevo, bridge /clock y autorización explícita.
"""
from __future__ import annotations
import argparse
import csv
import os
import select
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent
FIELDS = ('seed', 'status', 'duration_wall_s', 'pickup_ack', 'detach_ack', 'rounds',
          'cbs_pickup_costs', 'cbs_delivery_costs', 'cbs_pickup_steps',
          'cbs_delivery_steps', 'dds_retries', 'align_retries', 'log', 'note')


def paths(base, seed):
    d = base / f'seed_{seed}'
    return d, d / 'scenario.json', d / 'world.sdf', d / 'mission'


def run_checked(cmd):
    print('+', ' '.join(map(str, cmd)), flush=True)
    subprocess.run(list(map(str, cmd)), check=True, cwd=ROOT)


def prepare(args):
    d, scenario, world, _ = paths(args.output_dir, args.seed)
    if d.exists() and any(d.iterdir()):
        raise RuntimeError(f'Carpeta no vacía: {d}. No sobrescribo pruebas anteriores.')
    d.mkdir(parents=True, exist_ok=True)
    try:
        run_checked([sys.executable, ROOT/'escenario_aleatorio_swarm.py', '--seed', args.seed,
                     '--output', scenario])
        run_checked([sys.executable, ROOT/'swarm_cbs_grasp_prepare.py',
                     '--scenario', scenario, '--world', args.world_original,
                     '--output', world])
        # SDF original contiene texto UTF-8 con declaración ASCII; corregir
        # declaración sin alterar mundo original.
        s = world.read_text(encoding='utf-8')
        s = re.sub(r'(<\?xml\s+[^>]*encoding\s*=\s*[\'\"])ASCII([\'\"])',
                   r'\1UTF-8\2', s, count=1, flags=re.I)
        world.write_text(s, encoding='utf-8')
        import xml.etree.ElementTree as ET
        ET.parse(world)
        meta = {'seed': args.seed, 'created_utc': datetime.now(timezone.utc).isoformat(),
                'status': 'prepared', 'scenario': str(scenario), 'world': str(world)}
        (d/'metadata.json').write_text(json.dumps(meta, indent=2), encoding='utf-8')
    except Exception:
        print('PREPARACIÓN FALLIDA; carpeta conservada para diagnóstico:', d, file=sys.stderr)
        raise
    print('\nPREPARADO: ', d)
    print('Terminal 1 (Gazebo):')
    print(f'ros2 launch swarm_worlds swarm_cbs_grasp_base.launch.py seed:={args.seed} '
          f'scenario:={scenario} world_sdf:={world}')
    print("Terminal 2 (/clock):")
    print("ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'")
    print('Terminal 3 (misión, solo después de inspeccionar Gazebo):')
    print(f'python3 scripts/validar_semillas_swarm.py run --seed {args.seed} --output-dir {args.output_dir}')


def analyze_log(s):
    def values(pattern):
        return [int(x) for x in re.findall(pattern, s)]
    # Coste CBS discreto, NO metros ni duración de navegación.
    pickup_cost = values(r'CBS OK:\s*costo=(\d+)')
    delivery_cost = values(r'CBS ENTREGA OK:\s*costo=(\d+)')
    pickup_steps = values(r'CBS OK:[^\n]*pasos=(\d+)')
    delivery_steps = values(r'CBS ENTREGA OK:[^\n]*pasos=(\d+)')
    return {
        'cbs_pickup_costs': ';'.join(map(str, pickup_cost)),
        'cbs_delivery_costs': ';'.join(map(str, delivery_cost)),
        'cbs_pickup_steps': ';'.join(map(str, pickup_steps)),
        'cbs_delivery_steps': ';'.join(map(str, delivery_steps)),
        'dds_retries': len(re.findall(r'REENVIO', s, re.I)),
        'align_retries': len(re.findall(r'ALIGN REENVIO', s, re.I)),
    }


def run(args):
    d, scenario, world, prefix = paths(args.output_dir, args.seed)
    if not scenario.is_file() or not world.is_file():
        raise RuntimeError('Escenario no preparado. Ejecuta prepare primero.')
    if json.loads(scenario.read_text(encoding='utf-8')).get('seed') != args.seed:
        raise RuntimeError('La semilla del JSON no coincide.')
    files = [d/'run.json', d/'mission_console.log']
    files += [Path(f'{prefix}_ronda{i}_{suffix}') for i in (1, 2)
              for suffix in ('estado.json', 'transporte.json', 'log')]
    occupied = [str(p) for p in files if p.exists()]
    if occupied:
        raise RuntimeError('No repetir sobre estado existente: ' + ', '.join(occupied))
    marker = input(f'Confirma Gazebo NUEVO de seed={args.seed}, 6 basuras, 3 pinzas vacías, '
                   '/clock activo. Escribe INICIAR_SEMILLA: ').strip()
    if marker != 'INICIAR_SEMILLA':
        print('Cancelado sin enviar comandos.')
        return
    cmd = [sys.executable, str(ROOT/'mision_fisica_completa.py'), '--scenario', str(scenario),
           '--prefix', str(prefix), '--execute']
    log = d/'mission_console.log'
    record = d/'run.json'
    started = datetime.now(timezone.utc).isoformat()
    begin = time.monotonic()
    rc = 1
    interrupted = False
    print('Ejecutando:', ' '.join(cmd), flush=True)
    with log.open('wb') as outf:
        proc = None
        try:
            # stdin heredado para responder a confirmaciones; stdout reemitido
            # en vivo incluso cuando los prompts no terminan en nueva línea.
            proc = subprocess.Popen([sys.executable, '-u', *cmd[1:]],
                                    stdin=None, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, cwd=ROOT,
                                    bufsize=0, env={**os.environ, 'PYTHONUNBUFFERED': '1'})
            while True:
                ready, _, _ = select.select([proc.stdout], [], [], .25)
                if ready:
                    chunk = os.read(proc.stdout.fileno(), 4096)
                    if chunk:
                        outf.write(chunk)
                        outf.flush()
                        sys.stdout.buffer.write(chunk)
                        sys.stdout.buffer.flush()
                    else:
                        break
                if proc.poll() is not None and not ready:
                    # Retirar cualquier resto pendiente antes de concluir.
                    chunk = os.read(proc.stdout.fileno(), 4096)
                    if chunk:
                        outf.write(chunk)
                        sys.stdout.buffer.write(chunk)
                        sys.stdout.buffer.flush()
                    else:
                        break
            rc = proc.wait()
        except KeyboardInterrupt:
            interrupted = True
            if proc is not None:
                proc.terminate()
                try:
                    proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
            rc = 130
    elapsed = round(time.monotonic()-begin, 2)
    output = log.read_text(encoding='utf-8', errors='replace')
    states = []
    for i in (1, 2):
        p = Path(f'{prefix}_ronda{i}_estado.json')
        try:
            states.append(json.loads(p.read_text(encoding='utf-8')))
        except (OSError, ValueError):
            states.append({})
    releases = [set(s.get('completed_releases') or []) for s in states]
    complete = bool(rc == 0 and 'DOS RONDAS FINALIZADAS: 6 AGARRES + 6 DETACH REGISTRADOS.' in output
                    and len(releases[0]) == 3 and len(releases[1]) == 3
                    and len(releases[0] | releases[1]) == 6)
    status = 'completed_pending_visual_check' if complete else ('interrupted' if interrupted else 'failed')
    record_data = {
        'seed': args.seed, 'status': status, 'duration_wall_s': elapsed,
        'started_utc': started, 'ended_utc': datetime.now(timezone.utc).isoformat(),
        'returncode': rc, 'pickup_ack': 6 if complete else None,
        'detach_ack': len(releases[0] | releases[1]),
        'rounds': sum(len(a) == 3 for a in releases),
        'log': str(log), 'note': 'Requiere inspección visual; los ACK no prueban la posición final.',
        **analyze_log(output)
    }
    record.write_text(json.dumps(record_data, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    print('\nRESULTADO:', status, '| duración real (incluye aprobaciones):', elapsed, 's')
    print('Logs:', log, '| reporte:', record)
    if not complete:
        print('MISIÓN INCOMPLETA: no reintentar en el mismo Gazebo; inspecciona primero.', file=sys.stderr)
        raise RuntimeError(f'Código {rc}. Revisa {log}')


def report(args):
    rows = []
    for file in sorted(args.output_dir.glob('seed_*/run.json')):
        try:
            row = json.loads(file.read_text(encoding='utf-8'))
            rows.append({x: row.get(x, '') for x in FIELDS})
        except (OSError, ValueError):
            continue
    args.output_dir.mkdir(parents=True, exist_ok=True)
    out = args.output_dir/'results.csv'
    with out.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    ok = [r for r in rows if r['status'] == 'completed_pending_visual_check']
    print(f'Reporte: {out} | pruebas registradas={len(rows)} | completadas con ACK={len(ok)}')
    print('NOTA: duración incluye espera humana; costos CBS son unidades del planificador, no metros.')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='command', required=True)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument('--output-dir', type=Path, default=Path('/tmp/swarm_validacion'))
    prep = sub.add_parser('prepare', parents=[common])
    prep.add_argument('--seed', required=True, type=int)
    prep.add_argument('--world-original', type=Path, required=True)
    r = sub.add_parser('run', parents=[common])
    r.add_argument('--seed', required=True, type=int)
    sub.add_parser('report', parents=[common])
    args = p.parse_args()
    try:
        {'prepare': prepare, 'run': run, 'report': report}[args.command](args)
    except (RuntimeError, subprocess.CalledProcessError, OSError, ValueError) as exc:
        p.exit(1, f'ERROR: {exc}\n')

if __name__ == '__main__':
    main()
