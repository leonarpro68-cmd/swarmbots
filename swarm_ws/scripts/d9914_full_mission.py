#!/usr/bin/env python3
"""D9.9.14: supervisor secuencial de las 4 etapas históricas.

No publica mensajes ROS: invoca exclusivamente D9.9.7 y D9.9.13 existentes.
Sin --execute sólo valida archivos, hashes, y hace diagnóstico de pickup1.
Con --execute realiza alineación + preflight + ejecución por etapa.
No realiza agarre, transporte físico de basura, ni detach. No modifica D8.
"""
import argparse
import hashlib
import json
import os
import signal
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

STAGES = ('pickup1', 'delivery1', 'pickup2', 'delivery2')
SHAS = {
    'pickup1': '3235223232eb1b47e36c46cc107086c933e5020917aa7b78758b9b183985e2dd',
    'delivery1': '70bb3550dfb3a937f002f5b8e83d621ef8b229180f6a0b8c6bdc626de19fb40b',
    'pickup2': '7f626f10002aeb40be8fd3684e8cc2ee49512ffbf01fdc969f489584c1f94e11',
    'delivery2': 'ce13db58df80acdfc4bedebfc115c3a37a2bc5dad501727bfa9812ea2d1b2b60',
}


def write_report(path, report):
    tmp = path / 'report.json.tmp'
    tmp.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    os.replace(tmp, path / 'report.json')


def call_step(argv, expected, path, report, stage, action):
    dest = path / f'{stage}_{action}.json'
    cmd = [sys.executable, *argv, '--output', str(dest)]
    print('\n[D9914]', stage, action, flush=True)
    # Outputs log separately; explicit stdbuf isn't required.
    with (path / 'pipeline.log').open('a', encoding='utf-8') as log:
        log.write('\n$ ' + ' '.join(cmd) + '\n'); log.flush()
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, errors='replace')
        log.write(result.stdout + '\nEXIT_CODE=' + str(result.returncode) + '\n')
    if result.stdout:
        print(result.stdout[-3500:], flush=True)
    try:
        content = json.loads(dest.read_text(encoding='utf-8'))
    except (FileNotFoundError, ValueError) as exc:
        raise RuntimeError(f'{stage}/{action}: sin informe JSON válido; consulta pipeline.log: {exc}')
    state = content.get('status')
    entry = dict(stage=stage, action=action, status=state, exit_code=result.returncode,
                 report=str(dest), problems=content.get('problems', []))
    report['steps'].append(entry)
    write_report(path, report)
    if result.returncode != 0 or state not in expected:
        raise RuntimeError(f'{stage}/{action}: status={state!r}, exit={result.returncode}, problemas={content.get("problems")}')
    return content


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute', action='store_true', help='Autoriza mover los robots en las 4 etapas')
    p.add_argument('--confirm-runtime-v2', action='store_true', help='Confirmación explícita del protocolo V2 cargado')
    p.add_argument('--output-dir', default='scripts/resultados/d9914_mision_001')
    p.add_argument('--reference-dir', default='scripts/d996_reference')
    p.add_argument('--scripts-dir', default='scripts')
    a = p.parse_args()
    root = Path(a.output_dir).resolve()
    # Nunca sobreescribir resultados previos ni repetir movimientos accidentalmente.
    if root.exists():
        p.error(f'La carpeta de salida ya existe: {root}. Usa un nombre nuevo para esta corrida.')
    root.mkdir(parents=True)
    report = dict(schema='swarm.d9914.full_mission.v1',
                  status='INITIALIZING', execution_requested=a.execute,
                  created_utc=datetime.now(timezone.utc).isoformat(),
                  stages=list(STAGES), steps=[], completed_stages=[],
                  physical_trash_handling=False, problems=[])
    write_report(root, report)
    try:
        if a.execute and not a.confirm_runtime_v2:
            raise RuntimeError('REQUIRE_EXPLICIT_RUNTIME_V2_CONFIRMATION')
        scripts = Path(a.scripts_dir).resolve()
        aligner = scripts / 'd997_initial_alignment_path.py'
        coordinator = scripts / 'd9913_stage_coordinator.py'
        if not aligner.is_file() or not coordinator.is_file():
            raise RuntimeError('No se encuentran D9.9.7 y D9.9.13 en scripts/')
        references = {}
        for stage in STAGES:
            src = Path(a.reference_dir).resolve() / (stage + '_d994_NON_EXECUTABLE.npz')
            if not src.is_file():
                raise RuntimeError(f'{stage}: trayectoria histórica ausente: {src}')
            sha = hashlib.sha256(src.read_bytes()).hexdigest()
            if sha != SHAS[stage]:
                raise RuntimeError(f'{stage}: SHA256 incorrecto: {sha}')
            references[stage] = src
        report['verified_hashes'] = SHAS
        report['status'] = 'REFERENCES_VERIFIED'
        write_report(root, report)
        if not a.execute:
            # Solo se diagnostica pickup1: las etapas siguientes requieren que
            # los robots hayan llegado a los destinos de etapas previas.
            src = references['pickup1']
            call_step([str(aligner), '--trajectory', str(src)],
                      {'DRY_RUN_READY', 'ALREADY_ALIGNED'}, root, report, 'pickup1', 'alignment_dry')
            # El coordinador dry-run exige alineación previa; no equivale a un
            # examen de toda la misión, no prometer lo contrario.
            report['status'] = 'PASS_DRY_RUN_PICKUP1_AND_HASHES_NO_COMMANDS'
            write_report(root, report)
            return 0
        for stage in STAGES:
            src = references[stage]
            call_step([str(aligner), '--trajectory', str(src), '--execute'],
                      {'ALIGNED', 'ALREADY_ALIGNED'}, root, report, stage, 'alignment')
            call_step([str(coordinator), '--stage', stage],
                      {'PASS_GUARDED_DRY_RUN_NO_COMMANDS'}, root, report, stage, 'navigation_dry')
            call_step([str(coordinator), '--stage', stage, '--execute', '--confirm-runtime-v2'],
                      {'DONE_ALL_' + stage.upper()}, root, report, stage, 'navigation')
            report['completed_stages'].append(stage)
            write_report(root, report)
        report['status'] = 'DONE_ALL_FOUR_NAVIGATION_STAGES'
        write_report(root, report)
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        report['status'] = 'REJECT_STOPPED'
        report['problems'].append(f'{type(exc).__name__}: {exc}')
        write_report(root, report)
        print('[D9914] DETENIDO:', exc, file=sys.stderr)
        print('Revisa:', root / 'report.json', file=sys.stderr)
        print('No reinicies automáticamente: comprueba odometría y parada.', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
