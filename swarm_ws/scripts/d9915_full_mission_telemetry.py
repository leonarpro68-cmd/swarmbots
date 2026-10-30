#!/usr/bin/env python3
"""D9.9.15: misión automática con captura de odometría y gráfico comparativo.

Encadena alineación + ejecución de las 4 etapas como D9.9.14, pero además:
- arranca un grabador de odometría durante toda la misión,
- registra ventanas temporales por etapa,
- genera una gráfica teórico-vs-real al finalizar.
"""
import argparse
import hashlib
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

STAGES = ('pickup1', 'delivery1', 'pickup2', 'delivery2')
SHAS = {
    'pickup1': '3235223232eb1b47e36c46cc107086c933e5020917aa7b78758b9b183985e2dd',
    'delivery1': '70bb3550dfb3a937f002f5b8e83d621ef8b229180f6a0b8c6bdc626de19fb40b',
    'pickup2': '7f626f10002aeb40be8fd3684e8cc2ee49512ffbf01fdc969f489584c1f94e11',
    'delivery2': 'ce13db58df80acdfc4bedebfc115c3a37a2bc5dad501727bfa9812ea2d1b2b60',
}


def write_json(path: Path, data):
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    os.replace(tmp, path)


def call_step(argv, expected, root: Path, report: dict, stage: str, action: str):
    dest = root / f'{stage}_{action}.json'
    cmd = [sys.executable, *argv, '--output', str(dest)]
    print('\n[D9915]', stage, action, flush=True)
    with (root / 'pipeline.log').open('a', encoding='utf-8') as log:
        log.write('\n$ ' + ' '.join(cmd) + '\n'); log.flush()
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, errors='replace')
        log.write(result.stdout + '\nEXIT_CODE=' + str(result.returncode) + '\n')
    if result.stdout:
        print(result.stdout[-3500:], flush=True)
    try:
        content = json.loads(dest.read_text(encoding='utf-8'))
    except Exception as exc:
        raise RuntimeError(f'{stage}/{action}: sin informe JSON válido: {exc}')
    state = content.get('status')
    report['steps'].append({
        'stage': stage, 'action': action, 'status': state,
        'exit_code': result.returncode, 'report': str(dest),
        'problems': content.get('problems', [])
    })
    write_json(root / 'report.json', report)
    if result.returncode != 0 or state not in expected:
        raise RuntimeError(f'{stage}/{action}: status={state!r}, exit={result.returncode}, problemas={content.get("problems")}')
    return content


def start_recorder(root: Path, scripts: Path):
    rec_jsonl = root / 'odometria.jsonl'
    rec_report = root / 'odometria_report.json'
    cmd = [sys.executable, str(scripts / 'd9915_odom_recorder.py'),
           '--output-jsonl', str(rec_jsonl), '--report', str(rec_report)]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, errors='replace')
    time.sleep(1.0)
    return proc, rec_jsonl, rec_report


def stop_recorder(proc, root: Path):
    if proc is None:
        return
    try:
        proc.send_signal(signal.SIGINT)
        out, _ = proc.communicate(timeout=15)
    except subprocess.TimeoutExpired:
        proc.kill()
        out, _ = proc.communicate(timeout=5)
    with (root / 'pipeline.log').open('a', encoding='utf-8') as log:
        log.write('\n# RECORDER STDOUT\n')
        log.write((out or '') + '\n')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--execute', action='store_true')
    ap.add_argument('--confirm-runtime-v2', action='store_true')
    ap.add_argument('--output-dir', default='scripts/resultados/d9915_mision_001')
    ap.add_argument('--reference-dir', default='scripts/d996_reference')
    ap.add_argument('--scripts-dir', default='scripts')
    ap.add_argument('--mission-label', default='seed_100_run_007')
    args = ap.parse_args()

    root = Path(args.output_dir).resolve()
    if root.exists():
        ap.error(f'La carpeta de salida ya existe: {root}. Usa un nombre nuevo.')
    root.mkdir(parents=True)
    scripts = Path(args.scripts_dir).resolve()
    aligner = scripts / 'd997_initial_alignment_path.py'
    coordinator = scripts / 'd9913_stage_coordinator.py'
    plotter = scripts / 'd9915_plot_odom_vs_theory.py'
    report = {
        'schema': 'swarm.d9915.full_mission_telemetry.v1',
        'status': 'INITIALIZING',
        'execution_requested': args.execute,
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'steps': [],
        'completed_stages': [],
        'stage_windows': [],
        'problems': [],
    }
    write_json(root / 'report.json', report)

    recorder = None
    try:
        if args.execute and not args.confirm_runtime_v2:
            raise RuntimeError('REQUIRE_EXPLICIT_RUNTIME_V2_CONFIRMATION')
        if not aligner.is_file() or not coordinator.is_file() or not plotter.is_file():
            raise RuntimeError('Faltan scripts requeridos en scripts/')

        references = {}
        ref_dir = Path(args.reference_dir).resolve()
        for stage in STAGES:
            src = ref_dir / f'{stage}_d994_NON_EXECUTABLE.npz'
            if not src.is_file():
                raise RuntimeError(f'{stage}: trayectoria ausente: {src}')
            sha = hashlib.sha256(src.read_bytes()).hexdigest()
            if sha != SHAS[stage]:
                raise RuntimeError(f'{stage}: SHA256 incorrecto: {sha}')
            references[stage] = src
        report['verified_hashes'] = SHAS
        report['status'] = 'REFERENCES_VERIFIED'
        write_json(root / 'report.json', report)

        if not args.execute:
            call_step([str(aligner), '--trajectory', str(references['pickup1'])],
                      {'DRY_RUN_READY', 'ALREADY_ALIGNED'}, root, report, 'pickup1', 'alignment_dry')
            report['status'] = 'PASS_DRY_RUN_PICKUP1_AND_HASHES_NO_COMMANDS'
            write_json(root / 'report.json', report)
            return 0

        recorder, rec_jsonl, rec_report = start_recorder(root, scripts)
        report['odometria_jsonl'] = str(rec_jsonl)
        report['odometria_report'] = str(rec_report)
        write_json(root / 'report.json', report)

        for stage in STAGES:
            src = references[stage]
            call_step([str(aligner), '--trajectory', str(src), '--execute'],
                      {'ALIGNED', 'ALREADY_ALIGNED'}, root, report, stage, 'alignment')
            call_step([str(coordinator), '--stage', stage],
                      {'PASS_GUARDED_DRY_RUN_NO_COMMANDS'}, root, report, stage, 'navigation_dry')
            nav = call_step([str(coordinator), '--stage', stage, '--execute', '--confirm-runtime-v2'],
                            {'DONE_ALL_' + stage.upper()}, root, report, stage, 'navigation')
            start_time_s = float(nav.get('start_time_s', nav.get('clock_s', 0.0) + 10.0))
            max_dur = max(float(info['duration_s']) for info in nav['robots'].values())
            end_time_s = start_time_s + max_dur + 0.75
            report['stage_windows'].append({
                'stage': stage,
                'trajectory': str(src),
                'start_time_s': start_time_s,
                'end_time_s': end_time_s,
                'max_duration_s': max_dur,
            })
            report['completed_stages'].append(stage)
            write_json(root / 'report.json', report)

        stop_recorder(recorder, root)
        recorder = None
        stage_windows_path = root / 'stage_windows.json'
        write_json(stage_windows_path, {'schema': 'swarm.d9915.stage_windows.v1', 'stages': report['stage_windows']})
        graph_path = root / 'velocidad_aceleracion_real_vs_teorica.png'
        plot_cmd = [sys.executable, str(plotter), '--odom-jsonl', str(rec_jsonl),
                    '--stage-windows', str(stage_windows_path), '--output', str(graph_path),
                    '--mission-label', args.mission_label]
        with (root / 'pipeline.log').open('a', encoding='utf-8') as log:
            log.write('\n$ ' + ' '.join(plot_cmd) + '\n')
            res = subprocess.run(plot_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 text=True, errors='replace')
            log.write(res.stdout + '\nEXIT_CODE=' + str(res.returncode) + '\n')
        if res.returncode != 0:
            raise RuntimeError('No se pudo generar la gráfica comparativa; revisa pipeline.log')
        report['comparison_plot'] = str(graph_path)
        report['status'] = 'DONE_ALL_FOUR_STAGES_WITH_TELEMETRY'
        write_json(root / 'report.json', report)
        print(json.dumps({'status': report['status'], 'report': str(root / 'report.json'), 'plot': str(graph_path)}, ensure_ascii=False))
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        if recorder is not None:
            stop_recorder(recorder, root)
        report['status'] = 'REJECT_STOPPED'
        report['problems'].append(f'{type(exc).__name__}: {exc}')
        write_json(root / 'report.json', report)
        print('[D9915] DETENIDO:', exc, file=sys.stderr)
        print('Revisa:', root / 'report.json', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
