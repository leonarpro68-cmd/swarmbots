#!/usr/bin/env python3
"""D9.9.22: misión física + telemetría y comparación temporalmente coherente.

No publica cmd_vel, no modifica D8, D9.9 o el supervisor D9.9.21-A.
El supervisor físico es ejecutado como subprocess; el grabador funciona en paralelo.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

STAGES = ('pickup1', 'delivery1', 'pickup2', 'delivery2')
ROBOTS = ('summit0', 'summit1', 'summit2')


def atomic_json(path, data):
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    os.replace(tmp, path)


def close_process(p, timeout=12):
    if p is None or p.poll() is not None:
        return
    p.send_signal(signal.SIGINT)
    try:
        p.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        p.terminate()
        try:
            p.wait(timeout=4)
        except subprocess.TimeoutExpired:
            p.kill()
            p.wait(timeout=4)


def save_scaled_refs(navigation_root, reference_dir, target_dir, milp_stages=None):
    import numpy as np
    target_dir.mkdir(parents=True, exist_ok=True)
    windows = []
    diagnostics = []
    for stage in STAGES:
        nav_path = navigation_root / (stage + '_navigation.json')
        if not nav_path.is_file():
            continue  # Do not invent telemetry windows for unexecuted stages.
        nav = json.loads(nav_path.read_text(encoding='utf-8'))
        if nav.get('commands_sent') != 3 or 'start_time_s' not in nav:
            continue
        sc = nav.get('navigation_parameters', {}).get('time_scale')
        if sc is None:
            raise RuntimeError(f'{stage}: no se registró el factor temporal efectivo')
        factor = float(sc)
        if not math.isfinite(factor) or factor <= 0:
            raise RuntimeError(f'{stage}: factor temporal inválido {factor}')
        src = (Path(milp_stages[stage]['npz']) if milp_stages is not None
               else reference_dir / (stage + '_d994_NON_EXECUTABLE.npz'))
        dst = target_dir / (stage + '_d994_NON_EXECUTABLE.npz')
        with np.load(src, allow_pickle=False) as z:
            data = {key: z[key].copy() for key in z.files}
        successive = nav.get('navigation_parameters', {}).get('time_model') == 'common_collective_successive_v1'
        clock_path = Path(milp_stages[stage]['clock']) if successive and milp_stages is not None else None
        clock = json.loads(clock_path.read_text()) if clock_path is not None else None
        for i in range(3):
            key = f'R{i}_durations_s'
            if not successive:
                data[key] = data[key] / factor
                expected=float(data[key].sum())
            else:
                # En el NPZ V3 las duraciones son VIRTUALES y nunca se dividen
                # por un factor uniforme: el reloj sucesivo define la ley real.
                expected=float(clock['finish_times_wall_s'][f'summit{i}'])
            reported = float(nav['robots'][f'summit{i}']['duration_s'])
            if abs(expected-reported)>0.001:
                raise RuntimeError(f'{stage}: duración enviada no coincide con el reloj: '
                                   f'{expected} != {reported}')
        np.savez_compressed(dst, **data)
        start = float(nav['start_time_s'])
        duration = max(float(nav['robots'][r]['duration_s']) for r in ROBOTS)
        windows.append({'stage': stage, 'trajectory': str(dst.resolve()),
                        'start_time_s': start, 'end_time_s': start + duration + 0.75,
                        'max_duration_s': duration, 'time_scale': factor,
                        'time_model':'common_collective_successive_v1' if successive else 'uniform',
                        **({'clock_json':str(clock_path.resolve())} if successive else {})})
        diagnostics.append({'stage': stage, 'factor': None if successive else factor,
                            'time_model':'successive' if successive else 'uniform', 'status': nav['status']})
    return windows, diagnostics


def run_logged(cmd, log_path):
    with log_path.open('w', encoding='utf-8') as log:
        run = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, text=True)
    return run.returncode


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute', action='store_true')
    p.add_argument('--confirm-runtime-v2', action='store_true')
    p.add_argument('--confirm-cargo-inspected', action='store_true')
    p.add_argument('--auto-time-scale', action='store_true')
    p.add_argument('--time-scale', type=float, default=1.0)
    p.add_argument('--nav-speed-limit', type=float, default=1.2)
    p.add_argument('--nav-accel-limit', type=float, default=1.0)
    p.add_argument('--output-dir', required=True)
    p.add_argument('--scripts-dir', default='scripts')
    p.add_argument('--reference-dir', default='scripts/d996_reference')
    # D9927_MILP_SELECTOR_FAIL_CLOSED
    p.add_argument('--assignment-method', choices=('original','milp'), default='original')
    p.add_argument('--milp-library-dir', default='scripts/milp_library')
    p.add_argument('--milp-timed-dir', default=None)
    p.add_argument('--enable-successive-execution',action='store_true',help='Habilitación explícita de V3 tras reconstruir los seguidores')
    args = p.parse_args()
    if args.assignment_method=='milp' and (args.auto_time_scale or abs(args.time_scale-1.)>1e-9):
        p.error('MILP_V3: no usar --auto-time-scale ni --time-scale distinto de 1; reloj ya optimizado')
    if args.assignment_method=='milp' and args.execute and not args.enable_successive_execution:
        p.error('MILP_V3: falta --enable-successive-execution explícito')
    root = Path(args.output_dir).resolve()
    if root.exists():
        p.error('Carpeta de salida ya existe; usa una nueva por corrida')
    if args.execute and not (args.confirm_runtime_v2 and args.confirm_cargo_inspected):
        p.error('Faltan --confirm-runtime-v2 y --confirm-cargo-inspected')
    milp_pre=None
    if args.assignment_method=='milp':
        if args.milp_timed_dir is None:
            p.error('MILP requiere --milp-timed-dir')
        from d9927_milp_preflight import verify as verify_milp
        scenario=Path(args.reference_dir)/'scenario.json'
        try:
            milp_pre=verify_milp(scenario,Path(args.milp_library_dir),Path(args.milp_timed_dir))
        except Exception as exc:
            p.error(f'MILP_PREFLIGHT_REJECT: {exc}')
    root.mkdir(parents=True)
    scripts = Path(args.scripts_dir).resolve()
    refdir = Path(args.reference_dir).resolve()
    mission_dir = root / 'mision_fisica'
    report_file = root / 'report.json'
    report = {'schema': 'swarm.d9922.mission_telemetry.v1', 'status': 'INITIALIZING',
              'execute': args.execute, 'mission_dir': str(mission_dir),
              'nav_speed_limit_m_s': args.nav_speed_limit, 'nav_accel_limit_m_s2': args.nav_accel_limit,
              'auto_time_scale': args.auto_time_scale, 'problems': []}
    atomic_json(report_file, report)
    recorder = None
    mission = None
    try:
        base = [sys.executable, str(scripts/'d9920_full_physical_mission.py'),
                '--output-dir', str(mission_dir), '--reference-dir', str(refdir),
                '--nav-speed-limit', str(args.nav_speed_limit),
                '--nav-accel-limit', str(args.nav_accel_limit)]
        if args.assignment_method=='milp':
            base += ['--assignment-method','milp','--milp-library-dir',args.milp_library_dir,
                     '--milp-timed-dir',args.milp_timed_dir,
                     '--scenario',str(Path(args.reference_dir)/'scenario.json')]
        if args.assignment_method=='milp' and args.execute:
            base += ['--enable-successive-execution']
        if args.auto_time_scale:
            base += ['--auto-time-scale']
        else:
            base += ['--time-scale', str(args.time_scale)]
        if args.execute:
            base += ['--execute', '--confirm-runtime-v2', '--confirm-cargo-inspected']
        # Dry-run stays non-moving; no recorder is needed.
        if not args.execute:
            rc = run_logged(base, root/'mission.log')
            report['status'] = 'DRY_RUN_DONE' if rc == 0 else 'DRY_RUN_FAILED'
            report['mission_exit_code'] = rc
            atomic_json(report_file, report)
            return 0 if rc == 0 else 2
        rec_cmd = [sys.executable, str(scripts/'d9915_odom_recorder.py'),
                   '--output-jsonl', str(root/'odometria.jsonl'),
                   '--report', str(root/'odometria_report.json')]
        rec_log = (root/'recorder.log').open('w', encoding='utf-8')
        try:
            recorder = subprocess.Popen(rec_cmd, stdout=rec_log, stderr=subprocess.STDOUT,
                                        start_new_session=True)
        finally:
            rec_log.close()
        time.sleep(2.0)
        if recorder.poll() is not None:
            raise RuntimeError('No se pudo iniciar el grabador de odometría; revisar recorder.log')
        report['status'] = 'RUNNING'
        atomic_json(report_file, report)
        with (root/'mission.log').open('w', encoding='utf-8') as log:
            mission = subprocess.Popen(base, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            try:
                mission_rc = mission.wait()
            except KeyboardInterrupt:
                close_process(mission)
                raise
        report['mission_exit_code'] = mission_rc
        if not mission_dir.joinpath('report.json').is_file():
            raise RuntimeError('Misión no generó report.json; revisar mission.log')
        mission_report = json.loads((mission_dir/'report.json').read_text())
        report['mission_status'] = mission_report.get('status')
        # Stop recording BEFORE processing files, including on failed mission.
        close_process(recorder)
        recorder = None
        rec_report = root/'odometria_report.json'
        if not rec_report.exists():
            raise RuntimeError('Grabador no generó reporte al detenerse')
        rec_meta = json.loads(rec_report.read_text())
        if any(int(rec_meta.get('odom_count', {}).get(r, 0)) < 10 for r in ROBOTS):
            raise RuntimeError('No hay suficientes muestras de odometría para todos los Summit')
        windows, factors = save_scaled_refs(mission_dir, refdir, root/'referencias_reescaladas',
                                            milp_pre['stages'] if milp_pre else None)
        atomic_json(root/'stage_windows.json', {'schema':'swarm.d9922.stage_windows.v1',
                                                'stages': windows})
        report['time_factors'] = factors
        report['stage_count_with_telemetry'] = len(windows)
        if windows:
            v3_plot = bool(milp_pre and milp_pre['stages'].get('pickup1',{}).get('clock'))
            plot_cmd = [sys.executable, str(scripts/('d9931_plot_successive_telemetry.py' if v3_plot else 'd9915_plot_odom_vs_theory.py')),
                        '--odom-jsonl', str(root/'odometria.jsonl'),
                        '--stage-windows', str(root/'stage_windows.json'),
                        '--output', str(root/'velocidad_aceleracion_real_vs_teorica.png'),
                        '--nav-speed-limit', str(args.nav_speed_limit),
                        '--nav-accel-limit', str(args.nav_accel_limit)]
            # The original plotting script expects all 4 stages. Only render when complete.
            if len(windows) == 4:
                if run_logged(plot_cmd, root/'plot.log') != 0:
                    report['problems'].append('No se pudo generar gráfica; revisar plot.log')
                # El antiguo D9916 trata las duraciones NPZ como tiempo de
                # pared, pero en V3 son tiempo VIRTUAL. Usar el reloj exacto
                # para no reportar métricas falsas en misiones sucesivas.
                if v3_plot:
                    metric_cmd = [sys.executable, str(scripts/'d9932_v3_metrics.py'),
                                  '--odom-jsonl', str(root/'odometria.jsonl'),
                                  '--stage-windows', str(root/'stage_windows.json'),
                                  '--out-dir', str(root/'metricas')]
                else:
                    metric_cmd = [sys.executable, str(scripts/'d9916_validate_telemetry.py'),
                                  '--odom-jsonl', str(root/'odometria.jsonl'),
                                  '--stage-windows', str(root/'stage_windows.json'),
                                  '--reference-dir', str(root/'referencias_reescaladas'),
                                  '--out-dir', str(root/'metricas')]
                if run_logged(metric_cmd, root/'metrics.log') != 0:
                    report['problems'].append('No se pudieron generar métricas; revisar metrics.log')
        if len(windows) == 4:
            route_cmd = [sys.executable, str(scripts/'d9922_plot_mission_routes.py'),
                         '--stage-windows', str(root/'stage_windows.json'),
                         '--output', str(root/'mision_rutas.png')]
            scenario = refdir / 'scenario.json'
            if scenario.is_file():
                route_cmd += ['--scenario', str(scenario)]
            if run_logged(route_cmd, root/'routes.log') != 0:
                report['problems'].append('No se pudo generar mision_rutas.png; revisar routes.log')
        if mission_rc != 0 or report['mission_status'] != 'DONE_ALL_FOUR_PHYSICAL_STAGES_ACK':
            report['status'] = 'MISSION_FAILED_TELEMETRY_SAVED'
            report['problems'].append('La misión no terminó; revisar mision_fisica/report.json y mission.log')
            atomic_json(report_file, report)
            return 2
        if report['problems']:
            report['status'] = 'MISSION_DONE_ANALYSIS_PARTIAL'
            atomic_json(report_file, report)
            return 2
        report['status'] = 'DONE_MISSION_WITH_TELEMETRY_AND_METRICS'
        atomic_json(report_file, report)
        print(json.dumps({'status': report['status'], 'output_dir': str(root),
                          'time_factors': factors}, indent=2, ensure_ascii=False))
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        report['status'] = 'FAILED_TELEMETRY_MAY_BE_PARTIAL'
        report['problems'].append(f'{type(exc).__name__}: {exc}')
        atomic_json(report_file, report)
        print('[D9922] ERROR:', exc, file=sys.stderr)
        return 2
    finally:
        if mission is not None and mission.poll() is None:
            close_process(mission)
        if recorder is not None:
            close_process(recorder)

if __name__ == '__main__':
    raise SystemExit(main())
