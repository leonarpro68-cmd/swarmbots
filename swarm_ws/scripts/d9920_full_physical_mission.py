#!/usr/bin/env python3
"""D9.9.20: misión integrada con despeje CBS ejecutado entre oleadas.

Mantiene las barreras de seguridad de D9.9.17 y D9.9.18. No fabrica
certificados de despeje ni confirma entrega solo por recibir ACK.
La ruta CBS de despeje se genera desde odometría real y se sigue
con el path_follower existente. Wave2 exige posición despejada verificada.
"""
import argparse
import hashlib
import json
import os
import subprocess
import sys
import signal
import time
from datetime import datetime, timezone
from pathlib import Path

STAGES = ('pickup1', 'delivery1', 'pickup2', 'delivery2')
ROBOTS = ('summit0', 'summit1', 'summit2')
SHAS = {
    'pickup1':'3235223232eb1b47e36c46cc107086c933e5020917aa7b78758b9b183985e2dd',
    'delivery1':'70bb3550dfb3a937f002f5b8e83d621ef8b229180f6a0b8c6bdc626de19fb40b',
    'pickup2':'7f626f10002aeb40be8fd3684e8cc2ee49512ffbf01fdc969f489584c1f94e11',
    'delivery2':'ce13db58df80acdfc4bedebfc115c3a37a2bc5dad501727bfa9812ea2d1b2b60',
}


def write(path, data):
    tmp=path.with_suffix('.json.tmp')
    tmp.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n', encoding='utf8')
    os.replace(tmp,path)


def step(root, report, label, args, result_path, expected):
    command=[sys.executable, *map(str,args)]
    print('[D9920]',label,flush=True)
    with (root/'pipeline.log').open('a',encoding='utf8') as log:
        log.write('\n$ '+' '.join(command)+'\n');log.flush()
        output=subprocess.run(command,capture_output=True,text=True,errors='replace')
        log.write(output.stdout+'\n'+output.stderr+'\nEXIT='+str(output.returncode)+'\n')
    if output.stdout: print(output.stdout[-1200:],flush=True)
    try: data=json.loads(result_path.read_text(encoding='utf8'))
    except (ValueError,OSError) as exc: raise RuntimeError(f'{label}: sin informe válido; consultar pipeline.log: {exc}')
    status=data.get('status')
    report['steps'].append({'step':label,'status':status,'exit_code':output.returncode,'file':str(result_path)})
    write(root/'report.json',report)
    if output.returncode!=0 or status not in expected:
        raise RuntimeError(f'{label}: estado {status!r}, exit {output.returncode}; revisar {result_path}')
    return data



def parallel_pickup(root, report, stage, physical, nav_out, pickup_dir, extra_args=()):
    """Lanza el agarre de los tres robots a la vez; cancela los restantes si uno falla.

    Las tres rutas D9.9 llegan a su objetivo y se comprueban ANTES de iniciar
    cualquiera de las rutinas físicas. Cada rutina maneja un robot distinto.
    """
    jobs = {}
    try:
        for robot in ROBOTS:
            result_path = pickup_dir / f'{stage}_{robot}_execute.json'
            cmd = [sys.executable, str(physical), '--stage', stage, '--robot', robot,
                   '--navigation-report', str(nav_out), '--execute', '--output', str(result_path), *map(str,extra_args)]
            log_path = root / f'{stage}_{robot}_physical_pickup.log'
            log = log_path.open('w', encoding='utf8')
            try:
                process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                           start_new_session=True)
            except Exception:
                log.close()
                raise
            jobs[robot] = {'process': process, 'log': log, 'result': result_path,
                           'log_path': log_path}
        print(f'[D9920] {stage}: agarre paralelo de 3 Summit en ejecución', flush=True)
        pending = set(jobs)
        failure = None
        while pending:
            for robot in list(pending):
                code = jobs[robot]['process'].poll()
                if code is None:
                    continue
                pending.remove(robot)
                jobs[robot]['log'].flush()
                try:
                    data = json.loads(jobs[robot]['result'].read_text(encoding='utf8'))
                    status = data.get('status')
                except (ValueError, OSError) as exc:
                    status = f'INVALID_OR_MISSING_REPORT: {exc}'
                report['steps'].append({
                    'step': f'{stage}_{robot}_physical_pickup', 'status': status,
                    'exit_code': code, 'file': str(jobs[robot]['result']),
                    'log': str(jobs[robot]['log_path']), 'mode': 'parallel',
                })
                write(root / 'report.json', report)
                if code != 0 or status != 'PHYSICAL_ACK_REQUIRES_GAZEBO_INSPECTION':
                    failure = f'{robot}: exit={code} status={status}; revisar {jobs[robot]["log_path"]}'
                    break
            if failure:
                break
            if pending:
                time.sleep(0.1)
        if failure:
            # Las rutinas D8 son las dueñas de sus cmd_vel; sin evidencia de éxito
            # no se pasa a la etapa siguiente. Se interrumpe el grupo de procesos.
            for robot in pending:
                proc = jobs[robot]['process']
                if proc.poll() is None:
                    try:
                        os.killpg(proc.pid, signal.SIGINT)
                    except ProcessLookupError:
                        pass
            for robot in pending:
                proc = jobs[robot]['process']
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(proc.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    proc.wait(timeout=5)
            raise RuntimeError(f'{stage}: fallo en agarre paralelo: {failure}. '
                               'Verificar parada física; no reiniciar sobre mundo parcial.')
        print(f'[D9920] {stage}: los tres agarres reportaron ACK', flush=True)
    finally:
        for job in jobs.values():
            if job['process'].poll() is None:
                try:
                    os.killpg(job['process'].pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    job['process'].wait(timeout=5)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(job['process'].pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    job['process'].wait()
            job['log'].close()


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--time-scale',type=float,default=1.,help='Factor manual de navegación')
    ap.add_argument('--auto-time-scale',action='store_true',help='Calcula automáticamente el factor común seguro para cada etapa')
    ap.add_argument('--nav-speed-limit',type=float,default=1.2,help='m/s; también configurar hybrid_max_speed en launch')
    ap.add_argument('--nav-accel-limit',type=float,default=1.,help='m/s²; solo navegación Bézier')
    ap.add_argument('--execute',action='store_true',help='Permite navegación y agarres/descargas reales, sin maniobra de despeje')
    ap.add_argument('--confirm-runtime-v2',action='store_true')
    ap.add_argument('--confirm-cargo-inspected',action='store_true',help='Requerido por D9.9.18: autoriza ejecutar descargas tras comprobar la carga en Gazebo')
    ap.add_argument('--output-dir',default='scripts/resultados/d9920_mision_001')
    ap.add_argument('--scripts-dir',default='scripts')
    ap.add_argument('--reference-dir',default='scripts/d996_reference')
    ap.add_argument('--ledger',default='scripts/resultados/d8/seed_100_run_007/mission/mision.json')
    ap.add_argument('--scenario',default='scripts/d996_reference/scenario.json')
    ap.add_argument('--assignment-method',choices=('original','milp'),default='original')
    ap.add_argument('--milp-library-dir',default='scripts/milp_library')
    ap.add_argument('--milp-timed-dir',default=None)
    ap.add_argument('--enable-successive-execution',action='store_true',help='Solo tras comprobar runtime V3 con prueba offline y ROS reconstruido')
    a=ap.parse_args()
    root=Path(a.output_dir).resolve()
    if root.exists(): ap.error('No se sobrescriben corridas anteriores; elige otra carpeta')
    root.mkdir(parents=True)
    scripts=Path(a.scripts_dir).resolve()
    ref=Path(a.reference_dir).resolve()
    report={'schema':'swarm.d9920.physical_mission.v1','status':'INITIALIZING',
            'created_utc':datetime.now(timezone.utc).isoformat(),
            'execute':a.execute,'steps':[],'completed_stages':[],
            'physical_deliveries_verified':False,'problems':[]}
    write(root/'report.json',report)
    try:
        if not (0.2<=a.time_scale<=2.) or not (0.1<=a.nav_speed_limit<=2.) or not (0.1<=a.nav_accel_limit<=2.):
            raise ValueError('PARAMETROS_NAVEGACION_FUERA_DE_RANGO')
        report['navigation_parameters']={'time_scale':a.time_scale,'auto_time_scale':a.auto_time_scale,'nav_speed_limit_m_s':a.nav_speed_limit,'nav_accel_limit_m_s2':a.nav_accel_limit}
        write(root/'report.json',report)
        if a.execute and not a.confirm_cargo_inspected:
            raise RuntimeError('Falta --confirm-cargo-inspected requerido por módulo D9.9.18')
        if a.execute and not a.confirm_runtime_v2:
            raise RuntimeError('Falta --confirm-runtime-v2')
        files={'align':scripts/'d997_initial_alignment_path.py',
               'nav':scripts/'d9913_stage_coordinator.py',
               'physical':scripts/'d9917_physical_stage.py',
               'waves':scripts/'d9918_delivery_waves.py',
               'settle':scripts/'d9920s_wait_still.py'}
        for name,p in files.items():
            if not p.is_file(): raise RuntimeError(f'No existe módulo {name}: {p}')
        # Verificar identidad del escenario, ledger y trayectorias antes de mover.
        scenario=Path(a.scenario).resolve()
        milp_mode=a.assignment_method=='milp'
        if milp_mode and not a.milp_timed_dir: raise RuntimeError('MILP_TIMED_DIR_REQUIRED')
        if milp_mode and a.execute and not a.enable_successive_execution:
            raise RuntimeError('SUCCESSIVE_EXECUTION_REQUIRES_EXPLICIT_OPT_IN')
        milp_preflight=None
        milp_extra=[]
        if milp_mode:
            from d9927_milp_preflight import verify as verify_milp
            pre=verify_milp(scenario,Path(a.milp_library_dir),Path(a.milp_timed_dir))
            # El preflight se materializa para que las rutinas físicas usen la misma identidad.
            milp_preflight=root/'milp_preflight.json'
            write(milp_preflight,pre)
            milp_extra=['--milp-assignment',str(milp_preflight),'--scenario',str(scenario)]
            report['assignment_method']='milp'
            report['milp_assignment']=pre['assignment']
            report['milp_scenario_sha256']=pre['scenario_sha256']
        ledger=Path(a.ledger).resolve()
        from pathlib import Path as _P
        scenario_hash=hashlib.sha256(scenario.read_bytes()).hexdigest()
        if not milp_mode:
            metadata=json.loads(ledger.read_text())
            if metadata.get('scenario_sha256')!=scenario_hash:
                raise RuntimeError('Escenario distinto al ledger histórico')
        refs={}
        ref_shas={}
        for stage in STAGES:
            if milp_mode:
                p=Path(pre['stages'][stage]['npz'])
                digest=pre['stages'][stage]['sha256']
            else:
                p=ref/(stage+'_d994_NON_EXECUTABLE.npz')
                digest=SHAS[stage]
            if hashlib.sha256(p.read_bytes()).hexdigest()!=digest:
                raise RuntimeError(f'{stage}: NPZ no coincide con referencia de asignación')
            refs[stage]=p
            ref_shas[stage]=digest
        report['status']='PREFLIGHT_REFERENCES_VALID'
        write(root/'report.json',report)
        if not a.execute:
            path=root/'pickup1_alignment_dry.json'
            step(root,report,'pickup1_alignment_dry',
                 [files['align'],'--trajectory',refs['pickup1'],'--output',path],path,
                 {'DRY_RUN_READY','ALREADY_ALIGNED'})
            report['status']='DRY_RUN_REFERENCES_AND_PICKUP1_OK_NO_COMMANDS'
            write(root/'report.json',report)
            return 0

        pickup_dir=root/'agarres'
        pickup_dir.mkdir()
        for stage in STAGES:
            src=refs[stage]
            if milp_mode:
                # D9931: ejecuta reloj ya calculado, SIN --auto-time-scale.
                nav_time_mode=['--successive-clock',pre['stages'][stage]['clock']]
                if a.auto_time_scale or abs(a.time_scale-1.)>1e-9:
                    raise RuntimeError('DOUBLE_TIME_SCALING_FORBIDDEN_IN_SUCCESSIVE_MODE')
            else:
                nav_time_mode=['--auto-time-scale'] if a.auto_time_scale else ['--time-scale',str(a.time_scale)]
            align_path=root/(stage+'_alignment.json')
            step(root,report,stage+'_alignment',
                 [files['align'],'--trajectory',src,'--execute','--output',align_path],align_path,
                 {'ALIGNED','ALREADY_ALIGNED'})
            nav_dry=root/(stage+'_navigation_dry.json')
            step(root,report,stage+'_navigation_dry',
                 [files['nav'],'--stage',stage,'--trajectory',src,'--expected-sha256',ref_shas[stage],*nav_time_mode,'--nav-speed-limit',str(a.nav_speed_limit),'--nav-accel-limit',str(a.nav_accel_limit),'--output',nav_dry],nav_dry,
                 {'PASS_GUARDED_DRY_RUN_NO_COMMANDS'})
            nav_out=root/(stage+'_navigation.json')
            step(root,report,stage+'_navigation',
                 [files['nav'],'--stage',stage,'--trajectory',src,'--expected-sha256',ref_shas[stage],*nav_time_mode,'--nav-speed-limit',str(a.nav_speed_limit),'--nav-accel-limit',str(a.nav_accel_limit),'--execute','--confirm-runtime-v2','--output',nav_out],nav_out,
                 {'DONE_ALL_'+stage.upper()})
            if stage.startswith('pickup'):
                # Recogida simultánea: los tres robots ya han terminado su
                # navegación D9.9 y ahora ejecutan sus rutinas físicas en paralelo.
                parallel_pickup(root,report,stage,files['physical'],nav_out,pickup_dir,
                                [*milp_extra,'--milp-trajectory',str(src),'--milp-expected-sha256',ref_shas[stage]] if milp_mode else [])
            else:
                plan=root/(stage+'_wave_plan.json')
                step(root,report,stage+'_wave_plan',
                     [files['waves'],'--stage',stage,'--navigation-report',nav_out,
                      '--pickup-evidence-dir',pickup_dir,*milp_extra,'--out',plan],plan,
                     {'WAVE_PLAN_READY_NO_COMMANDS'})
                if not a.confirm_cargo_inspected:
                    report['status']='PAUSED_CARGO_INSPECTION_REQUIRED'
                    report['pending_stage']=stage
                    report['pending_wave']=1
                    report['problems'].append('No se ha acreditado el joint de cada carga en Gazebo; ACK no basta')
                    write(root/'report.json',report)
                    print('[D9920] PARADA SEGURA: falta inspección física de carga')
                    return 3
                wave1=root/(stage+'_wave1.json')
                step(root,report,stage+'_wave1',
                     [files['waves'],'--stage',stage,'--navigation-report',nav_out,
                      '--pickup-evidence-dir',pickup_dir,'--execute-wave','1',
                      '--confirm-cargo-inspected',*milp_extra,'--out',wave1],wave1,
                     {'WAVE_COMPLETED_ACK_REQUIRES_GAZEBO_INSPECTION'})
                # Segunda oleada solo si existen robots pendientes.
                wave_count=len(json.loads(plan.read_text())['waves'])
                if wave_count==1 and milp_mode:
                    report['completed_stages'].append(stage)
                    write(root/'report.json',report)
                    continue
                # NO existe despeje: cada Summit vuelve al nodo seguro tras
                # entregar. Esperar reposo de los tres antes de habilitar wave2.
                rest=root/(stage+'_between_waves_settle.json')
                step(root,report,stage+'_between_waves_settle',
                     [files['settle'],'--output',rest],rest,
                     {'ROBOTS_SETTLED_VERIFIED'})
                wave2=root/(stage+'_wave2.json')
                step(root,report,stage+'_wave2',
                     [files['waves'],'--stage',stage,'--navigation-report',nav_out,
                      '--pickup-evidence-dir',pickup_dir,'--execute-wave','2',
                      '--confirm-cargo-inspected','--prior-wave-report',wave1,
                      *milp_extra,'--out',wave2],wave2,
                     {'WAVE_COMPLETED_ACK_REQUIRES_GAZEBO_INSPECTION'})
            report['completed_stages'].append(stage)
            write(root/'report.json',report)

            # Después del segundo agarre físico, esperar reposo
            # antes de verificar la alineación de delivery2.
            if stage == 'pickup2':
                settle_out = root / 'pickup2_before_delivery2_settle.json'

                step(
                    root,
                    report,
                    'pickup2_before_delivery2_settle',
                    [
                        files['settle'],
                        '--settle-s', '1.5',
                        '--timeout-s', '20.0',
                        '--speed-limit', '0.025',
                        '--output', settle_out,
                    ],
                    settle_out,
                    {'ROBOTS_SETTLED_VERIFIED'}
                )
        report['status']='DONE_ALL_FOUR_PHYSICAL_STAGES_ACK'
        report['physical_deliveries_verified']=False
        write(root/'report.json',report)
        return 0
    except (Exception,KeyboardInterrupt) as exc:
        report['status']='REJECT_STOPPED'
        report['problems'].append(f'{type(exc).__name__}: {exc}')
        write(root/'report.json',report)
        print('[D9920] DETENIDO:',exc,file=sys.stderr)
        return 2

if __name__=='__main__':raise SystemExit(main())
