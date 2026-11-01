#!/usr/bin/env python3
"""Unique experiment IDs and runtime preflight; hands off to existing mission orchestrator.

Never retries robot manipulation. Doesn't certify physical attachment of objects.
"""
import argparse, datetime, json, os, pathlib, re, subprocess, sys, time, uuid


def unique_run_dir(root, mode, seed):
    root.mkdir(parents=True,exist_ok=True)
    # O_EXCL-style atomic mkdir; human-readable, collision-resistant.
    for _ in range(20):
        now=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        name=f'{mode}_seed{seed}_{now}_{uuid.uuid4().hex[:8]}'
        target=root/name
        try:target.mkdir(mode=0o775);return target
        except FileExistsError:continue
    raise RuntimeError('No se pudo crear un directorio único')


def validated_contract(prepared, seed, mode, workspace):
    if seed!=100:raise ValueError('Sólo semilla 100 certificada; NO usar trayectorias de seed100 con otra semilla')
    source=workspace/'config/supermarket/seed_100.json'
    scenario=json.loads(source.read_text())
    if scenario.get('seed')!=seed:raise ValueError('Semilla del escenario no coincide')
    pre=json.loads((prepared/'phase5_static_preflight.json').read_text())
    if pre.get('status')!='PASS_STATIC_PRECHECK_NOT_EXECUTABLE' or pre.get('errors'):
        raise ValueError('Preflight estático no pasó')
    for stage in ('pickup1','delivery1','pickup2','delivery2'):
        record=pre.get('stages',{}).get(stage)
        if not record:raise ValueError(f'Falta etapa {stage}')
        for field in ('trajectory','successive_clock'):
            if not pathlib.Path(record[field]).is_file():raise FileNotFoundError(record[field])
    return source


def check_live(ws, scenario, timeout):
    deadline=time.monotonic()+timeout
    observer=ws/'scripts/supermarket_preflight/ros_observer_readonly.py'
    support=ws/'scripts/supermarket_support/check_support_graph.py'
    if not observer.is_file() or not support.is_file():
        raise FileNotFoundError('Falta preflight ROS de sólo lectura')
    last=''
    while time.monotonic()<deadline:
        try:
            p=subprocess.run([sys.executable,str(support),'--n-robots','3'],capture_output=True,text=True,timeout=25,cwd=ws)
            if p.returncode==0 and 'PASS_SUPPORT_GRAPH_NOT_EXECUTION_AUTHORIZATION' in p.stdout:
                q=subprocess.run([sys.executable,str(observer),'--scenario',str(scenario),'--seconds','8',
                     '--output',str(ws/'scripts/resultados/supermarket_one_launch_preflight.json')],capture_output=True,text=True,timeout=45,cwd=ws)
                if q.returncode==0 and 'PASS_READ_ONLY_NOT_EXECUTION_AUTHORIZATION' in q.stdout:
                    print('ROS_PREFLIGHT_PASS: graph + initial world/odom positions',flush=True)
                    return
                last=f'observer: code={q.returncode}, output={q.stdout[-450:]} {q.stderr[-250:]}'
            else:last=f'graph: code={p.returncode}, output={p.stdout[-300:]} {p.stderr[-200:]}'
        except (subprocess.TimeoutExpired, OSError) as ex:last=str(ex)
        time.sleep(3)
    raise RuntimeError('ROS no está listo o la escena NO coincide con la semilla inicial: '+last)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--workspace',type=pathlib.Path,required=True)
    ap.add_argument('--prepared',type=pathlib.Path,required=True)
    ap.add_argument('--assignment-mode',choices=['fixed','milp'],required=True)
    ap.add_argument('--seed',type=int,required=True)
    ap.add_argument('--results-root',type=pathlib.Path,required=True)
    ap.add_argument('--readiness-seconds',type=int,default=180)
    a=ap.parse_args();ws=a.workspace.expanduser().resolve();prepared=a.prepared.expanduser().resolve()
    scenario=validated_contract(prepared,a.seed,a.assignment_mode,ws)
    # Preflight first; do NOT create a run if Gazebo isn't ready.
    check_live(ws,scenario,a.readiness_seconds)
    root=a.results_root.expanduser().resolve();run=unique_run_dir(root,a.assignment_mode,a.seed)
    (run/'experiment_metadata.json').write_text(json.dumps({
      'assignment_mode':a.assignment_mode,'seed':a.seed,'scenario':str(scenario),
      'prepared':str(prepared),'status':'STARTED_NOT_PHYSICAL_CERTIFICATE'
    },indent=2)+'\n')
    print('RESULTS_DIRECTORY:',run,flush=True)
    # Mission script explicitly accepts --authorize-noninteractive only for the one-launch
    # workflow; --unattended removes intermediate prompts but is not a physical certificate.
    cmd=[sys.executable,str(ws/'scripts/supermarket_auto_experiment/run_auto_experiment.py'),
        '--assignment-mode',a.assignment_mode,'--physical-policy','warehouse',
        '--prepared',str(prepared),'--run-dir',str(run),'--mode','execute',
        '--confirm-new-session','--confirm-supervision','--unattended','--authorize-noninteractive']
    # The mission expects the run dir to be empty. Metadata lives in a sibling directory.
    metadata=run/'experiment_metadata.json'; meta=root/(run.name+'.metadata.json')
    metadata.replace(meta) # keep uniquely reserved empty run directory
    try:
        with (root/(run.name+'.runner.log')).open('w') as f:
            proc=subprocess.run(cmd,cwd=ws,stdout=f,stderr=subprocess.STDOUT,check=False)
        print('MISSION_EXIT_CODE:',proc.returncode,flush=True)
        print('MISSION_LOG:',root/(run.name+'.runner.log'),flush=True)
        print('EXPERIMENT_RESULTS:',run,flush=True)
        if proc.returncode != 0:
            log_path = root/(run.name+'.runner.log')
            lines = log_path.read_text(errors='replace').splitlines()
            print('MISSION_ERROR_TAIL_BEGIN',flush=True)
            print('\n'.join(lines[-35:]),flush=True)
            print('MISSION_ERROR_TAIL_END',flush=True)
        return proc.returncode
    except Exception:
        raise

if __name__=='__main__':sys.exit(main())
