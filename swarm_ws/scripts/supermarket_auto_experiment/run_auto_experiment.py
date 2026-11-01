#!/usr/bin/env python3
"""Orquestador experimental FIXED/MILP. No arranca Gazebo ni el soporte ROS2."""
import argparse, hashlib, json, os, pathlib, subprocess, sys, time, signal, math
from collections import Counter
STAGES=('pickup1','delivery1','pickup2','delivery2')
ROBOTS=('summit0','summit1','summit2')

def sha(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()

def read(path):return json.loads(path.read_text())

def check_contract(prepared, mode):
    d=read(prepared/'phase5_static_preflight.json')
    if d.get('status')!='PASS_STATIC_PRECHECK_NOT_EXECUTABLE' or d.get('errors'):raise RuntimeError('El preflight fase5 no pasó')
    tasks={}
    for t in d.get('d8_tasks',[]):tasks[(t['stage'],t['robot'])]=t
    for stage in STAGES:
        nav=d['stages'][stage];p=pathlib.Path(nav['trajectory']);c=pathlib.Path(nav['successive_clock'])
        if not p.is_file() or sha(p)!=nav['sha256']:raise RuntimeError(f'{stage}: NPZ faltante o SHA distinto')
        if not c.is_file() or read(c).get('input_sha256')!=nav['sha256']:raise RuntimeError(f'{stage}: reloj no coincide')
        for r in ROBOTS:
            x=tasks.get((stage,r))
            if not x or not x.get('d8_distance_gate') or not x['local_corridor_screen'].get('straight_line_geometric_screen_pass'):
                raise RuntimeError(f'{stage}/{r}: falta autorización geométrica nominal')
    scenario=prepared/f'scenario_for_original_D8_seed100_{mode.upper()}.json'
    if not scenario.exists():raise RuntimeError('Falta escenario D8')
    scene=read(scenario)
    items={x['name']:x for x in scene.get('elements',[])}
    for stage in STAGES:
        fname=prepared/f'assignment_{stage}_{mode.upper()}.json'
        if not fname.is_file():raise RuntimeError(f'{stage}: falta asignación MILP')
        ass=read(fname)
        for robot in ROBOTS:
            slot=ass['robots'][robot]
            task=tasks[(stage,robot)]
            trash=slot['trash']
            deposit=slot['target']
            if trash not in items or items[trash].get('type')!='trash':
                raise RuntimeError(f'{stage}/{robot}: basurita inválida: {trash}')
            if items[trash].get('color')!=slot['color']:
                raise RuntimeError(f'{stage}/{robot}: color inconsistente')
            if deposit!=task['target'] or slot['color']!=task['color']:
                raise RuntimeError(f'{stage}/{robot}: preflight y asignación inconsistentes')
            if stage.startswith('delivery'):
                if deposit not in items or items[deposit].get('type')!='deposit' or deposit!='deposit_'+slot['color']:
                    raise RuntimeError(f'{stage}/{robot}: depósito inválido: {deposit}')
            elif deposit!=trash:
                raise RuntimeError(f'{stage}/{robot}: objetivo de pickup inválido')
    return d, tasks, scenario

def build_nav(stage,nav,run):
    base=[sys.executable,'scripts/d9913_stage_coordinator.py','--stage',stage,'--trajectory',nav['trajectory'],
          '--successive-clock',nav['successive_clock'],'--expected-sha256',nav['sha256'],
          '--nav-speed-limit','2.0','--nav-accel-limit','2.0','--observe-seconds','8','--start-delay','15']
    return [
     ('align_dry',[sys.executable,'scripts/d997_initial_alignment_path.py','--trajectory',nav['trajectory'],'--output',str(run/f'{stage}_alignment_dry.json')]),
     ('align',[sys.executable,'scripts/d997_initial_alignment_path.py','--trajectory',nav['trajectory'],'--execute','--output',str(run/f'{stage}_alignment_real.json')]),
     ('nav_dry',base+['--output',str(run/f'{stage}_nav_dry.json')]),
     ('nav',base+['--execute','--confirm-runtime-v2','--output',str(run/f'{stage}_navigation.json')])]

def physical(stage, robot, task, scenario, execute, prepared, mode):
    """Build D8 command from typed stage assignment, never from deposit name as trash."""
    assignment = read(prepared / f'assignment_{stage}_{mode.upper()}.json')
    slot = assignment['robots'][robot]
    trash = slot['trash']
    color = slot['color']
    if not isinstance(trash, str) or not trash.startswith('trash_'):
        raise RuntimeError(f'{stage}/{robot}: nombre de basurita inválido: {trash!r}')
    if color != task['color'] or slot['target'] != task['target']:
        raise RuntimeError(f'{stage}/{robot}: asignación D8 distinta del preflight')
    if stage.startswith('pickup'):
        if slot['target'] != trash:
            raise RuntimeError(f'{stage}/{robot}: target de recogida no coincide con basurita')
        cmd = [sys.executable,'scripts/prueba_alinear_agarrar_cbs.py',
               '--scenario',str(scenario),'--robot',robot,'--trash',trash,
               '--max-distance','2.2']
        if not execute:cmd+=['--dry-run']
    else:
        deposit = slot['target']
        if deposit != 'deposit_'+color:
            raise RuntimeError(f'{stage}/{robot}: depósito distinto al color de la carga')
        cmd = [sys.executable,'scripts/prueba_entrega_local_cbs.py',
               '--scenario',str(scenario),'--robot',robot,'--trash',trash,
               '--deposit',deposit,'--max-start-distance','1.8']
        if execute:cmd+=['--execute']
    return cmd

def make_waves(stage, prepared, mode, contract):
    """D9.9.18 principle: no duplicate deposit in a wave; screen local straight-line approach.

    This is conservative nominal geometry, not a live Gazebo safety certificate.
    """
    import numpy as np
    assignment=read(prepared/f'assignment_{stage}_{mode.upper()}.json')['robots']
    scenario=read(prepared/f'scenario_for_original_D8_seed100_{mode.upper()}.json')
    items={e['name']:e for e in scenario['elements']}
    with np.load(contract['stages'][stage]['trajectory'],allow_pickle=False) as z:
        starts={r:tuple(map(float,z[f'R{i}_controls'][-1,-1])) for i,r in enumerate(ROBOTS)}
    counts=Counter(assignment[r]['target'] for r in ROBOTS)
    pending=sorted(ROBOTS,key=lambda r:(counts[assignment[r]['target']],r))
    waves=[]
    while pending:
        used=set();group=[]
        for r in pending:
            dep=assignment[r]['target']
            if dep not in used:used.add(dep);group.append(r)
        waves.append(group);pending=[r for r in pending if r not in group]
    def distance_point_segment(p,a,b):
        vx=b[0]-a[0];vy=b[1]-a[1];q=vx*vx+vy*vy
        t=max(0.,min(1.,((p[0]-a[0])*vx+(p[1]-a[1])*vy)/q)) if q>1e-12 else 0.
        return math.hypot(p[0]-a[0]-t*vx,p[1]-a[1]-t*vy)
    def segments_distance(a,b,c,d):
        # Robust enough to reject crossing segments and sub-clearance pairs.
        def cross(u,v,w):return (v[0]-u[0])*(w[1]-u[1])-(v[1]-u[1])*(w[0]-u[0])
        if cross(a,b,c)*cross(a,b,d)<0 and cross(c,d,a)*cross(c,d,b)<0:return 0.
        return min(distance_point_segment(x,y,z) for x,y,z in ((a,c,d),(b,c,d),(c,a,b),(d,a,b)))
    paths={}
    for r in ROBOTS:
        a=starts[r]; b=items[assignment[r]['target']]['xy_m'];d=math.dist(a,b)
        if not 0.90<=d<=1.80:raise RuntimeError(f'{stage}/{r}: deposito a {d:.3f}m, fuera del rango D8')
        end=tuple(float(b[i])+(a[i]-float(b[i]))*.80/d for i in range(2))
        paths[r]=(a,end)
    for j,group in enumerate(waves):
        for ii,r in enumerate(group):
            for other in group[ii+1:]:
                if segments_distance(*paths[r],*paths[other])<1.25:
                    raise RuntimeError(f'{stage}: wave{j+1} corredor nominal {r}/{other} <1.25m')
            for other in ROBOTS:
                if other!=r and other not in group and distance_point_segment(starts[other],*paths[r])<1.25:
                    raise RuntimeError(f'{stage}: wave{j+1} pasa cerca de robot estacionado {other}')
    return waves


def concurrent_steps(stage, robots, build, run, log, save, mode, expected):
    """D9.9.20 pattern: three distinct D8 owners, simultaneous launch, fail-stop, no retry."""
    jobs={};failure=None
    try:
        for robot in robots:
            label=f'{stage}_{robot}_real';cmd=build(robot)
            rec={'name':label,'cmd':cmd,'started_wall_s':time.time(),'execution':'parallel'}
            log.append(rec);save()
            if mode=='plan':rec['result']='PLAN_ONLY';continue
            handle=(run/(label+'.log')).open('w')
            try:proc=subprocess.Popen(cmd,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
            except Exception:handle.close();raise
            jobs[robot]=(proc,handle,rec)
        if mode=='plan':save();return
        pending=set(jobs)
        while pending:
            for r in tuple(pending):
                proc,handle,rec=jobs[r];code=proc.poll()
                if code is None:continue
                pending.remove(r);handle.flush();rec.update(returncode=code,ended_wall_s=time.time())
                output=(run/(f'{stage}_{r}_real.log')).read_text(errors='replace')
                if code or expected not in output:
                    rec['result']='STOP_FAILED';failure=f'{stage}/{r}: D8 failed, inspect log';break
                rec['result']='PASS';save()
            if failure:break
            if pending:time.sleep(.1)
        if failure:raise RuntimeError(failure)
    finally:
        for r,(proc,handle,rec) in jobs.items():
            if proc.poll() is None:
                try:os.killpg(proc.pid,signal.SIGINT)
                except ProcessLookupError:pass
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    try:os.killpg(proc.pid,signal.SIGTERM)
                    except ProcessLookupError:pass
                    proc.wait(timeout=5)
            handle.close()
        save()

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--prepared',type=pathlib.Path,required=True)
    ap.add_argument('--run-dir',type=pathlib.Path,required=True)
    ap.add_argument('--mode',choices=['plan','execute'],default='plan')
    ap.add_argument('--confirm-new-session',action='store_true')
    ap.add_argument('--confirm-supervision',action='store_true')
    ap.add_argument('--assignment-mode',choices=['milp','fixed'],default='milp')
    ap.add_argument('--authorize-noninteractive',action='store_true',help='Autorización explícita desde launch unificado; NO certifica agarre físico')
    ap.add_argument('--unattended',action='store_true',help='Sin pausas humanas, NO certifica agarre/liberación física')
    ap.add_argument('--no-plots',action='store_true',help='Desactiva gráficos al terminar')
    ap.add_argument('--physical-policy',choices=['sequential','warehouse'],default='warehouse',help='warehouse: pickups paralelos y deliveries por oleadas con cribado de geometría')
    a=ap.parse_args();prepared=a.prepared.expanduser().resolve();run=a.run_dir.expanduser().resolve()
    contract,tasks,scenario=check_contract(prepared,a.assignment_mode)
    if run.exists() and any(run.iterdir()):raise SystemExit('ERROR: directorio de ensayo no vacío; no repetir ni sobreescribir')
    if a.mode=='execute' and not (a.confirm_new_session and a.confirm_supervision):raise SystemExit('Faltan confirmaciones explícitas')
    if a.mode=='execute' and not pathlib.Path('scripts/supermarket_experiment/record_odom.py').exists():raise SystemExit('Falta fase6 de evaluación')
    if a.mode=='execute' and not pathlib.Path('scripts/supermarket_experiment/mark_event.py').exists():raise SystemExit('Falta registrador de eventos')
    if a.mode=='execute' and not pathlib.Path('scripts/supermarket_support/check_support_graph.py').exists():raise SystemExit('Falta soporte ROS2')
    if a.mode=='execute':
        if not a.authorize_noninteractive:
            if not sys.stdin.isatty():raise SystemExit('Se requiere terminal interactiva para habilitar ejecución')
            value=input(f'Escribe EJECUTAR para autorizar una misión {a.assignment_mode.upper()} COMPLETA sin reintentos: ')
            if value!='EJECUTAR':raise SystemExit('Cancelado')
    run.mkdir(parents=True, exist_ok=True)
    log=[];rec=None
    def save(): (run/'automation_report.json').write_text(json.dumps({'mode':a.mode,'assignment_mode':a.assignment_mode,'events':log},ensure_ascii=False,indent=2)+'\n')
    def step(name,cmd,required_text=None,report_status=None):
        print('\n>>>',name,'\n', ' '.join(map(str,cmd)),flush=True)
        record={'name':name,'cmd':cmd,'started_wall_s':time.time()};log.append(record)
        if a.mode=='plan': record['result']='PLAN_ONLY';save();return
        with (run/(name+'.log')).open('w') as f:
            proc=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,check=False)
            f.write(proc.stdout)
        print(proc.stdout[-1200:],flush=True)
        record.update(returncode=proc.returncode,ended_wall_s=time.time())
        if proc.returncode or (required_text and required_text not in proc.stdout):
            record['result']='STOP_FAILED';save();raise RuntimeError(f'{name}: salida no válida; ver {name}.log')
        if report_status:
            r=read(pathlib.Path(cmd[-1])) if cmd[-2]=='--output' else None
            if r is None or r.get('status')!=report_status or r.get('problems'):
                record['result']='STOP_REPORT';save();raise RuntimeError(f'{name}: status inesperado')
        record['result']='PASS';save()
    def mark(evt):
        step('event_'+evt,[sys.executable,'scripts/supermarket_experiment/mark_event.py','--run-dir',str(run),'--event',evt])
    try:
        if a.mode=='execute':
            step('support_graph',[sys.executable,'scripts/supermarket_support/check_support_graph.py','--n-robots','3'],required_text='PASS_SUPPORT_GRAPH_NOT_EXECUTION_AUTHORIZATION')
            rec=subprocess.Popen([sys.executable,'scripts/supermarket_experiment/record_odom.py','--output',str(run/'odom.csv')],stdout=(run/'recorder.log').open('w'),stderr=subprocess.STDOUT)
            time.sleep(3)
            if rec.poll() is not None:raise RuntimeError('Recorder terminó prematuramente')
        mark('mission_start')
        for stage in STAGES:
            nav=contract['stages'][stage]
            for key,cmd in build_nav(stage,nav,run):
                if key=='nav':mark(stage+'_nav_start')
                status=('PASS_GUARDED_DRY_RUN_NO_COMMANDS' if key=='nav_dry' else 'DONE_ALL_'+stage.upper() if key=='nav' else None)
                step(stage+'_'+key,cmd,report_status=status)
                if key=='nav':mark(stage+'_nav_end')
            mark(stage+'_physical_start')
            if a.physical_policy=='warehouse':
                waves=([list(ROBOTS)] if stage.startswith('pickup') else make_waves(stage,prepared,a.assignment_mode,contract))
            else:waves=[[r] for r in ROBOTS]
            for wi,wave in enumerate(waves,1):
                print(f'{stage}: oleada {wi}/{len(waves)}, robots={wave}',flush=True)
                log.append({'name':stage+'_physical_wave','number':wi,'robots':wave,'policy':a.physical_policy});save()
                for robot in wave:
                    task=tasks[(stage,robot)]
                    step(stage+'_'+robot+'_dry',physical(stage,robot,task,scenario,False,prepared,a.assignment_mode),required_text='DRY RUN correcto' if stage.startswith('pickup') else 'DRY RUN:')
                concurrent_steps(stage,wave,
                   lambda robot:physical(stage,robot,tasks[(stage,robot)],scenario,True,prepared,a.assignment_mode),
                   run,log,save,a.mode,'REGRESO CONFIRMADO')
            if a.mode=='execute' and not a.unattended:
                ack=input(f'Comprueba físicamente las 3 cajas de {stage} en Gazebo. Escribe VERIFICADO para continuar: ')
                if ack!='VERIFICADO':raise RuntimeError('Parada por falta de confirmación física')
            mark(stage+'_physical_done')
        mark('mission_end')
        if a.mode=='execute':
            print('Misión automatizada completada según ACK ROS y verificaciones configuradas; evaluar estado físico.',flush=True)
    except Exception as ex:
        print('STOP:',ex,file=sys.stderr,flush=True)
        log.append({'result':'STOP','reason':str(ex),'time_wall_s':time.time()});save()
        raise SystemExit(2)
    finally:
        if rec is not None and rec.poll() is None:
            rec.send_signal(__import__('signal').SIGINT)
            try:rec.wait(timeout=8)
            except subprocess.TimeoutExpired:rec.terminate()
        save()
    if a.mode=='execute':
        subprocess.run([sys.executable,'scripts/supermarket_experiment/analyze_run.py','--run-dir',str(run)],check=False)
        if not a.no_plots:
            plot_script=pathlib.Path(__file__).with_name('plot_experiment.py')
            result=subprocess.run([sys.executable,str(plot_script),'--prepared',str(prepared),'--run-dir',str(run)],capture_output=True,text=True)
            (run/'plots.log').write_text(result.stdout+'\n'+result.stderr)
            print('PLOTS:', 'PASS' if result.returncode==0 else 'FAILED (review plots.log)',flush=True)
    print('REPORT:',run/'automation_report.json')
if __name__=='__main__':main()
