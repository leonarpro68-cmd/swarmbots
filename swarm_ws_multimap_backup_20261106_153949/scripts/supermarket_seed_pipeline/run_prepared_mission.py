#!/usr/bin/env python3
"""Read-only preflight + hash checking + seeded supervised Gazebo mission.

The old validated seed100 controller and its mission script remain unchanged.
"""
import argparse
import datetime
import hashlib
import json
import pathlib
import subprocess
import sys
import uuid

STAGES=('pickup1','delivery1','pickup2','delivery2')
HERE=pathlib.Path(__file__).resolve().parent

def sha(p):return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(pathlib.Path(p).read_text(encoding='utf-8'))

def validate(ws,seed,mode="milp"):
    if seed==100 or seed<0:raise ValueError('Usar launch seed100 para semilla 100; nuevas semillas deben ser no negativas')
    scene=ws/'config/supermarket'/f'seed_{seed}.json'
    base=HERE/'plans'/f'seed_{seed}'
    root=base if mode=='milp' else base/'fixed'
    prepared=root/'prepared'
    manifest=root/'inputs_manifest.json'
    pre=prepared/'phase5_static_preflight.json'
    cbs=root/'cbs_multigoal.json'
    assign=(ws/'scripts/supermarket_milp/results'/f'seed_{seed}'/'assignment_milp_one_color_per_round.json' if mode=='milp' else root/'fixed_assignment.json')
    geometry=HERE/'supermarket_obstaculos.json'
    graph=HERE/'supermarket_grafo.json'
    for p in (scene,manifest,pre,cbs,assign,geometry,graph):
        if not p.is_file():raise FileNotFoundError(p)
    if read(scene).get('seed')!=seed or read(scene).get('world')!='supermarket':raise ValueError('Escenario semilla/world incompatible')
    m=read(manifest)
    for k,p in [('scenario_sha256',scene),('assignment_sha256',assign),('geometry_sha256',geometry),('graph_sha256',graph)]:
        if m.get(k)!=sha(p):raise ValueError(f'HASH MISMATCH: {k}')
    if m.get('seed')!=seed or (mode=='fixed' and m.get('assignment_mode')!='fixed'):
        raise ValueError('Manifiesto de otra semilla o modo')
    if mode=='milp' and m.get('assignment_mode','milp')!='milp':raise ValueError('Manifiesto MILP alterado')
    cr=read(cbs)
    if cr.get('assignment_mode','milp')!=mode:raise ValueError('CBS pertenece a otro modo')
    if cr.get('seed')!=seed or cr.get('status')!='OFFLINE_CBS_ONLY_NOT_EXECUTABLE' or cr.get('inputs_sha256')!={'scenario':sha(scene),'assignment':sha(assign)}:
        raise ValueError('CBS no corresponde a esta asignación y semilla')
    d=read(pre)
    if d.get('status')!='PASS_STATIC_PRECHECK_NOT_EXECUTABLE' or d.get('errors'):
        raise ValueError('La fase5 no pasó completamente')
    if d.get('sha256')!={'cbs':sha(cbs),'scenario':sha(scene),'geometry':sha(geometry)}:
        raise ValueError('Hashes de preflight fuera de sincronía')
    items=prepared/f'scenario_for_original_D8_seed{seed}_{mode.upper()}.json'
    if not items.is_file() or read(items).get('seed')!=seed:raise ValueError('Archivo D8 de semilla incorrecta')
    for stage in STAGES:
        nav=d.get('stages',{}).get(stage)
        if not nav:raise ValueError(f'Etapa {stage} no encontrada')
        traj=pathlib.Path(nav.get('trajectory','')).resolve()
        clock=pathlib.Path(nav.get('successive_clock','')).resolve()
        expected=(root/'phase4'/stage).resolve()
        if traj.parent!=expected or clock.parent!=expected:raise ValueError(f'{stage}: trayectoria fuera de carpeta de semilla')
        if not traj.is_file() or not clock.is_file() or sha(traj)!=nav['sha256'] or read(clock).get('input_sha256')!=nav['sha256']:
            raise ValueError(f'{stage}: NPZ/reloj no coincide con SHA')
        rr=read(expected/f'{stage}_report.json')
        if rr.get('status')!='PASS_OFFLINE_HYBRID_V2_V3_NOT_EXECUTABLE' or rr.get('refinements_accepted')!=5 or not rr.get('continuous_certificate',{}).get('pass_certificate'):
            raise ValueError(f'{stage}: certificado Hybrid V2 ausente')
        aa=read(prepared/f'assignment_{stage}_{mode.upper()}.json')
        if aa.get('seed')!=seed or aa.get('navigation_sha256')!=nav['sha256']:
            raise ValueError(f'{stage}: asignación física de otra semilla')
    return scene,prepared

def unique_dir(root,seed,mode="milp"):
    root.mkdir(parents=True,exist_ok=True)
    for _ in range(20):
        name='{}_seed{}_{}_{}'.format(mode,seed,datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ'),uuid.uuid4().hex[:8])
        p=root/name
        try:p.mkdir();return p
        except FileExistsError:continue
    raise RuntimeError('No se pudo reservar carpeta nueva')

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--seed',type=int,required=True)
    ap.add_argument('--assignment-mode',choices=('fixed','milp'),default='milp')
    ap.add_argument('--workspace',type=pathlib.Path,required=True)
    ap.add_argument('--results-root',type=pathlib.Path,required=True)
    ap.add_argument('--readiness-seconds',type=int,default=180)
    a=ap.parse_args()
    ws=a.workspace.expanduser().resolve()
    scene,prepared=validate(ws,a.seed,a.assignment_mode)
    sys.path.insert(0,str(ws/'scripts/supermarket_one_launch'))
    from run_unique_mission import check_live  # reuse working read-only ROS preflight
    check_live(ws,scene,a.readiness_seconds)
    root=a.results_root.expanduser().resolve()
    run=unique_dir(root,a.seed,a.assignment_mode)
    meta={'seed':a.seed,'mode':a.assignment_mode,'scenario':str(scene),'prepared':str(prepared),
          'scenario_sha256':sha(scene),'status':'STARTED_SUPERVISED_NOT_PHYSICAL_CERTIFICATE'}
    (root/(run.name+'.metadata.json')).write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n')
    print('RESULTS_DIRECTORY:',run,flush=True)
    exe=HERE/'run_auto_experiment_seed.py'
    if not exe.is_file():raise FileNotFoundError(exe)
    cmd=[sys.executable,str(exe),'--assignment-mode',a.assignment_mode,'--physical-policy','warehouse',
         '--prepared',str(prepared),'--run-dir',str(run),'--mode','execute',
         '--confirm-new-session','--confirm-supervision','--unattended','--authorize-noninteractive']
    log=root/(run.name+'.runner.log')
    with log.open('w') as f:proc=subprocess.run(cmd,cwd=ws,stdout=f,stderr=subprocess.STDOUT,check=False)
    print('MISSION_EXIT_CODE:',proc.returncode,flush=True)
    print('MISSION_LOG:',log,flush=True)
    print('EXPERIMENT_RESULTS:',run,flush=True)
    if proc.returncode:
        print('MISSION_ERROR_TAIL_BEGIN',flush=True)
        print('\n'.join(log.read_text(errors='replace').splitlines()[-35:]),flush=True)
        print('MISSION_ERROR_TAIL_END',flush=True)
    return proc.returncode

if __name__=='__main__':raise SystemExit(main())
