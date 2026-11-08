#!/usr/bin/env python3
"""Offline only: MILP assignment -> CBS multigoal -> Hybrid V2 -> V3 -> D8 static preflight.

Does NOT move robots, start Gazebo or authorize physical execution.
Generated artifacts are tied to scenario / assignment / geometry hashes and are NEVER overwritten.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from swarm_maps import MAPS, map_files, scenario_file, milp_results_dir, plans_dir, validate_map

STAGES=('pickup1','delivery1','pickup2','delivery2')
HERE=Path(__file__).resolve().parent

def read(path:Path):return json.loads(path.read_text(encoding='utf-8'))
def sha(path:Path):return hashlib.sha256(path.read_bytes()).hexdigest()
def write_new(path:Path, data):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x',encoding='utf-8') as f:json.dump(data,f,ensure_ascii=False,indent=2);f.write('\n')
def launch(*args):
    cmd=[str(x) for x in args]
    print('\n$ '+' '.join(cmd),flush=True)
    env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
    subprocess.run(cmd,cwd=HERE,env=env,check=True)
def compatible_manifest(stored, expected, map_name, mode):
    """No reescribe los certificados; admite el formato MILP/FIXED histórico."""
    if stored==expected:
        return True
    if map_name!='supermarket':
        return False
    legacy=dict(expected)
    legacy.pop('map',None)
    if mode=='milp':
        legacy.pop('assignment_mode',None)
    return stored==legacy


def expected_input(ws,seed,mode,map_name="supermarket"):
    sc=scenario_file(ws,map_name,seed)
    asn=(milp_results_dir(ws,map_name,seed)/'assignment_milp_one_color_per_round.json'
         if mode=='milp' else plans_dir(ws,map_name,seed)/'fixed'/'fixed_assignment.json')
    for p in (sc,asn):
        if not p.is_file():raise FileNotFoundError(f'Falta archivo offline: {p}. Corre run_offline_seed.py primero.')
    scene=read(sc);assignment=read(asn)
    if scene.get('seed')!=seed or assignment.get('seed')!=seed or scene.get('world')!=map_name or assignment.get('world')!=map_name:
        raise ValueError('Semilla/world de escenario y asignación no concuerdan')
    if assignment.get('execution_authorized') is not False:raise ValueError('Asignación offline incompatible')
    if mode=='milp':
        if assignment.get('status')!='OPTIMAL_ASTAR_PROXY_OFFLINE_NOT_EXECUTABLE' or assignment.get('independent_exhaustive_verification',{}).get('global_proxy_makespan_optimum_verified') is not True:
            raise ValueError('MILP no tiene óptimo verificado')
    elif assignment.get('status')!='FIXED_DETERMINISTIC_OFFLINE_NOT_EXECUTABLE':
        raise ValueError('FIXED carece de estado determinista esperado')
    return sc,asn

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--seed',type=int,required=True)
    ap.add_argument('--map',default='supermarket')
    ap.add_argument('--assignment-mode',choices=('fixed','milp'),default='milp')
    ap.add_argument('--workspace',type=Path,default=Path(__file__).resolve().parents[2])
    ap.add_argument('--maxiter',type=int,default=450)
    ap.add_argument('--max-ct',type=int,default=None,help='Límite CBS: 10000 en planes nuevos; conserva el valor del manifiesto en planes existentes')
    ap.add_argument('--max-cbs-time',type=int,default=110)
    ap.add_argument('--until',choices=('cbs','v3','d8'),default='d8')
    a=ap.parse_args()
    if a.seed<0:ap.error('semilla negativa')
    ws=a.workspace.expanduser().resolve()
    if a.assignment_mode=='fixed':
        sc=scenario_file(ws,a.map,a.seed)
        target=plans_dir(ws,a.map,a.seed)/'fixed'/'fixed_assignment.json'
        if not target.exists():
            if not sc.is_file():raise FileNotFoundError(sc)
            launch(sys.executable,HERE/'make_fixed_assignment.py','--scenario',sc,'--output',target)
    scenario,assignment=expected_input(ws,a.seed,a.assignment_mode,a.map)
    graph,geom=map_files(ws,a.map)
    validate_map(graph,geom)
    base=plans_dir(ws,a.map,a.seed)
    cbs=(base/'cbs_multigoal.json' if a.assignment_mode=='milp' else base/'fixed'/'cbs_multigoal.json')
    root=cbs.parent
    # No invalidar los planes previos: sus parámetros están ligados al SHA
    # del manifiesto. Un nuevo plan usa 10000, uno existente conserva el
    # presupuesto original salvo que se indique --max-ct explícitamente.
    previous_manifest=root/'inputs_manifest.json'
    if a.max_ct is None:
        a.max_ct=read(previous_manifest).get('max_ct',6000) if previous_manifest.is_file() else 10000
    phase4=root/'phase4'
    prepared=root/'prepared'
    source={'scenario_sha256':sha(scenario),'assignment_sha256':sha(assignment),
            'geometry_sha256':sha(geom),'graph_sha256':sha(graph),
            'seed':a.seed,'assignment_mode':a.assignment_mode,'max_ct':a.max_ct,'max_cbs_time':a.max_cbs_time,
            'maxiter':a.maxiter,'refinements':5,'max_speed':2.,'max_acceleration':2.}
    source['map']=a.map  # mismo contrato para todos los mapas nuevos
    manifest=root/'inputs_manifest.json'
    if manifest.exists():
        if not compatible_manifest(read(manifest),source,a.map,a.assignment_mode):
            raise ValueError('BLOCKED: el escenario, MILP, geometría o parámetros cambiaron. No reutilizar planes anteriores. Crear nuevo directorio de planes.')
    else:
        allowed={'fixed_assignment.json'} if a.assignment_mode=='fixed' else set()
        if root.exists() and any(x.name not in allowed for x in root.iterdir()):raise ValueError('BLOCKED: hay archivos sin manifiesto; no se reutilizan')
        write_new(manifest,source)
    if not cbs.is_file():
        launch(sys.executable,HERE/'plan_milp_multigoal.py','--assignment',assignment,'--scenario',scenario,'--output',cbs,'--max-time',a.max_cbs_time,'--max-ct',a.max_ct,'--assignment-mode',a.assignment_mode,'--graph',graph)
    else:
        x=read(cbs)
        if x.get('seed')!=a.seed or x.get('inputs_sha256')!={'scenario':sha(scenario),'assignment':sha(assignment)} or x.get('status')!='OFFLINE_CBS_ONLY_NOT_EXECUTABLE':
            raise ValueError('BLOCKED: CBS existente no coincide con entradas')
        print('REUTILIZANDO CBS ligado a los SHA originales',flush=True)
    if a.until=='cbs':print('COMPLETADO CBS DISCRETO (NO EJECUTABLE)',cbs);return 0
    audit=root/'corridors_audit.json'
    if not audit.exists():
        launch(sys.executable,HERE/'audit_phase4.py','--cbs',cbs,'--scenario',scenario,'--geometry',geom,'--output',audit)
    ar=read(audit)
    if ar.get('status')!='PASS_CORRIDOR_PREFLIGHT_ONLY' or ar.get('source_sha256')!={'cbs':sha(cbs),'scenario':sha(scenario),'geometry':sha(geom)}:
        raise ValueError('BLOCKED: geometría de corredores inválida o audit no coincide con entradas')
    for st in STAGES:
        folder=phase4/st
        report=folder/f'{st}_report.json'
        clock=folder/f'{st}_MILP_V3_clock.json'
        npz=folder/f'{st}_MILP_TIMED_OFFLINE.npz'
        if not report.exists():
            if folder.exists() and any(folder.iterdir()):
                raise ValueError(f'{st}: etapa parcial tras interrupción. Mover carpeta a un archivo de diagnóstico antes de recomputar; nunca se borra automáticamente: {folder}')
            launch(sys.executable,HERE/'run_phase4.py','--cbs',cbs,'--scenario',scenario,
                '--geometry',geom,'--output-dir',folder,'--stage',st,'--maxiter',a.maxiter,
                '--refinements','5','--max-speed','2.0','--max-accel','2.0')
        r=read(report)
        if r.get('status')!='PASS_OFFLINE_HYBRID_V2_V3_NOT_EXECUTABLE' or not npz.is_file() or not clock.is_file() or read(clock).get('input_sha256')!=sha(npz):
            raise RuntimeError(f'BLOCKED {st}: SLSQP/V3 sin certificado PASS. Ver {report}')
        print(f'CERTIFICADO_OFFLINE {st} OK',flush=True)
    launch(sys.executable,HERE/'verify_offline.py','--results',phase4)
    if a.until=='v3':print('COMPLETADO HYBRID_V2_V3 OFFLINE (NO EJECUTABLE)',phase4);return 0
    preflight=prepared/'phase5_static_preflight.json'
    if not preflight.is_file():
        if prepared.exists() and any(prepared.iterdir()):raise RuntimeError('Hay directorio preparado parcial. No se sobrescribe')
        launch(sys.executable,HERE/'prepare_phase5.py','--cbs',cbs,'--scenario',scenario,
            '--geometry',geom,'--results',phase4,'--output-dir',prepared,'--assignment-mode',a.assignment_mode)
    x=read(preflight)
    if x.get('status')!='PASS_STATIC_PRECHECK_NOT_EXECUTABLE' or x.get('errors') or x.get('sha256')!={'cbs':sha(cbs),'scenario':sha(scenario),'geometry':sha(geom)}:
        raise ValueError('BLOCKED: fase5 sin PASS o hash distinto')
    print('\nOFFLINE_FULL_STATIC_PASS_NOT_GAZEBO_CERTIFIED',flush=True)
    print('MAPA:',a.map,flush=True)
    print('SEMILLA:',a.seed,flush=True)
    print('PLAN_PREPARADO:',prepared,flush=True)
    print('NO AUTORIZA iniciar Gazebo: quedan preflight ROS en vivo y validación física D8.',flush=True)
    return 0

if __name__=='__main__':
    try:raise SystemExit(main())
    except (ValueError,RuntimeError,FileNotFoundError,subprocess.CalledProcessError) as exc:
        print('OFFLINE_STOP:',exc,file=sys.stderr,flush=True)
        raise SystemExit(2)
