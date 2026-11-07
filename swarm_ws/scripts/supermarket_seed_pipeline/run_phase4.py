#!/usr/bin/env python3
"""Offline: MILP CBS multigoal -> Hybrid V2 SLSQP -> 5 refinements -> safety -> V3.

Never authorizes ROS/Gazebo execution, even on success. No overwrite of outputs.
The scripts copied in original_hybrid and original_v3 retain original solver logic.
"""
from __future__ import annotations
import argparse, hashlib, json, math, os, sys, time, traceback
from pathlib import Path
import numpy as np
from audit_phase4 import create_mission
from validar_corredores import run as audit_corridors
from stage_adapter import build_stage
from corridors_supermarket import Rectangle, corridors
from clearance_config import RECTANGLE_CENTER_CLEARANCE_M, WALL_CENTER_CLEARANCE_M, INTERROBOT_HALFSEPARATOR_M
from original_hybrid.slsqp import solve_all
from original_hybrid.refinement import cost
from refinement_supermarket import refine, certify as certify_halfspaces
from safety_supermarket import certify as certify_continuous, Settings
from original_v3.d9929_sucesivo_offline import optimize_stage
from original_v3.successive_clock import SuccessiveClock, command_envelopes

STAGES=('pickup1','delivery1','pickup2','delivery2')
ROOT=Path(__file__).resolve().parent

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))
def write(path,obj):
    path=Path(path)
    if path.exists():raise FileExistsError(f'Reporte existente, no sobrescribir: {path}')
    tmp=path.with_name(path.name+'.tmp')
    tmp.write_text(json.dumps(obj,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    os.replace(tmp,path)

def config(cbs_path,scenario_path,geometry_path):
    cbs=read(cbs_path);scene=read(scenario_path);geometry=read(geometry_path)
    if scene.get('seed') != cbs.get('seed'):raise ValueError('Semillas CBS y escenario incompatibles')
    mission=create_mission(cbs)
    audit=audit_corridors(scene,mission,geometry,margin=0.0)
    if not audit['pass_all']:
        detail=[f"{s['stage']}: {s.get('reason')}" for s in audit['stages'] if s['status']!='PASS']
        raise ValueError('BLOCKED_CORRIDOR_GEOMETRY: '+'; '.join(detail))
    return scene,geometry,mission,audit

def process_stage(stage,scene,geometry,mission,output,maxiter,refinements,samples,speed,accel,max_scale,transition):
    st=next(s for s in mission['best']['stages'] if s['stage']==stage)
    pts,circles,_=build_stage(scene,st,geometry)
    paths=st['paths_grid']
    rects=[Rectangle.from_json(r) for r in geometry['obstacles']]
    bounds=tuple(float(geometry['bounds'][a]) for a in ('xmin','xmax','ymin','ymax'))
    options=dict(rectangles=rects,bounds=bounds,
                 static_clearance_m=RECTANGLE_CENTER_CLEARANCE_M,
                 wall_clearance_m=WALL_CENTER_CLEARANCE_M,
                 robot_radius_m=INTERROBOT_HALFSEPARATOR_M,
                 circles_by_robot=circles)
    report={'stage':stage,'status':'STARTED','execution_authorized':False,
            'cbs_ticks':int(pts.shape[1]-1),'active_ticks':[len(p)-1 for p in paths],
            'targets':st['targets'],'selected_goals':st['selected_goals'],
            'refinements_requested':refinements,'source':'MILP CBS dynamic multigoal'}
    start=time.monotonic()
    try:
        hs=corridors(pts,(),**options)
        baseline,meta=solve_all(pts,paths,hs,require_success=False,maxiter=maxiter)
        report['initial_slsqp']=meta
        first=certify_halfspaces(baseline,hs,pts,paths)
        if not first['pass_certificate'] or any(not m['success'] or m['inequality_violation']>1e-6 or m['equality_violation']>1e-6 for m in meta):
            raise ValueError('REJECTED_INITIAL_SLSQP_OR_CERTIFICATE')
        report['initial_cost']=float(cost(baseline))
        controls,hist,last_hs=refine(pts,paths,baseline,iterations=refinements,samples=samples,
                                     maxiter=maxiter,return_last_halfspaces=True,**options)
        accepted=sum(bool(x.get('accepted')) for x in hist[1:]);report['refinement_history']=hist
        report['refinements_accepted']=accepted;report['final_cost']=float(cost(controls))
        if accepted != refinements:raise ValueError(f'REJECTED_INCOMPLETE_REFINEMENT_{accepted}_OF_{refinements}')
        if last_hs is None:
            raise ValueError('REJECTED_MISSING_LAST_ACCEPTED_CORRIDORS')
        # Reuse the actual final SLSQP constraints. The continuous certificate
        # also checks direct hull-to-obstacle and hull-to-robot distances.
        cert=certify_continuous(controls,pts,paths,rects,bounds,RECTANGLE_CENTER_CLEARANCE_M,circles,last_hs,
                wall_clearance_m=WALL_CENTER_CLEARANCE_M,robot_radius_m=INTERROBOT_HALFSEPARATOR_M,
                settings=Settings(margin_m=0.0,max_speed_m_s=speed,max_acceleration_m_s2=accel))
        report['continuous_certificate']=cert
        if not cert['pass_certificate']:raise ValueError('REJECTED_CONTINUOUS_CERTIFICATE: '+'; '.join(cert['errors']))
        K=pts.shape[1]-1
        vc=max(float(np.linalg.norm(5*np.diff(c,axis=1),axis=2).max()) for c in controls)
        ac=max(float(np.linalg.norm(20*np.diff(c,n=2,axis=1),axis=2).max()) for c in controls)
        dt=max(1.0,vc/speed,math.sqrt(ac/accel))*1.02
        timed=output/f'{stage}_MILP_TIMED_OFFLINE.npz'
        if timed.exists():raise FileExistsError(timed)
        payload={}
        for i in range(3):
            payload[f'R{i}_controls']=np.asarray(controls[i],dtype=float)
            payload[f'R{i}_durations_s']=np.full(K,dt,dtype=float)
        np.savez_compressed(timed,**payload)
        clock=optimize_stage(timed,max_speed=speed,max_accel=accel,max_scale=max_scale,transition_tau=transition)
        check=SuccessiveClock(clock['uniform_scale'],clock['ramps'],clock['duration_original_s'],clock['duration_successive_s'])
        independent={f'summit{i}':command_envelopes(controls[i],[dt]*K,check) for i in range(3)}
        if any(v>speed+1e-6 or a>accel+1e-6 for v,a in independent.values()):
            raise ValueError('REJECTED_INDEPENDENT_V3_BOUNDS')
        clock['geometry_source']='MILP CBS corrected + Hybrid V2 convex-hull certificate'
        clock['physical_D8_reach_verified']=False
        clock['execution_authorized']=False
        write(output/f'{stage}_MILP_V3_clock.json',clock)
        report['v3']={'clock_file':f'{stage}_MILP_V3_clock.json','timed_input':timed.name,
                      'dt_s':float(dt),'original_s':clock['duration_original_s'],
                      'successive_s':clock['duration_successive_s'],
                      'verified_envelopes':{r:{'speed_m_s':float(v),'acceleration_m_s2':float(a)} for r,(v,a) in independent.items()}}
        report['status']='PASS_OFFLINE_HYBRID_V2_V3_NOT_EXECUTABLE'
    except Exception as e:
        report['status']='REJECTED_NOT_EXECUTABLE'
        report['problem']=f'{type(e).__name__}: {e}'
        if 'timed' in locals() and timed.is_file():
            timed.unlink()  # Never leave a candidate timed artifact after failure.
        clockfile=output/f'{stage}_MILP_V3_clock.json'
        if clockfile.exists():clockfile.unlink()
    report['wall_s']=round(time.monotonic()-start,3)
    write(output/f'{stage}_report.json',report)
    print(f"{stage}: {report['status']} (wall {report['wall_s']:.2f}s)",flush=True)
    if 'problem' in report:print('  '+report['problem'],flush=True)
    return report

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cbs',type=Path,required=True)
    p.add_argument('--scenario',type=Path,required=True)
    p.add_argument('--geometry',type=Path,default=ROOT/'supermarket_obstaculos.json')
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--stage',choices=(*STAGES,'all'),default='all')
    p.add_argument('--maxiter',type=int,default=450)
    p.add_argument('--refinements',type=int,default=5)
    p.add_argument('--samples',type=int,default=32)
    p.add_argument('--max-speed',type=float,default=2.0)
    p.add_argument('--max-accel',type=float,default=2.0)
    p.add_argument('--max-scale',type=float,default=2.0)
    p.add_argument('--transition',type=float,default=2.0)
    a=p.parse_args()
    if a.output_dir.exists() and any(a.output_dir.iterdir()):p.error('Carpeta no vacía: no se sobrescriben reportes anteriores')
    if a.refinements != 5:p.error('Este paquete exige exactamente cinco refinamientos')
    if a.max_speed <= 0 or a.max_accel <= 0:p.error('Límites dinámicos deben ser positivos')
    scene,geo,mission,audit=config(a.cbs,a.scenario,a.geometry)
    a.output_dir.mkdir(parents=True,exist_ok=True)
    stages=STAGES if a.stage=='all' else (a.stage,)
    summary={'schema':'supermarket.milp.phase4.hybrid_v2_v3.offline.v2','status':'STARTED',
             'execution_authorized':False,'source_sha256':{'cbs':sha(a.cbs),'scenario':sha(a.scenario),'geometry':sha(a.geometry)},
             'corridors':{row['stage']:row['status'] for row in audit['stages']},'stages':{},
             'limitations':['NO se validó el alcance de maniobras físicas D8','No se verificó seguimiento ni dinámica real de Gazebo',
                            'No autoriza ejecución de robots ni sustituye la misión anterior']}
    for st in stages:
        report=process_stage(st,scene,geo,mission,a.output_dir,a.maxiter,a.refinements,a.samples,a.max_speed,a.max_accel,a.max_scale,a.transition)
        summary['stages'][st]={'status':report['status'],'problem':report.get('problem'),
                              'cbs_ticks':report['cbs_ticks'],'report':f'{st}_report.json',
                              'v3':report.get('v3')}
        write(a.output_dir/f'{st}_stage_done.json',{'stage':st,'status':report['status'],'execution_authorized':False})
    summary['status']='PASS_OFFLINE_FOUR_STAGES_NOT_EXECUTABLE' if len(stages)==4 and all(r['status'].startswith('PASS_') for r in summary['stages'].values()) else 'PARTIAL_OR_REJECTED_NOT_EXECUTABLE'
    write(a.output_dir/'summary.json',summary)
    print('FINAL:',summary['status'],'OUTPUT:',a.output_dir,flush=True)
    return 0 if all(r['status'].startswith('PASS_') for r in summary['stages'].values()) else 2
if __name__=='__main__':raise SystemExit(main())
