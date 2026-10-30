#!/usr/bin/env python3
"""D9.9.25: genera y verifica controles Bézier OFFLINE para la mejor asignación MILP.

No produce artefactos de ejecución D9.9.20 ni envía comandos ROS.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np

from d9923_milp_bezier_seed100 import fixed_pickup, fixed_delivery
from d9924_assignment_library import scenario_info, read_json
from swarm_behavior.swarm_planning.bridge import PlanningBridge
from swarm_behavior.swarm_planning.scenario import load
from swarm_behavior.swarm_planning.hybrid_v2.cbs_adapter import plan_v2

STAGES=('pickup1','delivery1','pickup2','delivery2')

def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scenario', type=Path, required=True)
    ap.add_argument('--library-dir',type=Path,default=Path('scripts/milp_library'))
    ap.add_argument('--output-dir',type=Path,required=True)
    ap.add_argument('--iterations',type=int,default=2)
    ap.add_argument('--maxiter',type=int,default=450)
    ap.add_argument('--margin',type=float,default=.05)
    ap.add_argument('--max-ct',type=int,default=20000)
    ap.add_argument('--max-time',type=int,default=65)
    args=ap.parse_args()
    seed,sha,_=scenario_info(args.scenario)
    best_path=args.library_dir/f'seed_{seed}'/'best_assignment.json'
    if not best_path.is_file(): ap.error(f'MILP_ASSIGNMENT_NOT_FOUND: {best_path}')
    saved=read_json(best_path)
    if saved.get('scenario_sha256')!=sha or saved.get('seed')!=seed or saved.get('status')!='CERTIFIED':
        ap.error('La biblioteca no corresponde al escenario o carece de certificación')
    params=saved.get('optimization_parameters',{})
    if (not math.isclose(float(params.get('max_speed_m_s',-1)),2.,abs_tol=1e-9)
        or not math.isclose(float(params.get('max_accel_m_s2',-1)),2.,abs_tol=1e-9)):
        ap.error('Esta biblioteca fue definida para límites de optimización 2.0/2.0')
    assignment=saved['assignment']
    if set(assignment)!={'summit0','summit1','summit2'} or any(len(assignment[f'summit{i}'])!=2 for i in range(3)):
        ap.error('Asignación inválida: se requieren 2 basuras por Summit')
    names=[n for i in range(3) for n in assignment[f'summit{i}']]
    scenario=load(args.scenario)
    bridge=PlanningBridge(scenario)
    if len(set(names))!=6 or set(names)!=set(bridge.pending):
        ap.error('Las seis basuras guardadas no coinciden con el escenario')
    starts=tuple(map(tuple,scenario.robots))
    args.output_dir.mkdir(parents=True,exist_ok=True)
    result={'schema':'swarm.d9925.milp_bezier_offline.v1','seed':seed,'scenario_sha256':sha,
            'assignment':assignment,'max_speed_m_s':2.,'max_accel_m_s2':2.,
            'execution_authorized':False,'format':'C2_RAW_CONTROLS_NOT_TIMED_EXECUTION_NPZ',
            'stages':{},'status':'RUNNING'}
    manifest=args.output_dir/'manifest.json'
    def write():
        manifest.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    write()
    try:
        for k in range(2):
            picked=tuple(assignment[f'summit{i}'][k] for i in range(3))
            for kind in ('pickup','delivery'):
                stage=f'{kind}{k+1}'
                plan=(fixed_pickup(bridge,starts,picked,args) if kind=='pickup' else
                      fixed_delivery(bridge,starts,picked,args))
                report,controls=plan_v2(plan,bridge.graph,stage=stage,margin=args.margin,
                             iterations=args.iterations,max_speed=2.,max_accel=2.,maxiter=args.maxiter)
                if not report.get('certified') or controls is None:
                    raise RuntimeError(f'{stage}: no certificado: {report.get("reason",report.get("status"))}')
                duration=float(report['duration_s']);dt=float(report['dt_s'])
                if not all(math.isfinite(v) and v>0 for v in (duration,dt)):
                    raise ValueError(f'{stage}: tiempo inválido')
                arrays={f'R{i}':np.asarray(c,dtype=float) for i,c in enumerate(controls)}
                if any(not np.isfinite(c).all() for c in arrays.values()):
                    raise ValueError(f'{stage}: controles no finitos')
                arrays['dt_s']=np.asarray(dt)
                path=args.output_dir/f'{stage}_C2_OFFLINE_NON_EXECUTABLE.npz'
                np.savez_compressed(path,**arrays)
                # Verificación de lectura antes de registrar el artefacto.
                with np.load(path,allow_pickle=False) as z:
                    for key,val in arrays.items():
                        if not np.array_equal(z[key],val): raise RuntimeError(f'{stage}: NPZ inconsistente: {key}')
                result['stages'][stage]={'status':'CERTIFIED','duration_s':duration,
                     'dt_s':dt,'npz':path.name,'sha256':digest(path),
                     'cbs_path_lengths':[len(p) for p in plan.paths],
                     'trash':list(picked),'colors':list(getattr(plan,'colors',[]))}
                write()
                print(f'{stage} CERTIFIED {duration:.3f}s {path.name}',flush=True)
                starts=tuple(p[-1] for p in plan.paths)
                if kind=='pickup':
                    for n in picked: bridge.confirm_collected(n)
        result['status']='CERTIFIED_OFFLINE_ONLY'
    except Exception as exc:
        result['status']='INCOMPLETE'
        result['problem']=str(exc)
        write()
        raise
    write()
    print('Manifest:',manifest)
    print('ATENCIÓN: controles C2 offline, no compatibles directamente con el protocolo temporal de Gazebo.')
    return 0

if __name__=='__main__':
    raise SystemExit(main())
