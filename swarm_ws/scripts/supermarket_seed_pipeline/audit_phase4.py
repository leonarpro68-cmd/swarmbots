#!/usr/bin/env python3
"""Fase 4: auditoría geométrica de MILP->CBS antes de Hybrid V2 y V3.
No mueve robots ni crea trayectorias ejecutables. Se detiene ante fallos.
"""
from __future__ import annotations
import argparse, hashlib, json, sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from swarm_maps import map_files
from validar_corredores import run as check_corridors

STAGES=('pickup1','delivery1','pickup2','delivery2')
def read(path): return json.loads(Path(path).read_text(encoding='utf8'))
def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def create_mission(cbs):
    if cbs.get('status')!='OFFLINE_CBS_ONLY_NOT_EXECUTABLE' or cbs.get('execution_authorized') is not False: raise ValueError('CBS no es un reporte offline reconocido')
    if not isinstance(cbs.get('seed'), int) or cbs['seed'] < 0: raise ValueError('Semilla inválida')
    if list(cbs.get('stage_sequence',[]))!=list(STAGES): raise ValueError('Secuencia de etapas distinta')
    pending={v[f'pickup{k}'] for v in cbs['assignment'].values() for k in (1,2)}
    if len(pending)!=6: raise ValueError('Asignación no cubre seis basuritas distintas')
    result=[]
    for stage in STAGES:
        s=cbs['stages'][stage]
        if s['status']!='CBS_CONFLICT_FREE_DISCRETE': raise ValueError(f'{stage} no certificado por CBS')
        paths=[p['grid'] for p in s['paths']]
        if len(paths)!=3: raise ValueError('Se requieren tres robots')
        goals=[p[-1] for p in paths]
        if paths[0][0]!=s['starts_grid'][0]:raise ValueError('Inicio diferente del reporte CBS')
        targets=[cbs['assignment'][f'summit{i}'][stage] for i in range(3)]
        result.append({'stage':stage,'paths_grid':paths,'starts':s['starts_grid'],
                       'selected_goals':goals,'goal_options':s.get('goal_options',[[g] for g in goals]),
                       'pending_before':sorted(pending),'targets':targets})
        if stage.startswith('pickup'): pending.difference_update(targets)
    if pending:raise ValueError('No se recogieron todas las basuritas')
    return {'seed':cbs['seed'],'success':True,'best':{'assignment':{'pickup1':result[0]['targets'],'pickup2':result[2]['targets']},'stages':result}}

def audit(cbs_path,scene_path,output,geometry_path):
    cbs=read(cbs_path);scene=read(scene_path);geo=read(geometry_path)
    if scene['seed']!=cbs['seed']:raise ValueError('Incompatibilidad de semillas')
    mission=create_mission(cbs)
    audit=check_corridors(scene,mission,geo,margin=0.0,detailed=True)
    report={'schema':'supermarket.milp.phase4.geometry_gate.v1','status':'PASS_CORRIDOR_PREFLIGHT_ONLY' if audit['pass_all'] else 'BLOCKED_CORRIDOR_GEOMETRY',
            'execution_authorized':False,'source_sha256':{'cbs':sha(cbs_path),'scenario':sha(scene_path),'geometry':sha(geometry_path)},
            'stage_audit':audit['stages'],
            'next_gate':'Hybrid V2 SLSQP + 5 refinements + convex-hull certificate, then common V3 timewarp; NOT yet performed',
            'notes':'Los depósitos en la fase 3 usaron metas proxy fijas; la misión necesita compatibilidad física D8 y exclusiones estáticas/dinámicas antes de generar NPZ.'}
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    if output.exists():raise FileExistsError(f'No se sobrescribe: {output}')
    output.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    for row in audit['stages']:
        print(row['stage'],row['status'],row.get('reason','')[:180])
    print('STATUS:',report['status'],'\nREPORTE:',output)
    return 0 if audit['pass_all'] else 2

def main():
    p=argparse.ArgumentParser(description=__doc__);here=Path(__file__).resolve().parent
    p.add_argument('--cbs',type=Path,required=True)
    p.add_argument('--scenario',type=Path,required=True)
    p.add_argument('--geometry',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    return audit(a.cbs,a.scenario,a.output,a.geometry)
if __name__=='__main__':sys.exit(main())
