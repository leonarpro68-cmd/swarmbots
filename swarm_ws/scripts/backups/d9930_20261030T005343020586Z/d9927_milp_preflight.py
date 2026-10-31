#!/usr/bin/env python3
"""Preflight read-only for stored MILP assignment and offline timed trajectories.
No ROS and no execution authorization.
"""
import argparse, hashlib, json, math
from pathlib import Path

STAGES=('pickup1','delivery1','pickup2','delivery2')
ROBOTS=('summit0','summit1','summit2')
def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def load(path): return json.loads(Path(path).read_text(encoding='utf8'))
def verify(scenario,library,timed):
    s=load(scenario); seed=s.get('seed')
    if type(seed) is not int: raise ValueError('scenario.seed inválido')
    directory=library/f'seed_{seed}'
    best=load(directory/'best_assignment.json')
    if best.get('seed')!=seed or best.get('scenario_sha256')!=sha(scenario):
        raise ValueError('Escenario distinto de la asignación MILP')
    if sha(directory/'scenario.json')!=sha(scenario): raise ValueError('Copia de escenario distinta')
    if best.get('status')!='CERTIFIED': raise ValueError('Asignación no certificada')
    ass=best['assignment']
    if set(ass)!=set(ROBOTS) or any(len(ass[r])!=2 for r in ROBOTS): raise ValueError('Asignación incompleta')
    names=[a for r in ROBOTS for a in ass[r]]
    if len(set(names))!=6 or not all(isinstance(n,str) and n.startswith('trash_') for n in names):
        raise ValueError('Basuras no únicas')
    m=load(timed/'manifest.json')
    if m.get('status')!='OFFLINE_TIMED_FORMAT_VERIFIED' or m.get('execution_authorized') is not False:
        raise ValueError('Manifiesto offline no verificado')
    if m.get('seed')!=seed or m.get('scenario_sha256')!=sha(scenario) or m.get('assignment')!=ass:
        raise ValueError('Manifiesto no corresponde a biblioteca actual')
    if set(m.get('stages',{}))!=set(STAGES): raise ValueError('Faltan etapas')
    import numpy as np
    stages={}
    for stage in STAGES:
        e=m['stages'][stage]; filename=e['npz']
        if Path(filename).name!=filename: raise ValueError('Nombre NPZ inseguro')
        p=timed/filename
        if sha(p)!=e['sha256']: raise ValueError(f'{stage}: hash NPZ inválido')
        with np.load(p, allow_pickle=False) as z:
            for i in range(3):
                c=z[f'R{i}_controls']; d=z[f'R{i}_durations_s']
                if c.ndim!=3 or c.shape[1:]!=(6,2) or len(c)==0 or d.shape!=(len(c),):
                    raise ValueError(f'{stage} summit{i}: NPZ incompatible')
                if not (np.isfinite(c).all() and np.isfinite(d).all() and (d>0).all()):
                    raise ValueError(f'{stage}: valores inválidos')
        stages[stage]={'npz':str(p.resolve()),'sha256':e['sha256']}
    return {'status':'MILP_PREFLIGHT_VALID_OFFLINE_ONLY','seed':seed,'scenario_sha256':sha(scenario),
            'library_best_sha256':sha(directory/'best_assignment.json'),
            'timed_manifest_sha256':sha(timed/'manifest.json'),'assignment':ass,
            'estimated_total_s':best['estimated_total_s'],'stages':stages,
            'physical_execution_authorized':False}

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scenario',type=Path,required=True)
    p.add_argument('--library-dir',type=Path,default=Path('scripts/milp_library'))
    p.add_argument('--timed-dir',type=Path,required=True)
    p.add_argument('--output',type=Path)
    a=p.parse_args()
    try:
        r=verify(a.scenario,a.library_dir,a.timed_dir)
        rc=0
    except Exception as exc:
        r={'status':'REJECT','problems':[f'{type(exc).__name__}: {exc}']}
        rc=2
    if a.output:
        a.output.parent.mkdir(parents=True,exist_ok=True)
        a.output.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(r,ensure_ascii=False,indent=2))
    return rc
if __name__=='__main__': raise SystemExit(main())
