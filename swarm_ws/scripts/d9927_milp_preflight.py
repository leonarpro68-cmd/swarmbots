#!/usr/bin/env python3
"""D9.9.31 preflight MILP V3: identidad, NPZ y reloj certificados, sin ROS.

Legacy D9930 sigue bloqueado si falta paquete V3; previene ejecutar NPZ V2
con costos calculados sobre un reloj sucesivo que el seguidor no interpreta.
"""
import argparse,hashlib,json,math
from pathlib import Path
import numpy as np
STAGES=('pickup1','delivery1','pickup2','delivery2')
ROBOTS=('summit0','summit1','summit2')
MODEL='common_collective_successive_v1'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load(p):return json.loads(Path(p).read_text(encoding='utf8'))


def verify(scenario,library,timed):
    scenario=Path(scenario).resolve();library=Path(library).resolve();timed=Path(timed).resolve()
    s=load(scenario);seed=s.get('seed')
    if type(seed) is not int:raise ValueError('INVALID_SCENARIO_SEED')
    lib=library/f'seed_{seed}';best=load(lib/'best_assignment.json');sid=sha(scenario)
    if best.get('seed')!=seed or best.get('scenario_sha256')!=sid or sha(lib/'scenario.json')!=sid:
        raise ValueError('SCENARIO_LIBRARY_MISMATCH')
    if best.get('status')!='CERTIFIED' or best.get('time_model')!=MODEL:
        raise ValueError('D9931_REQUIRES_SUCCESSIVE_LIBRARY')
    ass=best['assignment']
    if set(ass)!=set(ROBOTS) or any(len(ass[r])!=2 for r in ROBOTS):raise ValueError('ASSIGNMENT_INCOMPLETE')
    names=[n for r in ROBOTS for n in ass[r]]
    if len(set(names))!=6 or not all(isinstance(n,str) and n.startswith('trash_') for n in names):
        raise ValueError('TRASH_IDENTITIES_INVALID')
    m=load(timed/'manifest.json')
    if m.get('schema')!='swarm.d9931.successive_bundle.v1' or m.get('status')!='OFFLINE_V3_BUNDLE_VERIFIED':
        raise ValueError('SUCCESSIVE_V3_BUNDLE_REQUIRED_NOT_OLD_NPZ')
    if (m.get('seed')!=seed or m.get('scenario_sha256')!=sid or m.get('assignment')!=ass or
        m.get('library_best_sha256')!=sha(lib/'best_assignment.json') or
        m.get('time_model')!=MODEL or m.get('execution_authorized') is not False):
        raise ValueError('SUCCESSIVE_V3_MANIFEST_IDENTITY_MISMATCH')
    if set(m.get('stages',{}))!=set(STAGES):raise ValueError('FOUR_STAGES_REQUIRED')
    stages={}
    for stage in STAGES:
        e=m['stages'][stage];f=e['npz'];cname=e['clock']
        if f!=Path(f).name or cname!=Path(cname).name:raise ValueError(f'{stage}: UNSAFE_FILENAME')
        npz=timed/f;clockpath=timed/cname
        if sha(npz)!=e['sha256'] or sha(clockpath)!=e['clock_sha256']:
            raise ValueError(f'{stage}: NPZ_CLOCK_HASH_MISMATCH')
        c=load(clockpath)
        if (c.get('status')!='OFFLINE_SCHEDULE_CERTIFIED_NOT_EXECUTABLE' or c.get('input_sha256')!=e['sha256'] or
            c.get('common_time_warp') is not True):raise ValueError(f'{stage}: CLOCK_INVALID')
        with np.load(npz,allow_pickle=False) as z:
            for i in range(3):
                pts=np.asarray(z[f'R{i}_controls']); d=np.asarray(z[f'R{i}_durations_s'])
                if pts.ndim!=3 or pts.shape[1:]!=(6,2) or len(pts)<1 or d.shape!=(len(pts),):
                    raise ValueError(f'{stage}: INVALID_NPZ_SHAPES')
                if not np.isfinite(pts).all() or not np.isfinite(d).all() or np.any(d<=0):
                    raise ValueError(f'{stage}: INVALID_NPZ_VALUES')
                if abs(float(sum(d))-float(c['duration_original_s']))>1e-4:
                    raise ValueError(f'{stage}: VIRTUAL_HORIZON_MISMATCH')
        stages[stage]={'npz':str(npz),'sha256':e['sha256'],
                       'clock':str(clockpath),'clock_sha256':e['clock_sha256'],
                       'duration_successive_s':c['duration_successive_s']}
    return {'status':'MILP_V3_PREFLIGHT_VALID_OFFLINE_ONLY','seed':seed,'scenario_sha256':sid,
            'library_best_sha256':sha(lib/'best_assignment.json'),
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
    try:r=verify(a.scenario,a.library_dir,a.timed_dir);rc=0
    except Exception as exc:r={'status':'REJECT','problems':[f'{type(exc).__name__}: {exc}']};rc=2
    if a.output:
        a.output.parent.mkdir(parents=True,exist_ok=True)
        a.output.write_text(json.dumps(r,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(r,indent=2,ensure_ascii=False))
    return rc
if __name__=='__main__':raise SystemExit(main())
