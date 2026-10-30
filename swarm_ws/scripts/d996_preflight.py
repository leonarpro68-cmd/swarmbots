#!/usr/bin/env python3
"""D9.9.6: preflight estático y observación ROS/Gazebo sin publicación ni movimiento.

Estados informativos exclusivamente; nunca autoriza navegación.
"""
import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path


def digest(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for chunk in iter(lambda:f.read(1024*1024), b''): h.update(chunk)
    return h.hexdigest()


def static_check(reference, world, scenario, npz, conversion, offline_preflight):
    problems=[]; facts={}
    for key,p in [('world',world),('scenario',scenario),('npz',npz),('conversion',conversion),('offline_preflight',offline_preflight)]:
        facts[key+'_exists']=p.is_file()
        if not p.is_file():problems.append('MISSING_'+key.upper())
    if not isinstance(reference.get('robots'),dict) or set(reference['robots'])!={'summit0','summit1','summit2'}:problems.append('INVALID_REFERENCE_ROBOTS')
    if world.is_file():
        facts['world_sha256']=digest(world)
        if facts['world_sha256']!=reference.get('world',{}).get('run007_sdf_sha256'): problems.append('WORLD_HASH_MISMATCH')
    if scenario.is_file():
        facts['scenario_sha256']=digest(scenario)
        if facts['scenario_sha256']!=reference.get('world',{}).get('run007_scenario_sha256'):problems.append('SCENARIO_HASH_MISMATCH')
    if conversion.is_file():
        try:
            c=json.loads(conversion.read_text())
            facts['conversion_status']=c.get('status')
            if c.get('stage')!='pickup1' or c.get('status')!='CONVERTED_OFFLINE_CHECKS_PASSED':problems.append('CONVERSION_REPORT_INVALID')
            if npz.is_file() and digest(npz)!=c.get('source_sha256',{}).get('converted_npz'):problems.append('CONVERSION_NPZ_HASH_MISMATCH')
        except (ValueError,KeyError,TypeError): problems.append('CONVERSION_REPORT_UNREADABLE')
    if offline_preflight.is_file():
        try:
            p=json.loads(offline_preflight.read_text())
            facts['offline_preflight_status']=p.get('status')
            if p.get('stage')!='pickup1' or p.get('status')!='PASS_OFFLINE_READ_ONLY' or p.get('failures') or not p.get('robot_checks'): problems.append('OFFLINE_PREFLIGHT_INVALID')
            # Only compare hashes if both reports supply corresponding values.
            if npz.is_file():
                recorded=p.get('checked_sha256',{})
                if isinstance(recorded,dict):
                    for k,v in recorded.items():
                        if 'npz' in k.lower() and isinstance(v,str) and len(v)==64 and digest(npz)!=v:problems.append('PREFLIGHT_NPZ_HASH_MISMATCH')
        except (ValueError,KeyError,TypeError):problems.append('OFFLINE_PREFLIGHT_UNREADABLE')
    return problems,facts


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--reference-report',type=Path,required=True)
    ap.add_argument('--world-sdf',type=Path,required=True)
    ap.add_argument('--scenario',type=Path,required=True)
    ap.add_argument('--trajectory',type=Path,required=True)
    ap.add_argument('--conversion-report',type=Path,required=True)
    ap.add_argument('--offline-preflight',type=Path,required=True)
    ap.add_argument('--live',action='store_true',help='Observa ROS/Gazebo; no publica ni mueve')
    ap.add_argument('--observe-seconds',type=float,default=8)
    ap.add_argument('--output',type=Path,default=Path('scripts/resultados/d996_preflight.json'))
    a=ap.parse_args()
    try: reference=json.loads(a.reference_report.read_text())
    except (OSError,ValueError) as exc: ap.error('Reference report unavailable: '+str(exc))
    problems,facts=static_check(reference,a.world_sdf,a.scenario,a.trajectory,a.conversion_report,a.offline_preflight)
    live=None
    if a.live and not problems:
        observer=Path(__file__).with_name('d99_inicio_gazebo_solo_lectura.py')
        livefile=a.output.with_name(a.output.stem+'_live.json')
        cmd=[sys.executable,str(observer),'--reference-report',str(a.reference_report),'--world-sdf',str(a.world_sdf),'--scenario',str(a.scenario),'--observe-seconds',str(a.observe_seconds),'--output',str(livefile)]
        try:
            cp=subprocess.run(cmd,capture_output=True,text=True,timeout=max(25,a.observe_seconds+20))
            if livefile.exists():
                live=json.loads(livefile.read_text())
                if live.get('status')!='PASS_INERT_OBSERVATION_NOT_EXECUTION_AUTHORIZATION':problems.extend(live.get('problems',[]) or ['LIVE_GATE_REJECTED'])
            else:problems.append('LIVE_OBSERVER_FAILED_TO_WRITE_REPORT')
            if cp.returncode!=0:problems.append('LIVE_OBSERVER_FAILED')
            facts['live_stdout_tail']=cp.stdout[-1200:]
            facts['live_stderr_tail']=cp.stderr[-1200:]
        except (OSError,subprocess.TimeoutExpired,ValueError) as exc: problems.append('LIVE_OBSERVER_ERROR');facts['live_error']=str(exc)
    result={'schema':'swarm.d996.preflight.readonly.v1','status':('REJECT_PREFLIGHT' if problems else ('PASS_STATIC_AND_LIVE_INERT' if a.live else 'PASS_STATIC_LIVE_NOT_RUN')),'problems':sorted(set(problems)),'facts':facts,'live_observation':live,'tf_certified':False,'execution_authorized':False,'commands_sent':0,'publisher_created':False}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({'status':result['status'],'problems':result['problems'],'output':str(a.output)},indent=2,ensure_ascii=False))
    return 2 if problems else 0

if __name__=='__main__':raise SystemExit(main())
