#!/usr/bin/env python3
"""D9.6C.2B: fresh-odom planning -> D8 certified protocol -> D9.2/4/5.

READ ONLY. Never publishes commands or starts a mission. One stage per invocation.
A non-pickup1 stage requires an operator-provided verified D6 state.
"""
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

STAGES = ('pickup1', 'delivery1', 'pickup2', 'delivery2')

def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def run_cmd(parts, log):
    parts = [str(x) for x in parts]
    with log.open('a', encoding='utf-8') as stream:
        stream.write('\n$ ' + ' '.join(parts) + '\n');stream.flush()
        completed = subprocess.run(parts, stdout=stream, stderr=subprocess.STDOUT, check=False)
    if completed.returncode:
        raise RuntimeError(f"FAILED ({completed.returncode}) {parts[1]}: see {log}")

def source_valid(report, stage):
    s = report.get('stages',{}).get(stage,{})
    return s.get('certified') is True and s.get('hybrid_v2',{}).get('certified') is True

def stage_valid(path, stage, key, expected):
    data=json.loads(path.read_text())
    entry=data['stages'][stage] if 'stages' in data else data
    if entry.get(key)!=expected:raise RuntimeError(f'{path}: {key}={entry.get(key)!r}, expected {expected!r}')
    return entry

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scenario', type=Path, required=True, help='Exact JSON used in active Gazebo launch')
    p.add_argument('--stage', choices=STAGES, default='pickup1')
    p.add_argument('--state',type=Path,help='Existing verified D6 state, required after pickup1')
    p.add_argument('--output',type=Path,required=True,help='New, NONEXISTENT output directory')
    frame=p.add_mutually_exclusive_group(required=True)
    frame.add_argument('--accept-namespaced-world-odom',action='store_true',help='Operator verified coordinate alignment (not TF calibration)')
    frame.add_argument('--expected-frame',help='Verified common global frame')
    p.add_argument('--subdivisions',type=int,default=4)
    p.add_argument('--step',type=float,default=.005)
    a=p.parse_args(argv)
    scripts=Path(__file__).resolve().parent
    a.scenario=a.scenario.resolve()
    if not a.scenario.is_file():p.error('Missing scenario file')
    if a.output.exists():p.error('Output directory already exists; choose a new directory for every snapshot')
    if a.stage!='pickup1' and not a.state:p.error('Later stages require --state with verified physical mission ledger')
    if not (1<=a.subdivisions<=16 and 0<a.step<=.04):p.error('Invalid subdivisions or step')
    dependencies=['prepare_hybrid_v2_d6_stage.py','retime_hybrid_v2_d51.py',
                  'convert_d51_protocol_pieces.py','d92_smooth_retime.py',
                  'd94_convert_offline.py','d95_protocol_preflight.py','d6_mission_state.py']
    for dep in dependencies:
        if not (scripts/dep).is_file():p.error('Missing dependency: '+dep)
    if a.state and not a.state.is_file():p.error('Missing D6 mission state')
    a.output.mkdir(parents=True)
    log=a.output/'pipeline.log'
    report={'schema':'swarm.d96c2b.readonly.v1','stage':a.stage,'status':'STARTED',
            'execution_authorized':False,'ros_executable':False,'commands_sent':0,
            'scenario_path':str(a.scenario),'scenario_sha256':sha256(a.scenario),
            'created_wall_s':time.time()}
    result_file=a.output/'d96c2b_report.json'
    def save():result_file.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    save()
    py=sys.executable
    try:
        state=a.output/'d6_state.json'
        if a.state:
            shutil.copyfile(a.state,state)
        else:
            run_cmd([py,scripts/'d6_mission_state.py','--state',state,'init','--scenario',a.scenario],log)
        live=a.output/'live'
        cmd=[py,scripts/'prepare_hybrid_v2_d6_stage.py','--scenario',a.scenario,
             '--state',state,'--stage',a.stage,'--output-dir',live,
             '--radius','.55','--margin','.05','--max-speed','1.20','--max-accel','1.00',
             '--iterations','2']
        if a.accept_namespaced_world_odom:cmd.append('--accept-namespaced-world-odom')
        else:cmd.extend(['--expected-frame',a.expected_frame])
        run_cmd(cmd,log)
        raw_report=live/f'{a.stage}_live_report.json'
        raw_controls=live/f'{a.stage}_live_controls.npz'
        if not source_valid(json.loads(raw_report.read_text()),a.stage) or not raw_controls.is_file():
            raise RuntimeError('Fresh D8 planner did not certify its output')
        ret=live/'retimed'
        run_cmd([py,scripts/'retime_hybrid_v2_d51.py','--report',raw_report,
            '--controls',raw_controls,'--stage',a.stage,'--max-speed','1.20',
            '--max-accel','1.00','--safety-factor','1.08','--subdivisions','4',
            '--output-dir',ret],log)
        proto=live/'protocol'
        run_cmd([py,scripts/'convert_d51_protocol_pieces.py',
            '--report',ret/f'{a.stage}_retimed_report.json',
            '--controls',ret/f'{a.stage}_retimed_controls.npz',
            '--stage',a.stage,'--output-dir',proto],log)
        certified_proto=proto/f'{a.stage}_protocol_report.json'
        if not source_valid(json.loads(certified_proto.read_text()),a.stage):
            raise RuntimeError('D8 protocol missing full certified status')
        # D9 reads mission/<stage>/protocol; copy only verified files to this staging path.
        staging=a.output/'mission'/a.stage/'protocol'
        staging.mkdir(parents=True)
        for file in (certified_proto,proto/f'{a.stage}_protocol_controls.npz'):
            if not file.exists():raise RuntimeError(f'Missing converted D8 protocol: {file}')
            shutil.copyfile(file,staging/file.name)
        run_cmd([py,scripts/'d92_smooth_retime.py','--run',a.output,'--stage',a.stage],log)
        stage_valid(a.output/'d92_offline_report.json',a.stage,'status','CERTIFIED_CONSERVATIVE_OFFLINE')
        run_cmd([py,scripts/'d94_convert_offline.py','--run',a.output,'--stage',a.stage,
            '--subdivisions',a.subdivisions,'--step',a.step],log)
        stage_valid(a.output/'d94_conversion'/'d94_summary.json',a.stage,'status','CONVERTED_OFFLINE_CHECKS_PASSED')
        run_cmd([py,scripts/'d95_protocol_preflight.py','--run',a.output,'--stage',a.stage],log)
        pre=json.loads((a.output/'d95_preflight'/'d95_summary.json').read_text())
        if pre['stage_reports'][a.stage]['status']!='PASS_READ_ONLY':
            raise RuntimeError('D95 preflight rejected')
        report.update(status='PASS_READ_ONLY_LIVE_REPLAN',
            odometry_snapshot=json.loads(raw_report.read_text())['stages'][a.stage]['hybrid_v2'].get('odom_xy'),
            snapshot_time_s=json.loads(raw_report.read_text())['stages'][a.stage]['hybrid_v2'].get('odom_snapshot_time_s'),
            d92_makespan_s=json.loads((a.output/'d92_offline_report.json').read_text())['stages'][a.stage]['smooth_makespan_s'],
            source_controls_sha256=sha256(raw_controls),
            converted_controls_sha256=sha256(a.output/'d94_conversion'/f'{a.stage}_converted_NON_EXECUTABLE.npz'))
    except Exception as exc:
        report.update(status='REJECTED',reason=str(exc))
        save();print('REJECTED:',exc, file=sys.stderr)
        return 2
    save();print(json.dumps(report,indent=2,ensure_ascii=False));return 0

if __name__=='__main__':raise SystemExit(main())
