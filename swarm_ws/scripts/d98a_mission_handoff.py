#!/usr/bin/env python3
"""D9.8A: validate D9.2 navigation -> D6 physical mission handoff.

Strictly inert. Never publishes ROS/Gazebo commands and never writes a D6 ledger.
Physical gripper confirmation and D6 commit are deliberately separate steps.
"""
import argparse
import hashlib
import json
from pathlib import Path

STAGES = ('pickup1','delivery1','pickup2','delivery2')
GOOD_FOLLOWER = 'FINISHED_TRAJECTORY_NOT_MISSION'
GOOD_RELAY = 'NORMAL_END_STOP_CONFIRMED'

def digest(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def evaluate(run, stage, relay_path, ledger_path=None):
    run, relay_path = Path(run), Path(relay_path)
    reasons=[]
    def read(p,label):
        if not p.is_file():
            reasons.append(label+':MISSING');return None
        try:return json.loads(p.read_text(encoding='utf-8'))
        except (OSError,ValueError):
            reasons.append(label+':INVALID_JSON');return None
    pipeline=read(run/'d96c2b_report.json','PIPELINE')
    follower=read(run/'d97c'/f'{stage}_d97c_report.json','FOLLOWER')
    relay=read(relay_path,'RELAY')
    live=read(run/'live'/f'{stage}_live_report.json','D8_STAGE_REPORT')
    ledger=read(ledger_path,'D6_LEDGER') if ledger_path else None
    if stage not in STAGES: reasons.append('INVALID_STAGE')
    if pipeline:
        if pipeline.get('stage')!=stage or pipeline.get('status')!='PASS_READ_ONLY_LIVE_REPLAN':
            reasons.append('PIPELINE_NOT_VALID')
        source=run/'d94_conversion'/f'{stage}_converted_NON_EXECUTABLE.npz'
        if not source.is_file(): reasons.append('CONVERTED_CONTROLS_MISSING')
        elif digest(source)!=pipeline.get('converted_controls_sha256'):
            reasons.append('CONVERTED_SHA_MISMATCH')
    if follower:
        if follower.get('status')!=GOOD_FOLLOWER or follower.get('faults'):
            reasons.append('FOLLOWER_NOT_FINISHED')
        if follower.get('final_stop_confirmed') is not True or follower.get('end_signal_sent') is not True:
            reasons.append('FOLLOWER_STOP_NOT_CONFIRMED')
        if not isinstance(follower.get('min_measured_separation_m'),(float,int)) or follower['min_measured_separation_m']<1.15:
            reasons.append('SEPARATION_UNSAFE_OR_UNMEASURED')
        v=follower.get('final_observed_speed_m_s',{})
        if not isinstance(v,dict) or any(not isinstance(v.get(f'summit{i}'),(float,int)) or v[f'summit{i}']>=.03 for i in range(3)):
            reasons.append('FINAL_SPEED_NOT_CONFIRMED')
    if relay:
        if relay.get('status')!=GOOD_RELAY or relay.get('faults') or relay.get('normal_end') is not True:
            reasons.append('RELAY_NOT_NORMAL')
    stage_record = live.get('stages',{}).get(stage) if live else None
    names=[]
    if not isinstance(stage_record,dict):reasons.append('D8_STAGE_MISSING')
    else:
        cert=stage_record.get('hybrid_v2',{})
        if stage_record.get('certified') is not True or cert.get('certified') is not True:
            reasons.append('D8_STAGE_NOT_CERTIFIED')
        key='assigned_names' if stage.startswith('pickup') else 'delivered_names'
        names=stage_record.get(key,[])
        if len(names)!=3 or len(set(names))!=3 or not all(isinstance(x,str) and x for x in names):
            reasons.append('D8_OBJECT_IDENTIFIERS_INVALID')
    if ledger_path:
        if ledger:
            if ledger.get('version')!='swarmbots.d6.v1': reasons.append('D6_LEDGER_VERSION')
            completed=ledger.get('completed',[])
            if not isinstance(completed,list) or completed!=list(STAGES[:len(completed)]) or len(completed)>=4 or stage!=STAGES[len(completed)]:
                reasons.append('D6_STAGE_ORDER')
            if stage.startswith('delivery'):
                round_names=ledger.get('round1_names' if stage=='delivery1' else 'round2_names',[])
                if names!=round_names:reasons.append('DELIVERY_NAMES_DO_NOT_MATCH_CARGO')
            if stage.startswith('pickup') and any(n in ledger.get('collected',[]) for n in names):
                reasons.append('PICKUP_ALREADY_COLLECTED')
            if pipeline:
                if ledger.get('scenario_sha256')!=pipeline.get('scenario_sha256'):
                    reasons.append('SCENARIO_HASH_MISMATCH')
    elif stage!='pickup1': reasons.append('D6_LEDGER_REQUIRED_FOR_LATER_STAGE')
    # NEVER describe this as physical confirmation or execution authorization.
    return {'schema':'swarm.d98a.handoff.readonly.v1','stage':stage,
            'status':'READY_FOR_PHYSICAL_CONFIRMATION_ONLY' if not reasons else 'BLOCKED_HANDOFF',
            'reasons':reasons,'stage_object_names_from_D8_plan':names,
            'navigation_report_status':follower.get('status') if follower else None,
            'relay_report_status':relay.get('status') if relay else None,
            'physical_grasp_confirmed':False,'physical_delivery_confirmed':False,
            'mission_ledger_updated':False,'execution_authorized':False,
            'commands_sent':0,'publishers_created':0,
            'next_step':'Obtain actual gripper/result or release_result per robot, verify object identities and then commit D6 state with explicit physical confirmation.',
            'note':'Navigation alone cannot certify pickup or delivery. Do not reuse archived stages after robot motion.'}

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run',type=Path,required=True)
    ap.add_argument('--stage',choices=STAGES,required=True)
    ap.add_argument('--relay-report',type=Path,required=True)
    ap.add_argument('--ledger',type=Path)
    ap.add_argument('--output',type=Path)
    args=ap.parse_args()
    data=evaluate(args.run,args.stage,args.relay_report,args.ledger)
    output=args.output or args.run/'d98a'/f'{args.stage}_handoff.json'
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print(json.dumps(data,indent=2,ensure_ascii=False))
    return 0 if data['status']=='READY_FOR_PHYSICAL_CONFIRMATION_ONLY' else 2

if __name__=='__main__':raise SystemExit(main())
