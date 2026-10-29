#!/usr/bin/env python3
"""D9.8C: non-actuating pickup evidence audit; never commits mission state."""
import argparse,json,hashlib
from pathlib import Path
ROBOTS=('summit0','summit1','summit2')
OK='GRASP_MANAGER_ACK_NO_LEDGER_COMMIT'
def audit(handoff,reports):
    issues=[]
    names=handoff.get('stage_object_names_from_D8_plan',[])
    if handoff.get('status')!='READY_FOR_PHYSICAL_CONFIRMATION_ONLY' or handoff.get('stage')!='pickup1':issues.append('INVALID_HANDOFF')
    if len(names)!=3 or len(set(names))!=3:issues.append('INVALID_NAMES')
    result={}; used=set()
    for i,robot in enumerate(ROBOTS):
        name=names[i] if i<len(names) else None
        report=reports.get(robot)
        if report is None:
            issues.append(robot+':MISSING_REPORT');result[robot]={'object':name,'evidence':'MISSING'};continue
        matches=[x for x in report.get('robot_results',[]) if x.get('robot')==robot]
        if (report.get('status')!='GRASP_ACK_RECEIVED_NO_LEDGER_COMMIT' or len(matches)!=1 or
            matches[0].get('status')!=OK or matches[0].get('object')!=name or
            'GRASP_ACK_ATTACHED_BY_D8_MANAGER' not in [s.get('phase') for s in matches[0].get('steps',[]) ]):
            issues.append(robot+':ACK_OR_OBJECT_MISMATCH')
            result[robot]={'object':name,'evidence':'UNVERIFIED'}
        else:
            result[robot]={'object':name,'evidence':'D8_MANAGER_ACK_AND_TIP_PRECHECK','source_status':matches[0]['status']}
        if name in used:issues.append('DUPLICATE_OBJECT')
        used.add(name)
    return {'schema':'swarm.d98c.pickup_evidence.v1','status':'ACK_AUDIT_PASSED_NOT_ATTACHMENT_CERTIFIED' if not issues else 'BLOCKED',
            'issues':issues,'robots':result,'physical_joint_attachment_verified':False,
            'mission_ledger_updated':False,'delivery1_execution_authorized':False,
            'note':'D8 Int8 ACK does not identify the attachment. Gazebo joint inspection is required before committing mission state.'}
def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--handoff',type=Path,required=True)
    for r in ROBOTS:p.add_argument('--'+r+'-report',dest=r,type=Path,required=True)
    p.add_argument('--output',type=Path)
    a=p.parse_args(); h=json.loads(a.handoff.read_text())
    reports={}
    for r in ROBOTS:
        f=getattr(a,r)
        if f.is_file():reports[r]=json.loads(f.read_text())
    data=audit(h,reports)
    out=a.output or a.handoff.parent.parent/'d98c'/'pickup1_evidence.json'
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(data,indent=2,ensure_ascii=False));return 0 if not data['issues'] else 2
if __name__=='__main__':raise SystemExit(main())
