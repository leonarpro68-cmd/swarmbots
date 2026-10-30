#!/usr/bin/env python3
"""D9.8C.2: proposed D8-style delivery waves from read-only snapshot. No Gazebo commands."""
import argparse, json, math
from pathlib import Path
from collections import Counter
ROBOTS=('summit0','summit1','summit2')

def make_waves(snap):
    if snap.get('status')!='LOAD_GEOMETRY_COMPATIBLE_NOT_JOINT_CERTIFIED' or snap.get('issues'):
        raise ValueError('INVALID_LOAD_SNAPSHOT')
    rs=snap['robots']
    if set(rs)!=set(ROBOTS):raise ValueError('ROBOT_SET_MISMATCH')
    objs=[rs[r]['object'] for r in ROBOTS]
    if len(set(objs))!=3:raise ValueError('DUPLICATE_CARGO')
    for r in ROBOTS:
        v=rs[r]
        if not v.get('distance_compatible_with_load') or v.get('observed_speed_m_s',1)>.03:
            raise ValueError(r+':INVALID_LOAD_POSE')
    # Snapshot gives explicit colors in trash object names for this seed; reject unknown naming.
    colors={r:('verde' if rs[r]['object'].endswith('_verde') else 'azul' if rs[r]['object'].endswith('_azul') else None) for r in ROBOTS}
    if any(v is None for v in colors.values()):raise ValueError('UNKNOWN_TRASH_COLOR')
    counts=Counter(colors.values())
    pending=sorted(ROBOTS,key=lambda r:(counts[colors[r]],r))
    waves=[]
    while pending:
        seen=set(); wave=[]
        for r in pending:
            if colors[r] not in seen:wave.append(r);seen.add(colors[r])
        waves.append(wave)
        pending=[r for r in pending if r not in wave]
    return {
        'schema':'swarm.d98c2.delivery_wave_proposal.v1',
        'status':'PROPOSED_WAVES_ONLY_BLOCKED_FOR_EXECUTION',
        'waves':[{'number':i+1,'robots':[{'robot':r,'object':rs[r]['object'],'deposit_color':colors[r]} for r in wave]} for i,wave in enumerate(waves)],
        'wave_rule':'At most one Summit per deposit per wave (D8); CBS/geometric clearance must still be checked for all active and stationary robots.',
        'snapshot_joint_certified':bool(snap.get('gazebo_joint_attachment_verified',False)),
        'joint_verification_pending':not bool(snap.get('gazebo_joint_attachment_verified',False)),
        'requires_fresh_live_positions':True,
        'cbs_routes_computed':False,'mission_ledger_updated':False,'release_authorized':False,
        'commands_sent':0,'publishers_created':0,
        'next_step':'Confirm live cargo attachment; prepare D8 compatible mission ledger and fresh CBS+D9.2 delivery1 per wave; preserve stationary robot avoidance; confirm detach before next wave.'}

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--snapshot',type=Path,required=True)
    p.add_argument('--output',type=Path)
    a=p.parse_args()
    data=make_waves(json.loads(a.snapshot.read_text()))
    out=a.output or a.snapshot.parent/'delivery1_wave_proposal.json'
    out.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(data,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
