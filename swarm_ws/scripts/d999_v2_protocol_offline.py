#!/usr/bin/env python3
"""D9.9.9 V2 protocol roundtrip/offline validation. NO ROS publishers or robot motion."""
import argparse, hashlib, importlib.util, json, math, sys
from pathlib import Path
import numpy as np

def main():
    a=argparse.ArgumentParser()
    a.add_argument('--trajectory',required=True)
    a.add_argument('--protocol-file',default='src/swarm_behavior/swarm_behavior/swarm_planning/hybrid_refinement/timed_protocol.py')
    a.add_argument('--output',required=True)
    args=a.parse_args()
    proto=Path(args.protocol_file).resolve()
    spec=importlib.util.spec_from_file_location('d999_protocol',proto)
    module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    z=np.load(args.trajectory,allow_pickle=False)
    result={'schema':'swarm.d999.protocol.offline.v1','status':'REJECT','execution_authorized':False,'commands_sent':0,'robots':{},'problems':[],'trajectory_sha256':hashlib.sha256(Path(args.trajectory).read_bytes()).hexdigest()}
    for i in range(3):
        name=f'summit{i}'
        controls=np.asarray(z[f'R{i}_controls'],dtype=float)
        durations=np.asarray(z[f'R{i}_durations_s'],dtype=float)
        cmd={'schema':module.SCHEMA_V2,'action':'start','session_id':'offline_test','robot':name,
             'frame_id':name+'/odom','start_time_s':100.,'control_points_m':controls.tolist(),
             'segment_durations_s':durations.tolist(),'speed_limit_m_s':1.2,
             'acceleration_limit_m_s2':1.0,'max_tracking_error_m':.15,
             'goal_tolerance_m':.05,'goal_settle_s':.5,'finish_grace_s':4.}
        try:
            parsed=module.decode_command(json.dumps(cmd,allow_nan=False))
            assert math.isclose(parsed.duration_s,float(durations.sum()),abs_tol=1e-8)
            assert len(parsed.segment_durations_s)==len(durations)
            assert module.decode_command(parsed.to_json()).segment_durations_s==parsed.segment_durations_s
            p0,_,ph0=module.reference(parsed,99.99)
            pe,_,phe=module.reference(parsed,100.+parsed.duration_s+.001)
            assert np.linalg.norm(np.array(p0)-controls[0,0])<1e-9 and ph0=='before'
            assert np.linalg.norm(np.array(pe)-controls[-1,-1])<1e-9 and phe=='after'
            cumulative=np.cumsum(durations)
            maxerr=0.
            for j,t in enumerate(cumulative[:-1]):
                p,_,_=module.reference(parsed,100.+float(t))
                maxerr=max(maxerr,float(np.linalg.norm(np.asarray(p)-controls[j,-1])))
            assert maxerr<1e-7
            result['robots'][name]={'status':'PASS','segments':len(durations),'duration_s':parsed.duration_s,'boundary_error_m':maxerr}
        except Exception as exc:
            result['problems'].append(f'{name}: {exc}')
    if not result['problems']: result['status']='PASS_V2_OFFLINE_NOT_EXECUTABLE'
    out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
    return 0 if not result['problems'] else 2
if __name__=='__main__':raise SystemExit(main())
