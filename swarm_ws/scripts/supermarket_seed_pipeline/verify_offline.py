#!/usr/bin/env python3
"""Audit exported *offline* Hybrid V2/V3 artifacts (never ROS execution approval)."""
from pathlib import Path
import argparse,json,sys
import numpy as np
from original_v3.successive_clock import SuccessiveClock,command_envelopes
STAGES=('pickup1','delivery1','pickup2','delivery2')

def verify(root:Path,speed=2.,accel=2.):
    out={'status':'PASS_OFFLINE_ARTIFACTS_NOT_EXECUTABLE','execution_authorized':False,'stages':{},'problems':[]}
    for stage in STAGES:
        p=root/stage
        try:
            rep=json.loads((p/f'{stage}_report.json').read_text())
            clock=json.loads((p/f'{stage}_MILP_V3_clock.json').read_text())
            assert rep['status']=='PASS_OFFLINE_HYBRID_V2_V3_NOT_EXECUTABLE',stage+' status'
            assert rep['refinements_accepted']==5 and rep['continuous_certificate']['pass_certificate']
            assert rep['execution_authorized'] is False and clock['execution_authorized'] is False
            with np.load(p/f'{stage}_MILP_TIMED_OFFLINE.npz',allow_pickle=False) as z:
                cs=[z[f'R{i}_controls'] for i in range(3)]
                ds=[z[f'R{i}_durations_s'] for i in range(3)]
            K=len(cs[0]);assert K==rep['cbs_ticks']
            assert all(c.shape==(K,6,2) for c in cs)
            assert all(np.allclose(d,ds[0]) for d in ds)
            sc=SuccessiveClock(clock['uniform_scale'],clock['ramps'],clock['duration_original_s'],clock['duration_successive_s'])
            bounds=[command_envelopes(c,d.tolist(),sc) for c,d in zip(cs,ds)]
            vmax=max(v for v,a in bounds);amax=max(a for v,a in bounds)
            assert vmax<=speed+1e-6 and amax<=accel+1e-6
            out['stages'][stage]={'status':'PASS','refinements':rep['refinements_accepted'],'segments':K,
                                   'duration_v3_s':round(sc.duration_s,6),'v_bound':round(vmax,6),'a_bound':round(amax,6)}
        except Exception as e:
            out['status']='REJECTED_NOT_EXECUTABLE';out['problems'].append(f'{stage}: {type(e).__name__}: {e}')
    return out

def main():
    p=argparse.ArgumentParser();p.add_argument('--results',type=Path,required=True);a=p.parse_args()
    r=verify(a.results)
    print(json.dumps(r,indent=2,ensure_ascii=False))
    return 0 if not r['problems'] else 2
if __name__=='__main__':sys.exit(main())
