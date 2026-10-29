#!/usr/bin/env python3
"""Verifica reposo sostenido de los tres Summit antes del despeje CBS."""
import argparse,json,math,sys,time
from pathlib import Path
from d6_grasp_existing import snapshot_odom

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--settle-s',type=float,default=1.2)
    p.add_argument('--timeout-s',type=float,default=15.)
    p.add_argument('--speed-limit',type=float,default=.035)
    a=p.parse_args()
    start=time.monotonic();stable=None;status='REJECT_STOPPED';reason=[];last={}
    while time.monotonic()-start<a.timeout_s:
        try:
            raw=snapshot_odom()
            last={r:float(raw[r].get('speed_m_s',raw[r].get('speed',0.0))) for r in ('summit0','summit1','summit2')}
            ok=all(math.isfinite(v) and v<a.speed_limit for v in last.values())
        except Exception as exc:
            ok=False;reason=[f'{type(exc).__name__}: {exc}']
        if ok:
            if stable is None:stable=time.monotonic()
            if time.monotonic()-stable>=a.settle_s:
                status='ROBOTS_SETTLED_VERIFIED';reason=[];break
        else:stable=None
        time.sleep(.15)
    if status!='ROBOTS_SETTLED_VERIFIED' and not reason:reason=['Tiempo de estabilización agotado']
    d={'status':status,'speeds_m_s':last,'settle_s':a.settle_s,'problems':reason}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(d,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(d,ensure_ascii=False))
    return 0 if status=='ROBOTS_SETTLED_VERIFIED' else 2
if __name__=='__main__':sys.exit(main())
