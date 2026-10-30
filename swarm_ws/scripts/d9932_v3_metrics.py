#!/usr/bin/env python3
"""Métricas de telemetría de misión MILP con reloj V3 exacto (sin ROS).

Los NPZ V3 contienen tiempos VIRTUALES; no se deben evaluar mediante
D9916, que interpreta las duraciones de los NPZ como tiempos de pared.
Compara rapidez teórica v=q(tau)*v_Bezier(tau) con rapidez observada.
La estimación de aceleración de odometría por diferencias finitas tiene ruido,
por ello NO se utiliza como certificado de límites dinámicos.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import numpy as np
from swarm_behavior.swarm_planning.hybrid_refinement.successive_clock import clock_from_payload

STAGES=('pickup1','delivery1','pickup2','delivery2')
ROBOTS=('summit0','summit1','summit2')


def speed_reference(tau, controls, durations, knots, q):
    if tau < 0 or tau >= knots[-1]:
        return 0.0
    i=min(len(durations)-1,int(np.searchsorted(knots,tau,side='right')-1))
    u=min(1.,max(0.,(tau-knots[i])/durations[i]))
    deriv=5*np.diff(controls[i],axis=0)/durations[i]
    coeff=np.array([1,4,6,4,1],dtype=float)*np.power(u,np.arange(5))*np.power(1-u,4-np.arange(5))
    v=coeff@deriv
    return float(np.linalg.norm(v)*q)


def compute(odom,stage_windows):
    records={r:[] for r in ROBOTS}
    for line in Path(odom).read_text(encoding='utf8').splitlines():
        if not line.strip():continue
        obj=json.loads(line)
        name=obj.get('robot')
        if name in records and isinstance(obj.get('clock_s'),(float,int)):
            records[name].append(obj)
    windows=json.loads(Path(stage_windows).read_text(encoding='utf8'))['stages']
    if {x['stage'] for x in windows}!=set(STAGES):
        raise ValueError('V3_METRICS_REQUIRES_FOUR_STAGES')
    output={'schema':'swarm.d9932.v3_metrics.v1',
            'time_model':'common_collective_successive_v1',
            'note':'Velocidad observada por odometría frente a referencia V3; no certifica dinámica física ni colisiones',
            'stages':{}}
    for entry in windows:
        stage=entry['stage']
        if entry.get('time_model')!='common_collective_successive_v1' or 'clock_json' not in entry:
            raise ValueError(f'{stage}: MISSING_SUCCESSIVE_CLOCK')
        obj=json.loads(Path(entry['clock_json']).read_text(encoding='utf8'))
        payload={'model':'common_collective_successive_v1','baseline_scale':obj['uniform_scale'],
                 'ramps':obj['ramps'],'duration_original_s':obj['duration_original_s'],
                 'duration_successive_s':obj['duration_successive_s']}
        clock=clock_from_payload(payload)
        start=float(entry['start_time_s'])
        finish=start+clock.duration_s
        stage_data={'reference_duration_s':clock.duration_s,'robots':{}}
        with np.load(entry['trajectory'],allow_pickle=False) as z:
            for i,name in enumerate(ROBOTS):
                c=np.asarray(z[f'R{i}_controls'],dtype=float)
                d=np.asarray(z[f'R{i}_durations_s'],dtype=float)
                knots=np.r_[0.,np.cumsum(d)]
                rows=[r for r in records[name] if start<=float(r['clock_s'])<=finish]
                if len(rows)<3:
                    stage_data['robots'][name]={'status':'INSUFFICIENT_ODOM','samples':len(rows)}
                    continue
                measured=[];expected=[]
                for r in rows:
                    elapsed=float(r['clock_s'])-start
                    tau=clock.inverse(elapsed)
                    expected.append(speed_reference(tau,c,d,knots,clock.scale(tau)))
                    measured.append(float(r['speed_m_s']))
                measured=np.asarray(measured);expected=np.asarray(expected)
                if not np.isfinite(measured).all() or not np.isfinite(expected).all():
                    raise ValueError(f'{stage}/{name}: NONFINITE_ODOM_OR_REFERENCE')
                diff=measured-expected
                stage_data['robots'][name]={
                    'status':'ANALYZED','samples':len(rows),
                    'theoretical_peak_speed_m_s':float(expected.max()),
                    'observed_peak_speed_m_s':float(measured.max()),
                    'speed_rmse_m_s':float(np.sqrt(np.mean(diff**2))),
                    'speed_mae_m_s':float(np.mean(np.abs(diff))),
                    'speed_bias_m_s':float(diff.mean())}
        output['stages'][stage]=stage_data
    return output


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--odom-jsonl',required=True)
    p.add_argument('--stage-windows',required=True)
    p.add_argument('--out-dir',required=True)
    a=p.parse_args()
    result=compute(a.odom_jsonl,a.stage_windows)
    dest=Path(a.out_dir)
    dest.mkdir(parents=True,exist_ok=True)
    target=dest/'v3_speed_metrics.json'
    target.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
    print(json.dumps({'status':'V3_METRICS_READY','report':str(target)}))

if __name__=='__main__': main()
