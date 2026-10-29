#!/usr/bin/env python3
"""Lectura y muestreo D5.1/D9.9, solo visualización; sin ROS."""
import json
from pathlib import Path
import numpy as np
from d9_offline_retime import STAGES, load
from d99_collective_phase_retime import solve, sample_solution


def _load_json(p):
    with open(p, encoding='utf-8') as f:
        return json.load(f)


def stages_from_args(run_dir=None, manifest=None, stages=None, output_dir=None):
    if bool(run_dir) == bool(manifest):
        raise ValueError('Especifica exactamente --run-dir o --manifest')
    if manifest:
        m=_load_json(manifest)
        if m.get('status') != 'PASS_OFFLINE_FRESH_START_NOT_EXECUTABLE':
            raise ValueError('Manifest fresh-start no aprobado')
        run=Path(m['artifacts']['source_protocol']).parents[2] # mission/pickup1/protocol -> run/mission
        # protocol path: run/mission/pickup1/protocol/file; parents[3] -> run
        run=Path(m['artifacts']['source_protocol']).parents[3]
        stage=m.get('stage','pickup1')
        if stage not in STAGES: raise ValueError(f'Etapa desconocida: {stage}')
        return run, [stage], {stage:Path(m['artifacts']['collective_report'])}, {stage:Path(m['scenario'])}
    run=Path(run_dir).expanduser().resolve()
    chosen=stages or list(STAGES)
    if any(x not in STAGES for x in chosen): raise ValueError(f'Etapas inválidas: {chosen}')
    # historical report can be supplied in run/d99 or run/d99_resultados; if absent solve offline
    reports={}
    for st in chosen:
        candidates=[run/'d99'/f'{st}_d99_report.json',run/'d99_resultados'/f'{st}_d99_report.json',run/f'{st}_d99_report.json']
        reports[st]=next((p for p in candidates if p.exists()),None)
    return run,chosen,reports,{x:run/'scenario.json' for x in chosen}


def samples_for_stage(run,stage,report_path=None,step_q=.015):
    cs,dt=load(run,stage)  # fuente geométrica D8 certificada
    if report_path is not None:
        r=_load_json(report_path)
        from d9_offline_retime import active
        if abs(float(r['original_dt_s'])-dt)>1e-8 or list(r['active_pieces']) != [active(c) for c in cs]:
            raise ValueError(f'{stage}: reporte D9.9 no corresponde al protocolo D8')
    else:
        r=solve(cs,dt)
    if r.get('status')!='CERTIFIED_OFFLINE_COLLECTIVE':
        raise ValueError(f'{stage}: D9.9 no certificado: {r.get("status")}')
    z=sample_solution(cs,dt,r,q_step=step_q)
    return r,z,dt


def original_profiles(z,dt,i):
    """Inversión analítica de la regla de la cadena para el reloj original t=q*dt."""
    s=z['scale_common']
    ds=z['scale_derivative_q']
    v=z[f'R{i}_velocity_xy']
    a=z[f'R{i}_acceleration_xy']
    vb=v*s[:,None]
    ab=a*s[:,None]**2+v*ds[:,None]/dt
    return vb,ab
