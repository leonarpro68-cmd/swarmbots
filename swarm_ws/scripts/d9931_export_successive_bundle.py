#!/usr/bin/env python3
"""D9.9.31: paquete de fuente Bezier + relojes sucesivos V3, SOLO OFFLINE.

Entrada: biblioteca D9930 y d9925/manifest.json recien generado para el
mismo escenario y asignacion. Salida: cuatro NPZ con DURACIONES VIRTUALES
sin alterar (NO son tiempos fisicos), relojes certificados y manifest hashes.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from d9929_sucesivo_offline import optimize_stage,sha,_write_json,STAGES

MODEL='common_collective_successive_v1'
STATUS='OFFLINE_V3_BUNDLE_VERIFIED'


def run(scenario,library,source,out,tolerance=.05):
    scenario=Path(scenario).resolve();library=Path(library).resolve()
    source=Path(source).resolve();out=Path(out).resolve()
    if out.exists() and any(out.iterdir()):raise ValueError('OUTPUT_NOT_EMPTY')
    s=json.loads(scenario.read_text());seed=s['seed'];scenario_sha=sha(scenario)
    best_file=library/f'seed_{seed}'/'best_assignment.json';best=json.loads(best_file.read_text())
    if (best['seed']!=seed or best['scenario_sha256']!=scenario_sha or
        best.get('status')!='CERTIFIED' or best.get('time_model')!=MODEL):
        raise ValueError('LIBRARY_SCENARIO_OR_TEMPORAL_MODEL_MISMATCH')
    saved=best['assignment']
    src_manifest_path=source/'manifest.json';src=json.loads(src_manifest_path.read_text())
    if (src.get('seed')!=seed or src.get('scenario_sha256')!=scenario_sha or
        src.get('status')!='CERTIFIED_OFFLINE_ONLY' or src.get('assignment')!=saved):
        raise ValueError('EXPORT_D9925_SOURCE_NOT_MATCHING_LIBRARY')
    params=best.get('time_model_parameters',{})
    scale=float(params['max_scale']);tau=float(params['transition_tau'])
    out.mkdir(parents=True,exist_ok=True)
    manifest={'schema':'swarm.d9931.successive_bundle.v1','status':'RUNNING',
              'execution_authorized':False,'seed':seed,'scenario_sha256':scenario_sha,
              'assignment':saved,'library_best_sha256':sha(best_file),
              'source_manifest_sha256':sha(src_manifest_path),'time_model':MODEL,
              'optimization_speed_m_s':2.,'optimization_accel_m_s2':2.,
              'time_model_parameters':params,'stages':{}}
    _write_json(out/'manifest.json',manifest)
    try:
        for stage in STAGES:
            meta=src['stages'][stage]
            filename=meta['npz']
            if filename!=Path(filename).name or sha(source/filename)!=meta['sha256']:
                raise ValueError(f'{stage}: SOURCE_SHA_MISMATCH')
            arrays={};lengths=[]
            with np.load(source/filename,allow_pickle=False) as z:
                dt=float(z['dt_s']);ref_dt=float(meta['dt_s'])
                if not math.isfinite(dt) or dt<=0 or abs(dt-ref_dt)>1e-8:
                    raise ValueError(f'{stage}: SOURCE_DT_INVALID')
                for i in range(3):
                    points=np.asarray(z[f'R{i}'],dtype=float)
                    if points.ndim!=3 or points.shape[1:]!=(6,2) or len(points)<1:
                        raise ValueError(f'{stage}: INVALID_CONTROLS')
                    if not np.isfinite(points).all():raise ValueError(f'{stage}: NOT_FINITE')
                    arrays[f'R{i}_controls']=points
                    arrays[f'R{i}_durations_s']=np.full(len(points),dt)
                    lengths.append(len(points))
            if len(set(lengths))!=1:raise ValueError(f'{stage}: NOT_COMMON_TIMELINE')
            filename=f'{stage}_MILP_SUCCESSIVE_SOURCE_VIRTUAL.npz'
            np.savez_compressed(out/filename,**arrays)
            clock=optimize_stage(out/filename,max_speed=2.,max_accel=2.,
                                 max_scale=scale,transition_tau=tau)
            estimated=float(best['stage_costs'][stage] if 'stage_costs' in best else
                            best['stages'][stage]['duration_bezier_s'])
            difference=abs(clock['duration_successive_s']-estimated)
            if difference>tolerance:
                raise ValueError(f'{stage}: REPLAN_COST_MISMATCH actual={clock["duration_successive_s"]:.6f} '
                                 f'vs MILP={estimated:.6f} difference={difference:.6f}s')
            clock_file=f'{stage}_clock.json'
            _write_json(out/clock_file,clock)
            manifest['stages'][stage]={
                'status':'CERTIFIED_OFFLINE_V3', 'npz':filename, 'sha256':sha(out/filename),
                'clock':clock_file,'clock_sha256':sha(out/clock_file),
                'duration_original_s':clock['duration_original_s'],
                'duration_successive_s':clock['duration_successive_s'],
                'duration_library_expected_s':estimated,'cost_difference_s':difference,
                'certified_envelopes':clock['certified_envelopes']}
            _write_json(out/'manifest.json',manifest)
            print(f'{stage}: {clock["duration_original_s"]:.3f}s virtual -> '
                  f'{clock["duration_successive_s"]:.3f}s sucesivo; ΔMILP {difference:.4f}s',flush=True)
        manifest['status']=STATUS
        _write_json(out/'manifest.json',manifest)
        print('BUNDLE OFFLINE VERIFICADO:',out,flush=True)
        return manifest
    except BaseException:
        manifest['status']='INCOMPLETE'
        _write_json(out/'manifest.json',manifest)
        raise


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scenario',type=Path,required=True)
    ap.add_argument('--library-dir',type=Path,default=Path('scripts/milp_library'))
    ap.add_argument('--source',type=Path,required=True,help='Directorio exportado por D9925')
    ap.add_argument('--output-dir',type=Path,required=True)
    ap.add_argument('--max-cost-difference-s',type=float,default=.05)
    a=ap.parse_args()
    run(a.scenario,a.library_dir,a.source,a.output_dir,a.max_cost_difference_s)

if __name__=='__main__':main()
