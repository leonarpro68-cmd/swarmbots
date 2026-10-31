#!/usr/bin/env python3
"""Adaptador de delivery1 que reutiliza D9.9.7 y D9.9.13 originales. Sin movimiento por defecto."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

EXPECTED_SHA='500234f35039471f15f42801d3c76ebc5686cbe4c480fac6eda98c142fdec0e3'
EXPECTED_STARTS=[(-13.75,10.0),(12.5,-2.5),(-12.5,-7.5)]
EXPECTED_ENDS=[(5.,-10.),(-2.5,-1.25),(-5.,15.)]

def validate(ws):
    import numpy as np
    data=ws/'scripts/supermarket_v3/data'
    npz=data/'delivery1_supermarket_seed100_v3.npz'
    clock=data/'delivery1_supermarket_seed100_clock.json'
    for f in (npz,clock,ws/'scripts/d997_initial_alignment_path.py',ws/'scripts/d9913_stage_coordinator.py'):
        if not f.is_file():raise RuntimeError(f'No existe: {f}')
    digest=hashlib.sha256(npz.read_bytes()).hexdigest()
    if digest!=EXPECTED_SHA:raise RuntimeError('NPZ delivery1 no coincide con SHA256 esperado')
    j=json.loads(clock.read_text())
    if j.get('input_sha256')!=digest or j.get('seed')!=100 or j.get('stage')!='delivery1' or j.get('status')!='OFFLINE_SCHEDULE_CERTIFIED_NOT_EXECUTABLE':
        raise RuntimeError('Reloj V3 incompatible')
    if not (0<j.get('duration_successive_s',0)<j.get('duration_original_s',0)):
        raise RuntimeError('Duración del reloj inválida')
    with np.load(npz,allow_pickle=False) as a:
        for i in range(3):
            cps=a[f'R{i}_controls']; durations=a[f'R{i}_durations_s']
            if cps.ndim!=3 or cps.shape[1:]!=(6,2) or not np.isfinite(cps).all() or len(durations)!=len(cps) or not np.isfinite(durations).all() or not (durations>0).all():
                raise RuntimeError(f'Summit {i}: controles/duraciones inválidos')
            if math.dist(cps[0,0],EXPECTED_STARTS[i])>1e-6 or math.dist(cps[-1,-1],EXPECTED_ENDS[i])>1e-6:
                raise RuntimeError(f'Summit {i}: posiciones no corresponden al delivery1 validado')
    return npz,clock,digest,j

def run(cmd):
    print('Ejecutando:', ' '.join(map(str,cmd)),flush=True)
    return subprocess.run(list(map(str,cmd)),check=False).returncode

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--workspace',type=Path,default=Path.home()/'swarm_ws')
    p.add_argument('--mode',choices=['check','alignment-dry','alignment-execute','coordinator-dry','coordinator-execute'],default='check')
    p.add_argument('--confirm-physical-pickups',action='store_true',help='Confirmar que los 3 agarres se verificaron físicamente en Gazebo')
    p.add_argument('--confirm-ready',action='store_true',help='Confirmar preflight y supervisión de emergencia previos a ejecución')
    args=p.parse_args()
    try:
        ws=args.workspace.resolve(); npz,clock,digest,j=validate(ws)
        print(json.dumps({'stage':'delivery1','seed':100,'sha256':digest,'starts':EXPECTED_STARTS,'ends':EXPECTED_ENDS,'duration_v3_s':j['duration_successive_s'],'mode':args.mode},indent=2),flush=True)
        if args.mode=='check':return 0
        if not args.confirm_physical_pickups:raise RuntimeError('Confirmación física de los TRES agarres requerida (--confirm-physical-pickups)')
        out=ws/'scripts/resultados';out.mkdir(parents=True,exist_ok=True)
        if args.mode.startswith('alignment'):
            active=args.mode=='alignment-execute'
            if active and not args.confirm_ready:raise RuntimeError('Alineación activa requiere --confirm-ready')
            report=out/('supermarket_delivery1_alignment_REAL.json' if active else 'supermarket_delivery1_alignment_DRY.json')
            cmd=[sys.executable,ws/'scripts/d997_initial_alignment_path.py','--trajectory',npz,'--output',report,'--position-tolerance','0.03','--max-offset','0.30','--stationary-speed','0.03','--settle-seconds','1.0','--timeout','30.0']
            if active:cmd.append('--execute')
            code=run(cmd)
            if code:return code
            d=json.loads(report.read_text()); allowed={'ALIGNED','ALREADY_ALIGNED'} if active else {'DRY_RUN_READY','ALREADY_ALIGNED'}
            return 0 if d.get('status') in allowed and not d.get('problems') else 2
        active=args.mode=='coordinator-execute'
        if active:
            if not args.confirm_ready:raise RuntimeError('Ejecución de navegación requiere --confirm-ready')
            ar=out/'supermarket_delivery1_alignment_REAL.json'
            if not ar.exists():raise RuntimeError('Falta el reporte de alineación REAL de delivery1')
            ad=json.loads(ar.read_text());
            if ad.get('status') not in ('ALIGNED','ALREADY_ALIGNED') or ad.get('problems'):
                raise RuntimeError('No se confirmó alineación real correcta')
        report=out/('supermarket_delivery1_REAL.json' if active else 'supermarket_delivery1_DRY.json')
        cmd=[sys.executable,ws/'scripts/d9913_stage_coordinator.py','--stage','delivery1','--trajectory',npz,'--successive-clock',clock,'--expected-sha256',digest,'--nav-speed-limit','2.0','--nav-accel-limit','2.0','--observe-seconds','8','--start-delay','15','--output',report]
        if active:cmd.extend(['--execute','--confirm-runtime-v2'])
        return run(cmd)
    except (RuntimeError,OSError,ValueError,KeyError,ImportError) as e:
        print('BLOQUEADO:',e,file=sys.stderr);return 2

if __name__=='__main__':raise SystemExit(main())
