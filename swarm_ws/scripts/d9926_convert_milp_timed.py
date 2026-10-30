#!/usr/bin/env python3
"""D9.9.26: convierte controles MILP C2 certificados en NPZ temporizados D9.9.13.

Solo conversión offline, no autoriza ni inicia movimiento. Mantiene el DT
certificado por pieza y la sincronización temporal de los tres robots.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import numpy as np

STAGES=('pickup1','delivery1','pickup2','delivery2')

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def write_json(path, obj):
    temp=path.with_suffix('.tmp')
    temp.write_text(json.dumps(obj,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    os.replace(temp,path)

def convert(args):
    root=args.source.resolve()
    manifest_path=root/'manifest.json'
    manifest=json.loads(manifest_path.read_text(encoding='utf-8'))
    if manifest.get('status')!='CERTIFIED_OFFLINE_ONLY' or manifest.get('execution_authorized') is not False:
        raise ValueError('Se requiere manifest CERTIFIED_OFFLINE_ONLY y execution_authorized=false')
    if set(manifest.get('stages',{}))!=set(STAGES):
        raise ValueError('Faltan etapas certificadas')
    if float(manifest.get('max_speed_m_s',-1))!=2 or float(manifest.get('max_accel_m_s2',-1))!=2:
        raise ValueError('Parámetros offline distintos de 2.0/2.0')
    output=args.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise FileExistsError('Directorio de salida no vacío; no se sobrescriben artefactos')
    output.mkdir(parents=True,exist_ok=True)
    report={'schema':'swarm.d9926.timed_conversion.v1','status':'IN_PROGRESS',
            'seed':manifest['seed'],'scenario_sha256':manifest['scenario_sha256'],
            'assignment':manifest['assignment'],'source_manifest_sha256':sha(manifest_path),
            'optimization_limits':{'speed_m_s':2.,'accel_m_s2':2.},
            'execution_authorized':False,'stages':{}}
    write_json(output/'manifest.json',report)
    for stage in STAGES:
        entry=manifest['stages'][stage]
        if entry.get('status')!='CERTIFIED':raise ValueError(f'{stage}: no certificado')
        name=entry.get('npz')
        if not isinstance(name,str) or Path(name).name!=name:raise ValueError(f'{stage}: nombre de NPZ inválido')
        source=root/name
        if sha(source)!=entry.get('sha256'):raise ValueError(f'{stage}: SHA256 inválido')
        dt=float(entry['dt_s']); duration=float(entry['duration_s'])
        if not math.isfinite(dt) or dt<=0 or not math.isfinite(duration) or duration<=0:
            raise ValueError(f'{stage}: tiempos inválidos')
        arrays={}; lengths=[]; vpeak=0.; apeak=0.
        with np.load(source,allow_pickle=False) as z:
            if not math.isclose(float(z['dt_s']),dt,rel_tol=0,abs_tol=1e-8):
                raise ValueError(f'{stage}: dt_s no coincide')
            for i in range(3):
                c=np.asarray(z[f'R{i}'],dtype=float)
                if c.ndim!=3 or c.shape[1:]!=(6,2) or len(c)<1 or not np.isfinite(c).all():
                    raise ValueError(f'{stage}: R{i} tiene controles inválidos')
                lengths.append(len(c))
                vel=5*np.diff(c,axis=1)/dt
                acc=20*np.diff(c,n=2,axis=1)/(dt*dt)
                vpeak=max(vpeak,float(np.linalg.norm(vel,axis=2).max()))
                apeak=max(apeak,float(np.linalg.norm(acc,axis=2).max()))
                arrays[f'R{i}_controls']=c
                arrays[f'R{i}_durations_s']=np.full(len(c),dt,dtype=float)
        if len(set(lengths))!=1 or not math.isclose(lengths[0]*dt,duration,rel_tol=1e-5,abs_tol=1e-3):
            raise ValueError(f'{stage}: horizonte compartido o duración no consistente: {lengths}, {dt}, {duration}')
        # Límites sobre la envolvente convexa; cada componente temporal es una Bézier quíntica.
        if vpeak>2.0+1e-5 or apeak>2.0+1e-5:
            raise ValueError(f'{stage}: cota dinámica conservadora excedida: v={vpeak:.4f}, a={apeak:.4f}')
        target=output/f'{stage}_MILP_TIMED_VALIDATED_OFFLINE.npz'
        np.savez_compressed(target,**arrays)
        with np.load(target,allow_pickle=False) as chk:
            for k,v in arrays.items():
                if not np.array_equal(chk[k],v):raise ValueError(f'{stage}: roundtrip NPZ falla en {k}')
        report['stages'][stage]={'source_sha256':entry['sha256'],'npz':target.name,
                                 'sha256':sha(target),'pieces':lengths[0], 'dt_s':dt,
                                 'duration_s':duration,'velocity_bound_m_s':vpeak,
                                 'acceleration_bound_m_s2':apeak,'format':'D9913_FIELDS'}
        write_json(output/'manifest.json',report)
        print(f'{stage}: {lengths[0]} piezas, {duration:.3f}s, v≤{vpeak:.3f}, a≤{apeak:.3f}',flush=True)
    report['status']='OFFLINE_TIMED_FORMAT_VERIFIED'
    write_json(output/'manifest.json',report)
    print('Conversión terminada. NPZ NO autorizados aún para misión física:',output)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True,help='Directorio de salida de d9925 con manifest.json')
    p.add_argument('--output-dir',type=Path,required=True)
    a=p.parse_args();convert(a)

if __name__=='__main__':main()
