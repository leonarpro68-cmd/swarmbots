#!/usr/bin/env python3
"""Biblioteca D9.9.24: registro offline de la mejor asignación por semilla.

No cambia rutas ni actúa sobre ROS/Gazebo. Modo 'save' recibe el informe D9.9.23
con cálculo a 2.0/2.0; modo 'lookup' verifica identidad del escenario.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
import os
import tempfile
from datetime import datetime, timezone
from pathlib import Path

SCHEMA = 'swarm.milp_library.v1'
TIME_MODEL = 'common_collective_successive_v1'
PARAM_SPEED = PARAM_ACCEL = 2.0

def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))

def atomic_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix='.' + path.name + '.', dir=str(path.parent))
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as out:
            json.dump(data, out, indent=2, ensure_ascii=False)
            out.write('\n')
            out.flush()
            os.fsync(out.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)

def atomic_bytes(path, blob):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix='.' + path.name + '.', dir=str(path.parent))
    try:
        with os.fdopen(fd, 'wb') as out:
            out.write(blob)
            out.flush()
            os.fsync(out.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)

def scenario_info(path):
    data_bytes = Path(path).read_bytes()
    obj = json.loads(data_bytes)
    if not isinstance(obj, dict):
        raise ValueError('scenario.json no es objeto JSON')
    seed = obj.get('seed')
    if not isinstance(seed, int) or isinstance(seed, bool) or seed < 0:
        raise ValueError('scenario.json no contiene seed entero válido')
    return seed, hashlib.sha256(data_bytes).hexdigest(), data_bytes

def verify_assignment(best):
    if not isinstance(best, dict):
        raise ValueError('best no es un objeto')
    first = best.get('pickup1')
    second = best.get('pickup2')
    if not (isinstance(first, list) and isinstance(second, list) and len(first) == len(second) == 3):
        raise ValueError('Asignación debe incluir 3 robots en cada ronda')
    names = first + second
    if not all(isinstance(n, str) and n.startswith('trash_') for n in names):
        raise ValueError('Nombres de basura inválidos')
    if len(set(names)) != 6:
        raise ValueError('Se esperan seis basuras diferentes')
    if any(best.get('stages', {}).get(stage, {}).get('status') != 'CERTIFIED'
           for stage in ('pickup1', 'delivery1', 'pickup2', 'delivery2')):
        raise ValueError('Las cuatro etapas deben estar CERTIFIED')
    total = float(best['estimated_total_s'])
    if not math.isfinite(total) or total <= 0:
        raise ValueError('Costo temporal inválido')
    return {'summit0': [first[0], second[0]],
            'summit1': [first[1], second[1]],
            'summit2': [first[2], second[2]]}, total

def save(args):
    seed, sha, blob = scenario_info(args.scenario)
    source = read_json(args.report)
    if source.get('schema') != 'swarm.d9923.offline_milp_bezier.v1':
        raise ValueError('Informe fuente distinto a D9.9.23')
    if source.get('seed') != seed:
        raise ValueError('Semilla del informe y escenario no coinciden')
    params = source['params']
    if not (math.isclose(float(params['max_speed']), 2.0, abs_tol=1e-8)
            and math.isclose(float(params['max_accel']), 2.0, abs_tol=1e-8)):
        raise ValueError('Biblioteca solo admite optimización offline a 2.0 m/s y 2.0 m/s²')
    if source.get('time_model') != TIME_MODEL or source.get('execution_authorized') is not False:
        raise ValueError('Informe fuente no contiene el modelo temporal sucesivo OFFLINE esperado')
    settings=source.get('time_model_parameters', {})
    tparams = {k:float(settings[k]) for k in ('max_scale','transition_tau')}
    if tparams['max_scale'] < 1 or tparams['transition_tau'] <= 0 or not all(map(math.isfinite,tparams.values())):
        raise ValueError('Parámetros D9929 inválidos')
    best = source.get('best')
    assignment, cost = verify_assignment(best)
    folder = Path(args.library_dir) / f'seed_{seed}'
    current_file = folder / 'best_assignment.json'
    current = read_json(current_file) if current_file.exists() else None
    if current is not None:
        if current.get('seed') != seed or current.get('scenario_sha256') != sha:
            raise ValueError('COLISION_SEMILLA_ESCENARIO: mismo seed con escenario distinto')
        old_model = current.get('time_model')
        if old_model != TIME_MODEL:
            if not args.replace_legacy:
                raise ValueError('TIME_MODEL_MISMATCH: biblioteca histórica; utiliza --replace-legacy explícitamente')
            # Se conserva en history.jsonl; su costo antiguo jamás se compara con el sucesivo.
            current_is_comparable = False
        else:
            current_is_comparable = True
        old_p = current.get('optimization_parameters', {})
        for k,v in [('max_speed_m_s',2.0),('max_accel_m_s2',2.0),
                    ('pickup_wave_s',float(params['pickup_s'])),
                    ('delivery_wave_s',float(params['delivery_s']))]:
            if current_is_comparable and not math.isclose(float(old_p[k]),v,abs_tol=1e-8):
                raise ValueError('Parámetros de evaluación diferentes: costos no comparables')
        if current_is_comparable and current.get('time_model_parameters') != tparams:
            raise ValueError('Parámetros de reloj diferentes: costos no comparables')
        if current_is_comparable and cost >= float(current['estimated_total_s']) - args.epsilon:
            print(json.dumps({'status':'UNCHANGED', 'seed':seed,
                              'stored_cost_s':current['estimated_total_s'],
                              'candidate_cost_s':cost}, indent=2))
            return 0
    record = {'schema':SCHEMA,'seed':seed,'scenario_file':'scenario.json',
              'scenario_sha256':sha,'optimization_parameters':{
                'max_speed_m_s':2.0,'max_accel_m_s2':2.0,
                'pickup_wave_s':float(params['pickup_s']),
                'delivery_wave_s':float(params['delivery_s'])},
              'time_model':TIME_MODEL,'time_model_parameters':tparams,
              'execution_authorized':False,
              'assignment':assignment,'estimated_total_s':cost,
              'navigation_s':float(best['navigation_s']),
              'physical_s':float(best['physical_s']),
              'waves_delivery':list(best['waves_delivery']),
              'stages':best['stages'], 'status':'CERTIFIED',
              'global_optimality_proven':False,
              'evaluated_certified_candidates':int(source.get('milp',{}).get('candidates_certified',0)),
              'updated_utc':datetime.now(timezone.utc).isoformat()}
    folder.mkdir(parents=True, exist_ok=True)
    # Guardar historial antes de cambiar la mejor solución.
    with (folder / 'history.jsonl').open('a', encoding='utf-8') as out:
        out.write(json.dumps({'event':'REPLACED' if current else 'INITIAL',
                              'prior':current, 'new':record},ensure_ascii=False)+'\n')
        out.flush()
        os.fsync(out.fileno())
    atomic_bytes(folder / 'scenario.json', blob)
    atomic_json(current_file, record)
    print(json.dumps({'status':'UPDATED' if current else 'CREATED','seed':seed,
                      'cost_s':cost,'file':str(current_file)}, indent=2))
    return 0

def lookup(args):
    seed, sha, _ = scenario_info(args.scenario)
    path = Path(args.library_dir) / f'seed_{seed}' / 'best_assignment.json'
    if not path.exists():
        raise ValueError(f'MILP_ASSIGNMENT_NOT_FOUND: seed={seed}, path={path}')
    data=read_json(path)
    if data.get('schema')!=SCHEMA or data.get('seed')!=seed or data.get('scenario_sha256')!=sha:
        raise ValueError('MILP_ASSIGNMENT_SCENARIO_MISMATCH')
    # Corroborar que las basuras guardadas corresponden al escenario (usando el
    # propio cargador de planificación, si está disponible).
    names = sum((list(v) for v in data['assignment'].values()), [])
    if len(names)!=6 or len(set(names))!=6 or set(data['assignment'])!= {'summit0','summit1','summit2'}:
        raise ValueError('MILP_ASSIGNMENT_INVALID')
    try:
        from swarm_behavior.swarm_planning.scenario import load
        from swarm_behavior.swarm_planning.bridge import PlanningBridge
        obj=load(Path(args.scenario))
        pending=set(PlanningBridge(obj).pending)
        if set(names)!=pending:
            raise ValueError('MILP_ASSIGNMENT_TRASH_MISMATCH')
    except ImportError:
        pass
    print(json.dumps({'status':'FOUND','seed':seed,'assignment':data['assignment'],
                      'time_model':data.get('time_model'), 'execution_authorized':data.get('execution_authorized'),
                      'estimated_total_s_at_2_0':data['estimated_total_s']},
                     ensure_ascii=False,indent=2))
    return 0

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    sub=ap.add_subparsers(dest='action',required=True)
    for cmd in ('save','lookup'):
        p=sub.add_parser(cmd)
        p.add_argument('--scenario',required=True,type=Path)
        p.add_argument('--library-dir',type=Path,default=Path('scripts/milp_library'))
        if cmd=='save':
            p.add_argument('--replace-legacy',action='store_true',
                           help='Reemplaza la solución de modelo anterior, conservando historial')
            p.add_argument('--report',required=True,type=Path)
            p.add_argument('--epsilon',type=float,default=0.01)
    args=ap.parse_args()
    try:
        if args.action=='save':return save(args)
        return lookup(args)
    except (OSError, KeyError, ValueError, TypeError, json.JSONDecodeError) as exc:
        ap.exit(2,f'ERROR: {exc}\n')

if __name__=='__main__':raise SystemExit(main())
