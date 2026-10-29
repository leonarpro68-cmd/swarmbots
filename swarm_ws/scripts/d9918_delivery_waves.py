#!/usr/bin/env python3
"""D9.9.18: oleadas de entrega D8 sobre navegación D9.9, conservador.

Por defecto es SOLO LECTURA. En modo --execute-wave ejecuta UNA oleada,
ejecutando en paralelo los robots de la oleada activa.
Nunca inicia dos robots con el mismo depósito en una oleada. Requiere evidencia explícita de carga y del estado de depósitos.
La segunda oleada espera wave1 y verifica que los robots permanecen en nodos seguros. No solicita despeje.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
import os
import subprocess
import signal
import time
import sys
from datetime import datetime, timezone
from pathlib import Path

ROBOTS = ('summit0','summit1','summit2')
ACK = ('DETACH CONFIRMADO','REGRESO CONFIRMADO')

def write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False)+'\n')
    os.replace(tmp,path)

def mapping(scenario, ledger, stage):
    names=ledger['round1_names' if stage=='delivery1' else 'round2_names']
    items={x['name']:x for x in scenario['elements']}
    if len(names)!=3 or len(set(names))!=3 or any(n not in items or items[n]['type']!='trash' for n in names):
        raise RuntimeError('Asignación histórica inválida')
    return {r:{'trash':n,'color':items[n]['color'],'deposit':'deposit_'+items[n]['color']} for r,n in zip(ROBOTS,names)}

def waves_from_mapping(mapping):
    # Se conserva el criterio D7: primero colores únicos, después colores compartidos.
    from collections import Counter
    counts=Counter(m['deposit'] for m in mapping.values())
    pending=sorted(ROBOTS,key=lambda r:(counts[mapping[r]['deposit']],r))
    waves=[]
    while pending:
        used=set(); wave=[]
        for r in pending:
            d=mapping[r]['deposit']
            if d not in used:
                used.add(d); wave.append(r)
        waves.append(wave)
        pending=[r for r in pending if r not in wave]
    return waves

def preflight_geometry(mapping, waves, scenario, poses, clearance):
    # Base geométrica D7: aproxima cada robot a 0.8 m del depósito; NO sustituye CBS.
    items={e['name']:e for e in scenario['elements']}
    endpoints={}
    active = set().union(*map(set, waves))
    for r in active:
        sx,sy=poses[r]
        dx,dy=items[mapping[r]['deposit']]['xy_m']
        dist=math.hypot(dx-sx,dy-sy)
        if not 0.90<=dist<=1.8:
            raise RuntimeError(f'{r}: distancia al depósito {dist:.3f} m fuera de [0.90, 1.80]')
        k=(dist-0.8)/dist
        endpoints[r]=[sx+k*(dx-sx),sy+k*(dy-sy)]
    from d7_parallel_delivery import point_segment,segment_distance
    for wi,wave in enumerate(waves):
        for i,r in enumerate(wave):
            for s in wave[i+1:]:
                if segment_distance(poses[r],endpoints[r],poses[s],endpoints[s])<clearance:
                    raise RuntimeError(f'oleada {wi+1}: maniobras de {r}/{s} se aproximan demasiado')
            for s in ROBOTS:
                if s==r or s in wave: continue
                # Los robots no activos siguen en su posición en este preflight.
                if point_segment(poses[s],poses[r],endpoints[r])<clearance:
                    raise RuntimeError(f'oleada {wi+1}: trayectoria local de {r} cerca de {s} estacionado')
    return endpoints

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--stage',choices=('delivery1','delivery2'),required=True)
    ap.add_argument('--scenario',type=Path,default=Path('scripts/d996_reference/scenario.json'))
    ap.add_argument('--ledger',type=Path,default=Path('scripts/resultados/d8/seed_100_run_007/mission/mision.json'))
    ap.add_argument('--navigation-report',type=Path,required=True)
    ap.add_argument('--pickup-evidence-dir',type=Path,required=True,
                    help='Carpeta con {stage_pickup}_{robot}_execute.json de ESTA sesión; exige ACK de joint')
    ap.add_argument('--world',default='world_demo')
    ap.add_argument('--execute-wave',type=int,choices=(1,2))
    ap.add_argument('--confirm-cargo-inspected',action='store_true',help='Inspección visual de joint/carga actual')
    ap.add_argument('--prior-wave-report',type=Path,
                    help='Informe de wave1 actual para habilitar wave2')
    ap.add_argument('--clearance-m',type=float,default=1.25)
    ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args()
    if a.out.exists():ap.error('No se sobrescribe el reporte de una ejecución')
    report={'schema':'swarm.d9918.delivery_waves.v1','stage':a.stage,'status':'REJECT',
            'execution_requested':a.execute_wave is not None,'wave_requested':a.execute_wave,
            'created_utc':datetime.now(timezone.utc).isoformat(),
            'commands_may_have_been_sent':False,'problems':[],'completed_robots':[]}
    try:
        scenario=json.loads(a.scenario.read_text())
        ledger=json.loads(a.ledger.read_text())
        if scenario.get('seed')!=100:raise RuntimeError('Solo seed 100')
        if ledger.get('scenario_sha256')!=hashlib.sha256(a.scenario.read_bytes()).hexdigest():
            raise RuntimeError('Ledger y escenario no coinciden')
        nav=json.loads(a.navigation_report.read_text())
        if nav.get('status')!='DONE_ALL_'+a.stage.upper() or nav.get('problems'):
            raise RuntimeError('La navegación de entrega D9.9 no se completó')
        assign=mapping(scenario,ledger,a.stage)
        waves=waves_from_mapping(assign)
        report.update({'assignments':assign,'waves':waves,'deposit_exclusivity_validated':True,
                       'global_collision_avoidance_certified':False})
        pickup='pickup1' if a.stage=='delivery1' else 'pickup2'
        evidence={}
        for r in ROBOTS:
            p=a.pickup_evidence_dir/f'{pickup}_{r}_execute.json'
            if not p.exists():raise RuntimeError(f'Falta confirmación del agarre actual: {p}')
            d=json.loads(p.read_text())
            if not (d.get('stage')==pickup and d.get('robot')==r and
                    d.get('trash')==assign[r]['trash'] and
                    d.get('status')=='PHYSICAL_ACK_REQUIRES_GAZEBO_INSPECTION' and
                    'JOINT CONFIRMADO' in d.get('ack_markers',[])):
                raise RuntimeError(f'{r}: evidencia de agarre incompatible/no confirmada')
            evidence[r]=str(p)
        report['pickup_evidence']=evidence
        # El estado físico real tiene que medirse al momento de la operación.
        if a.execute_wave is None:
            report['status']='WAVE_PLAN_READY_NO_COMMANDS'
            report['requires_live_geometry_check']=True
        else:
            if not a.confirm_cargo_inspected:
                raise RuntimeError('Requiere --confirm-cargo-inspected; ACK no certifica la carga real')
            wi=a.execute_wave-1
            if wi>=len(waves):raise RuntimeError('Esa oleada no existe')
            if wi:
                if not a.prior_wave_report:
                    raise RuntimeError('Wave2 requiere informe de wave1 completada')
                p1=json.loads(a.prior_wave_report.read_text())
                if not (p1.get('status')=='WAVE_COMPLETED_ACK_REQUIRES_GAZEBO_INSPECTION' and
                        p1.get('stage')==a.stage and p1.get('wave_requested')==1 and
                        set(p1.get('completed_robots',[]))==set(waves[0])):
                    raise RuntimeError('Wave1 no está acreditada en la misma etapa')
                # Sin maniobra de despeje: nodo de retorno seguro es el nodo
                # histórico del plan. Se conserva comprobación de todos los
                # robots inmóviles y de colisión de la aproximación local.
            from d6_grasp_existing import snapshot_odom
            odom=snapshot_odom()
            # La fuente D6 entrega el formato {robot:{xy:...,speed:...}}, verifique abajo.
            poses={}
            for r in ROBOTS:
                d=odom[r]
                if isinstance(d,dict):
                    xy=d.get('xy',d.get('xy_m'))
                    speed=d.get('speed_m_s',d.get('speed',0.0))
                else:
                    raise RuntimeError(f'Odometría inesperada {r}')
                if xy is None or len(xy)!=2:raise RuntimeError(f'Odometría incompleta {r}')
                poses[r]=tuple(map(float,xy))
                if speed>0.08:raise RuntimeError(f'{r} sigue moviéndose: {speed:.3f} m/s')
            endpoints=preflight_geometry(assign,[waves[wi]],scenario,poses,a.clearance_m)
            report.update({'poses_live':poses,'local_endpoints':endpoints})
            script=Path(__file__).resolve().parent/'prueba_entrega_local_cbs.py'
            if not script.exists():raise RuntimeError('Falta rutina D8 original')
            report['status']='WAVE_EXECUTING'
            write(a.out,report)
            # Una oleada se ejecuta en PARALELO: primero preparar todos los
            # subprocess; esperar a todos antes de pasar al despeje o wave2.
            # Cada oleada tiene a lo sumo un Summit por depósito.
            pending = {}
            logs = {}
            try:
                for r in waves[wi]:
                    cmd=[sys.executable,str(script),'--scenario',str(a.scenario),
                         '--robot',r,'--trash',assign[r]['trash'],
                         '--deposit',assign[r]['deposit'],'--max-start-distance','1.8','--execute']
                    log=a.out.with_name(a.out.stem+'_'+r+'.log')
                    log.parent.mkdir(parents=True,exist_ok=True)
                    fp=log.open('w',encoding='utf-8')
                    logs[r]=fp
                    p=subprocess.Popen(cmd,stdout=fp,stderr=subprocess.STDOUT,
                                       start_new_session=True)
                    pending[r]=(p,time.monotonic(),log)
                    report.setdefault('robot_logs',{})[r]=str(log)
                    report['commands_may_have_been_sent']=True
                    report['parallel_wave']=True
                    write(a.out,report)
                # No arrancar la siguiente oleada hasta terminar todos.
                while pending:
                    for r in list(pending):
                        proc,beg,log=pending[r]
                        code=proc.poll()
                        if code is None:
                            if time.monotonic()-beg>115.0:
                                raise RuntimeError(f'{r}: timeout de entrega (115 s); revisar {log}')
                            continue
                        del pending[r]
                        logs[r].flush()
                        stdout=log.read_text(encoding='utf-8',errors='replace')
                        if code or not all(marker in stdout for marker in ACK):
                            raise RuntimeError(f'{r}: exit={code} / faltan ACK. Revisar {log}; no reintentar a ciegas')
                        report['completed_robots'].append(r)
                        write(a.out,report)
                    if pending: time.sleep(0.1)
            finally:
                # Interrumpir solo operaciones físicas propias que sigan vivas.
                for r,(proc,_,_) in pending.items():
                    if proc.poll() is None:
                        try: os.killpg(proc.pid,signal.SIGINT)
                        except ProcessLookupError: pass
                for r,(proc,_,_) in pending.items():
                    try: proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        try: os.killpg(proc.pid,signal.SIGTERM)
                        except ProcessLookupError: pass
                for fp in logs.values(): fp.close()
            report['status']='WAVE_COMPLETED_ACK_REQUIRES_GAZEBO_INSPECTION'
    except Exception as e:
        report['problems'].append(f'{type(e).__name__}: {e}')
        report['status']='REJECT_STOPPED'
    write(a.out,report)
    print(json.dumps(report,indent=2,ensure_ascii=False))
    return 0 if report['status'] in ('WAVE_PLAN_READY_NO_COMMANDS','WAVE_COMPLETED_ACK_REQUIRES_GAZEBO_INSPECTION') else 2

if __name__=='__main__':raise SystemExit(main())
