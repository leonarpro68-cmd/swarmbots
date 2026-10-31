#!/usr/bin/env python3
"""Adaptador de manipulación D8 original para pickup2 y delivery2 (un Summit por vez)."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

ROBOTS=('summit0','summit1','summit2')
SHA={
 'pickup2':'7f05c330ef25bb2c0b7bb84c3b1acb4661134919c39480d39cf22cf59435ed91',
 'delivery2':'33aff6468eea915cc8f3e6da3d3df896392ca2fc996737bde7c02918dba65b70',
}
DELIVERY_GOALS={ 'summit0':(5.0,-10.0), 'summit1':(-2.5,15.0), 'summit2':(-2.5,1.25)}
ACK={ 'pickup2':('ALINEAR CONFIRMADO','JOINT CONFIRMADO','REGRESO CONFIRMADO'),
      'delivery2':('DETACH CONFIRMADO','REGRESO CONFIRMADO')}


def save(path, data):
    path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    temp.replace(path)


def prepare(stage, robot, scenario, assignment, nav):
    if scenario.get('seed')!=100 or scenario.get('world')!='supermarket':
        raise RuntimeError('Solo supermarket semilla 100')
    if assignment.get('stage')!='pickup2' or assignment.get('seed')!=100 or assignment.get('world')!='supermarket':
        raise RuntimeError('Asignación pickup2 incorrecta')
    if nav.get('stage')!=stage or nav.get('status')!='DONE_ALL_'+stage.upper() or nav.get('commands_sent')!=3 or nav.get('problems') or nav.get('execution_requested') is not True or nav.get('trajectory_sha256')!=SHA[stage]:
        raise RuntimeError('No existe evidencia válida de navegación '+stage)
    item=assignment['robots'][robot]
    trash=item['trash']; color=item['color']
    entries={e['name']:e for e in scenario['elements']}
    if trash not in entries or entries[trash].get('type')!='trash' or entries[trash].get('color')!=color:
        raise RuntimeError('Basurita no corresponde al escenario')
    if math.dist(item['trash_xy_m'],entries[trash]['xy_m'])>1e-6:
        raise RuntimeError('Coordenadas basurita distintas al escenario')
    if stage=='pickup2':
        distance=math.dist(item['goal_xy_m'],item['trash_xy_m'])
        if not .55<distance<2.2:raise RuntimeError('Basurita fuera del rango D8')
        return {'trash':trash,'color':color,'planned_approach_xy_m':item['goal_xy_m'],
                'planned_distance_to_trash_m':distance}
    deposit='deposit_'+color
    if deposit not in entries or entries[deposit].get('type')!='deposit' or entries[deposit].get('color')!=color:
        raise RuntimeError('Depósito de color incorrecto')
    distance=math.dist(DELIVERY_GOALS[robot],entries[deposit]['xy_m'])
    if not 0.9<=distance<=1.8:raise RuntimeError('Distancia D8 a depósito no apta')
    return {'trash':trash,'color':color,'deposit':deposit,'planned_approach_xy_m':DELIVERY_GOALS[robot],
            'planned_distance_to_deposit_m':distance}


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--stage',choices=('pickup2','delivery2'),required=True)
    ap.add_argument('--robot',choices=ROBOTS,required=True)
    ap.add_argument('--scenario',type=Path,required=True)
    ap.add_argument('--assignment',type=Path,required=True)
    ap.add_argument('--navigation-report',type=Path,required=True)
    ap.add_argument('--world',default='world_demo',help='Nombre INTERNO de Gazebo; no el nombre del mapa')
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--execute',action='store_true')
    ap.add_argument('--confirm-current-session',action='store_true',help='La navegación y el estado de Gazebo son de la misma sesión')
    ap.add_argument('--confirm-loaded',action='store_true',help='Solo delivery2: basurita observada físicamente sujeta')
    ap.add_argument('--confirm-supervision',action='store_true',help='Gazebo visible, pausa disponible y sin otro ejecutor')
    args=ap.parse_args()
    if args.output.exists():ap.error('El reporte ya existe. No repetir una acción física automáticamente')
    report={'schema':'supermarket.ronda2.physical.original_d8.v1','stage':args.stage,'robot':args.robot,
            'execution_requested':args.execute,'status':'REJECT_STOPPED',
            'commands_may_have_been_sent':False,'problems':[]}
    try:
        s=json.loads(args.scenario.read_text(encoding='utf-8'))
        a=json.loads(args.assignment.read_text(encoding='utf-8'))
        n=json.loads(args.navigation_report.read_text(encoding='utf-8'))
        report.update(prepare(args.stage,args.robot,s,a,n))
        report['navigation_session_id']=n.get('session_id')
        if args.execute and (not args.confirm_current_session or not args.confirm_supervision or (args.stage=='delivery2' and not args.confirm_loaded)):
            raise RuntimeError('Faltan confirmaciones de sesión, supervisión o carga física')
        scripts=Path(__file__).resolve().parents[1]
        if args.stage=='pickup2':
            # Lectura del escenario ROS/Gazebo antes de invocar la rutina D8.
            sys.path.insert(0,str(scripts))
            from d6_grasp_existing import snapshot_odom,gazebo_trash_positions,check_preapproach,scenario_targets
            odom=snapshot_odom()
            poses=gazebo_trash_positions(args.world)
            chk=check_preapproach(args.robot,report['trash'],odom,scenario_targets(s),poses,max_distance=2.2)
            pose=odom[args.robot]['xy']
            err=math.dist(pose,report['planned_approach_xy_m'])
            if err>0.15:raise RuntimeError(f'Summit no está en meta CBS: error {err:.3f} m >0.15 m')
            report['approach_goal_error_m']=err;report['physical_preflight']=chk
            original=scripts/'prueba_alinear_agarrar_cbs.py'
            cmd=[sys.executable,'-u',str(original),'--scenario',str(args.scenario.resolve()),
                 '--robot',args.robot,'--trash',report['trash'],'--max-distance','2.2']
            # En la rutina de pickup original, SIN --dry-run significa EJECUTAR.
            if not args.execute:
                report['status']='PREFLIGHT_READY_NO_COMMANDS'
                save(args.output,report)
                print(json.dumps(report,indent=2,ensure_ascii=False));return 0
        else:
            original=scripts/'prueba_entrega_local_cbs.py'
            cmd=[sys.executable,'-u',str(original),'--scenario',str(args.scenario.resolve()),
                 '--robot',args.robot,'--trash',report['trash'],'--deposit',report['deposit'],
                 '--max-start-distance','1.8']
            if args.execute:cmd.append('--execute')
        if not original.is_file():raise RuntimeError('Falta script original D8: '+str(original))
        if args.execute:
            report['commands_may_have_been_sent']=True
            save(args.output,report)
        else:
            # Solo delivery2; D8 realiza su propio dry-run sin --execute.
            result=subprocess.run(cmd,capture_output=True,text=True,timeout=35)
            log=args.output.with_suffix('.log');log.write_text(result.stdout+result.stderr,encoding='utf-8')
            if result.returncode!=0 or 'DRY RUN: no se ha publicado ninguna orden.' not in result.stdout:
                raise RuntimeError(f'Preflight D8 no apto: {log}')
            report['original_log']=str(log)
            report['status']='PREFLIGHT_READY_NO_COMMANDS'
            save(args.output,report)
            print(json.dumps(report,indent=2,ensure_ascii=False));return 0
        # Para delivery2, una segunda comprobación del preflight nativo antes de mover.
        if args.stage=='delivery2':
            dry=subprocess.run(cmd[:-1],capture_output=True,text=True,timeout=35)
            dry_log=args.output.with_name(args.output.stem+'_preflight.log')
            dry_log.write_text(dry.stdout+dry.stderr,encoding='utf-8')
            report['d8_preflight_log']=str(dry_log)
            if dry.returncode!=0 or 'DRY RUN: no se ha publicado ninguna orden.' not in dry.stdout:
                raise RuntimeError('Rechazo del dry-run original D8; revisar '+str(dry_log))
        result=subprocess.run(cmd,capture_output=True,text=True,timeout=180)
        log=args.output.with_suffix('.log');log.write_text(result.stdout+result.stderr,encoding='utf-8')
        report.update({'original_exit_code':result.returncode,'original_log':str(log)})
        if result.returncode!=0 or not all(x in result.stdout for x in ACK[args.stage]):
            raise RuntimeError(f'Sin ACK completo D8; exit={result.returncode}. Revisar {log}')
        report['ack_markers']=list(ACK[args.stage])
        report['status']='PHYSICAL_ACK_REQUIRES_GAZEBO_INSPECTION'
    except Exception as exc:
        report['status']='REJECT_STOPPED'
        report['problems'].append(f'{type(exc).__name__}: {exc}')
    save(args.output,report)
    print(json.dumps(report,indent=2,ensure_ascii=False))
    return 0 if report['status'] in ('PREFLIGHT_READY_NO_COMMANDS','PHYSICAL_ACK_REQUIRES_GAZEBO_INSPECTION') else 2

if __name__=='__main__':
    raise SystemExit(main())
