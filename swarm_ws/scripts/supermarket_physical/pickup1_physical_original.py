#!/usr/bin/env python3
"""Conservative supermarket pickup1 adapter for the original D8 physical routine.

Default: read-only ROS/Gazebo preflight, never commands. --execute runs ONE
robot through the unmodified prueba_alinear_agarrar_cbs.py. Never auto-retries.
"""
import argparse, hashlib, json, math, subprocess, sys, time
from pathlib import Path

ROBOT_NAMES=('summit0','summit1','summit2')
SUCCESS=('ALINEAR CONFIRMADO','JOINT CONFIRMADO','REGRESO CONFIRMADO')

def write_report(path,data):
    path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n')
    tmp.replace(path)

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--robot',choices=ROBOT_NAMES,required=True)
    ap.add_argument('--execute',action='store_true')
    ap.add_argument('--navigation-report',required=True,type=Path)
    ap.add_argument('--scenario',required=True,type=Path)
    ap.add_argument('--assignment',required=True,type=Path)
    ap.add_argument('--world',default='supermarket')
    ap.add_argument('--output',required=True,type=Path)
    args=ap.parse_args()
    if args.output.exists():ap.error('El reporte ya existe; no ejecutar dos veces sobre un mundo parcialmente modificado')
    data={'schema':'supermarket.pickup1.physical.original_d8.v1','robot':args.robot,'stage':'pickup1',
          'execution_requested':bool(args.execute),'status':'REJECT','problems':[],
          'commands_may_have_been_sent':False}
    try:
        assignment=json.loads(args.assignment.read_text())
        scenario=json.loads(args.scenario.read_text())
        nav=json.loads(args.navigation_report.read_text())
        if scenario.get('world')!='supermarket' or scenario.get('seed')!=100:
            raise RuntimeError('Escenario incorrecto: se requiere supermarket semilla 100')
        if assignment.get('seed')!=100 or assignment.get('world')!='supermarket':
            raise RuntimeError('Asignación de escenario incompatible')
        if nav.get('status')!='DONE_ALL_PICKUP1' or nav.get('problems') or nav.get('commands_sent')!=3:
            raise RuntimeError('No hay evidencia de navegación pickup1 completa para los tres robots')
        if nav.get('trajectory_sha256')!=assignment['navigation_sha256']:
            raise RuntimeError('SHA-256 de navegación no coincide con la misión supermarket')
        expected=assignment['robots'][args.robot]
        trash=expected['trash']
        objects={e['name']:e for e in scenario.get('elements',[]) if e.get('type')=='trash'}
        if trash not in objects or tuple(objects[trash]['xy_m'])!=tuple(expected['trash_xy_m']):
            raise RuntimeError('Basura o identidad incompatible con JSON y planificación')
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
        from d6_grasp_existing import snapshot_odom, gazebo_trash_positions, check_preapproach, scenario_targets
        odom=snapshot_odom()
        positions=gazebo_trash_positions(args.world)
        chk=check_preapproach(args.robot,trash,odom,scenario_targets(scenario),positions,max_distance=2.2)
        pose=odom[args.robot]['xy']
        goal_error=math.dist(pose,expected['goal_xy_m'])
        if goal_error>0.15:
            raise RuntimeError(f'Robot no está en su nodo de aproximación: desvío {goal_error:.3f} m > 0.15 m')
        data.update({'trash':trash,'navigation_session_id':nav.get('session_id'),
                     'approach_goal_error_m':goal_error,'physical_preflight':chk})
        original=Path(__file__).resolve().parents[1]/'prueba_alinear_agarrar_cbs.py'
        if not original.is_file():raise RuntimeError(f'No existe rutina D8 original: {original}')
        if not args.execute:
            data['status']='PREFLIGHT_READY_NO_COMMANDS'
        else:
            # Explicit physical single-robot operation: original D8 owns local turn, RGBD and grasp.
            cmd=[sys.executable,'-u',str(original),'--scenario',str(args.scenario),'--robot',args.robot,
                 '--trash',trash,'--max-distance','2.2']
            data['commands_may_have_been_sent']=True
            write_report(args.output,data)
            result=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,errors='replace')
            log=args.output.with_suffix('.log');log.write_text(result.stdout)
            data.update({'original_exit_code':result.returncode,'original_log':str(log)})
            if result.returncode!=0 or not all(tag in result.stdout for tag in SUCCESS):
                raise RuntimeError(f'La rutina original no confirmó todos los ACK: exit={result.returncode}; revisar {log}')
            data['ack_markers']=list(SUCCESS)
            data['status']='PHYSICAL_ACK_REQUIRES_GAZEBO_INSPECTION'
    except Exception as exc:
        data['problems'].append(str(exc))
        data['status']='REJECT_STOPPED'
    write_report(args.output,data)
    print(json.dumps(data,indent=2,ensure_ascii=False))
    return 0 if data['status'] in ('PREFLIGHT_READY_NO_COMMANDS','PHYSICAL_ACK_REQUIRES_GAZEBO_INSPECTION') else 2
if __name__=='__main__':raise SystemExit(main())
