#!/usr/bin/env python3
"""Preflight físico de D8 sobre ROS/Gazebo activo. Nunca publica órdenes de movimiento."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--prepared',type=Path,required=True,help='Carpeta producida por prepare_phase5.py')
    ap.add_argument('--stage',choices=('pickup1','delivery1','pickup2','delivery2'),required=True)
    ap.add_argument('--robot',choices=('summit0','summit1','summit2'),required=True)
    ap.add_argument('--navigation-report',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--world',default='world_demo')
    ap.add_argument('--confirm-current-session',action='store_true')
    ap.add_argument('--confirm-loaded',action='store_true',help='En delivery: confirmación visual de carga en pinzas')
    a=ap.parse_args()
    if a.output.exists():ap.error('El reporte ya existe, no sobrescribir')
    report={'schema':'supermarket.milp.phase5.live.d8.dry.v1','stage':a.stage,'robot':a.robot,
            'commands_sent':False,'execution_authorized':False,'status':'REJECTED','errors':[]}
    try:
        if not a.confirm_current_session:raise RuntimeError('Se exige confirmar sesión actual de Gazebo')
        if a.stage.startswith('delivery') and not a.confirm_loaded:raise RuntimeError('Se exige verificar físicamente la carga')
        manifest=json.loads((a.prepared/'phase5_static_preflight.json').read_text())
        if manifest['status']!='PASS_STATIC_PRECHECK_NOT_EXECUTABLE':raise RuntimeError('Fase 5 offline no pasó')
        expected=manifest['stages'][a.stage]
        nav=json.loads(a.navigation_report.read_text(encoding='utf-8'))
        if nav.get('stage')!=a.stage or nav.get('status')!='DONE_ALL_'+a.stage.upper() or nav.get('trajectory_sha256')!=expected['sha256'] or nav.get('commands_sent')!=3 or nav.get('problems') or nav.get('execution_requested') is not True:
            raise RuntimeError('Informe de navegación no corresponde al NPZ MILP ni a una ejecución terminada')
        ass=json.loads((a.prepared/f'assignment_{a.stage}_MILP.json').read_text())
        if ass['navigation_sha256'] != expected['sha256'] or not ass.get('execution_authorized') is False:
            raise RuntimeError('Asignación MILP incompatible')
        item=ass['robots'][a.robot]
        index=int(a.robot[-1]);goal=expected['goal_xy_m'][index]
        if math.dist(item['goal_xy_m'],goal)>1e-6:raise RuntimeError('Meta de asignación distinta de la navegación')
        scenario=a.prepared/'scenario_for_original_D8_seed100_MILP.json'
        scripts=Path(__file__).resolve().parents[1]
        sys.path.insert(0,str(scripts))
        from d6_grasp_existing import snapshot_odom, gazebo_trash_positions, check_preapproach, scenario_targets, point_segment_distance
        odom=snapshot_odom()
        actual=odom[a.robot]['xy'];error=math.dist(actual,goal)
        report['goal_error_m']=round(error,5)
        if error>0.12:raise RuntimeError(f'{a.robot}: error en meta CBS {error:.3f} > 0.12m')
        if a.stage.startswith('pickup'):
            observed=gazebo_trash_positions(a.world)
            sc=json.loads(scenario.read_text(encoding='utf-8'))
            report['live_preapproach']=check_preapproach(a.robot,item['trash'],odom,scenario_targets(sc),observed,max_distance=2.2)
            script=scripts/'prueba_alinear_agarrar_cbs.py'
            command=[sys.executable,'-u',str(script),'--scenario',str(scenario),'--robot',a.robot,
                     '--trash',item['trash'],'--max-distance','2.2','--dry-run']
            expected_marker='DRY RUN correcto; no se enviaron órdenes.'
        else:
            d=math.dist(actual,item['target_xy_m']);report['deposit_distance_m']=round(d,5)
            if d<.90 or d>1.80:raise RuntimeError(f'{a.robot}: D8 entrega fuera de rango d={d:.3f}')
            # La maniobra local apunta a una parada a 0.80m del depósito.
            center=item['target_xy_m']
            loc=[center[k]+(actual[k]-center[k])*.80/d for k in (0,1)]
            for other in ('summit0','summit1','summit2'):
                if other==a.robot:continue
                if point_segment_distance(odom[other]['xy'],actual,loc)<1.15:
                    raise RuntimeError(f'{a.robot}: trayectoria de entrega local demasiado cerca de {other}')
            script=scripts/'prueba_entrega_local_cbs.py'
            command=[sys.executable,'-u',str(script),'--scenario',str(scenario),'--robot',a.robot,
                     '--trash',item['trash'],'--deposit',item['target'],'--max-start-distance','1.8']
            expected_marker='DRY RUN: no se ha publicado ninguna orden.'
        if not script.is_file():raise FileNotFoundError(script)
        proc=subprocess.run(command,capture_output=True,text=True,timeout=40)
        log=a.output.with_suffix('.log');log.parent.mkdir(exist_ok=True,parents=True)
        if log.exists():raise FileExistsError(log)
        log.write_text(proc.stdout+proc.stderr,encoding='utf-8')
        report['original_d8_log']=str(log)
        report['original_exit_code']=proc.returncode
        if proc.returncode!=0 or expected_marker not in proc.stdout:
            raise RuntimeError(f'D8 original no pasó dry-run, consultar {log}')
        report['status']='PASS_LIVE_D8_DRY_ONLY_NO_COMMANDS'
    except Exception as exc:
        report['errors'].append(f'{type(exc).__name__}: {exc}')
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2,ensure_ascii=False))
    return 0 if not report['errors'] else 2

if __name__=='__main__':sys.exit(main())
