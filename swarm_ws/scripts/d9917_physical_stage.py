#!/usr/bin/env python3
"""D9.9.17: puente conservador entre navegación D9.9 y agarre/entrega D8.

No publica comandos ROS, no altera CBS / Hybrid V2 / D8. Por defecto es
preflight de solo lectura. En modo --execute invoca la rutina D8 original
para UN robot, con evidencia de la etapa D9.9 y asignación histórica.
"""
import argparse
import hashlib
import json
import math
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

ROBOTS=('summit0','summit1','summit2')
STAGES=('pickup1','delivery1','pickup2','delivery2')
SHA={
 'pickup1':'3235223232eb1b47e36c46cc107086c933e5020917aa7b78758b9b183985e2dd',
 'delivery1':'70bb3550dfb3a937f002f5b8e83d621ef8b229180f6a0b8c6bdc626de19fb40b',
 'pickup2':'7f626f10002aeb40be8fd3684e8cc2ee49512ffbf01fdc969f489584c1f94e11',
 'delivery2':'ce13db58df80acdfc4bedebfc115c3a37a2bc5dad501727bfa9812ea2d1b2b60',
}

def atomic_json(p,obj):
 p.parent.mkdir(parents=True,exist_ok=True)
 tmp=p.with_suffix('.tmp');tmp.write_text(json.dumps(obj,indent=2,ensure_ascii=False)+'\n');tmp.replace(p)

def mapping(stage, mission, scenario):
 if stage not in STAGES:raise ValueError('Etapa desconocida')
 names=mission['round1_names' if stage.endswith('1') else 'round2_names']
 if not isinstance(names,list) or len(names)!=3 or len(set(names))!=3:raise ValueError('No hay asignación histórica única')
 obj={e['name']:e for e in scenario['elements'] if e['type']=='trash'}
 if any(n not in obj for n in names):raise ValueError('La asignación no existe en escenario')
 return {r:(n, 'deposit_'+obj[n]['color']) for r,n in zip(ROBOTS,names)}

def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--stage',choices=STAGES,required=True)
 p.add_argument('--robot',choices=ROBOTS,required=True)
 p.add_argument('--scenario',default='scripts/d996_reference/scenario.json',type=Path)
 p.add_argument('--reference-dir',default='scripts/d996_reference',type=Path)
 p.add_argument('--historical-ledger',default='scripts/resultados/d8/seed_100_run_007/mission/mision.json',type=Path)
 p.add_argument('--navigation-report',required=True,type=Path,help='JSON D9.9.13 de la etapa, no el reporte general')
 p.add_argument('--world',default='world_demo')
 p.add_argument('--milp-assignment',type=Path,default=None)
 p.add_argument('--milp-trajectory',type=Path,default=None)
 p.add_argument('--milp-expected-sha256',default=None)
 p.add_argument('--execute',action='store_true')
 p.add_argument('--output',required=True,type=Path)
 a=p.parse_args()
 if a.output.exists():p.error('El informe ya existe; no repetir una maniobra sin revisar Gazebo')
 payload={'schema':'swarm.d9917.physical_stage.v1','status':'REJECT', 'stage':a.stage,'robot':a.robot,
          'execution_requested':a.execute,'commands_may_have_been_sent':False,
          'utc':datetime.now(timezone.utc).isoformat(),'problems':[]}
 try:
  scenario=json.loads(a.scenario.read_text())
  scenario_sha=hashlib.sha256(a.scenario.read_bytes()).hexdigest()
  if a.milp_assignment is None:
   if scenario.get('seed')!=100:raise ValueError('Solo se valida seed=100 histórico')
   historical=json.loads(a.historical_ledger.read_text())
   if historical.get('scenario_sha256')!=scenario_sha:
    raise ValueError('Escenario difiere del usado por D8 histórico')
   src=a.reference_dir/f'{a.stage}_d994_NON_EXECUTABLE.npz'
   if hashlib.sha256(src.read_bytes()).hexdigest()!=SHA[a.stage]:
    raise ValueError('Trayectoria D9.9 no coincide con la validada')
   assignments=mapping(a.stage,historical,scenario)
  else:
   if not a.milp_trajectory or not a.milp_expected_sha256:
    raise ValueError('MILP requiere NPZ y SHA explícitos')
   src=a.milp_trajectory
   if hashlib.sha256(src.read_bytes()).hexdigest()!=a.milp_expected_sha256:
    raise ValueError('MILP NPZ hash incorrecto')
   pre=json.loads(a.milp_assignment.read_text())
   if (pre.get('status') not in ('MILP_PREFLIGHT_VALID_OFFLINE_ONLY', 'MILP_V3_PREFLIGHT_VALID_OFFLINE_ONLY')
       or pre.get('seed')!=scenario.get('seed')
       or pre.get('scenario_sha256')!=scenario_sha):
    raise ValueError('MILP_PREFLIGHT_INVALID')
   st=pre['stages'][a.stage]
   if st['sha256']!=a.milp_expected_sha256 or Path(st['npz']).resolve()!=src.resolve():
    raise ValueError('MILP ruta distinta al preflight')
   assign=pre['assignment']
   names=[assign[r][0 if a.stage.endswith('1') else 1] for r in ROBOTS]
   objs={e['name']:e for e in scenario['elements'] if e['type']=='trash'}
   if any(n not in objs for n in names) or len(set(names))!=3:
    raise ValueError('MILP identidad inválida')
   assignments={r:(n,'deposit_'+objs[n]['color']) for r,n in zip(ROBOTS,names)}
  nav=json.loads(a.navigation_report.read_text())
  expected='DONE_ALL_'+a.stage.upper()
  if nav.get('status')!=expected or nav.get('problems'):
   raise ValueError(f'No está acreditada la navegación D9.9 de {a.stage}: {nav.get("status")}')
  trash,deposit=assignments[a.robot]
  payload.update({'trash':trash,'deposit':deposit,'mapping':assignments,
                  'navigation_verified':expected,'historical_assignments_only':a.milp_assignment is None})
  scripts=Path(__file__).resolve().parent
  if a.stage.startswith('pickup'):
   from d6_grasp_existing import snapshot_odom, gazebo_trash_positions, check_preapproach, scenario_targets
   odom=snapshot_odom()
   positions=gazebo_trash_positions(a.world)
   check=check_preapproach(a.robot,trash,odom,scenario_targets(scenario),positions,max_distance=2.2)
   payload['physical_preflight']=check
   original=scripts/'prueba_alinear_agarrar_cbs.py'
   cmd=[sys.executable,str(original),'--scenario',str(a.scenario),'--robot',a.robot,'--trash',trash,'--max-distance','2.2']
  else:
   # D8 delivery checks the live robot and joint state; a prior ACK is not enough.
   original=scripts/'prueba_entrega_local_cbs.py'
   cmd=[sys.executable,str(original),'--scenario',str(a.scenario),'--robot',a.robot,'--trash',trash,'--deposit',deposit,
        '--max-start-distance','1.8']
  if not original.is_file():raise FileNotFoundError(f'Rutina D8 ausente: {original}')
  if not a.execute:
   payload['status']='PREFLIGHT_READY_NO_COMMANDS' if a.stage.startswith('pickup') else 'DELIVERY_PREFLIGHT_NEEDS_LIVE_D8_CHECK_NO_COMMANDS'
  else:
   if not a.stage.startswith('pickup'):cmd.append('--execute')
   payload['commands_may_have_been_sent']=True
   atomic_json(a.output,payload)
   res=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,errors='replace')
   logfile=a.output.with_suffix('.log');logfile.write_text(res.stdout)
   payload['original_script_log']=str(logfile)
   payload['original_exit_code']=res.returncode
   if res.returncode:raise RuntimeError(f'Rutina física D8 devolvió {res.returncode}; revisar {logfile}')
   if a.stage.startswith('pickup'):
    required=('ALINEAR CONFIRMADO','JOINT CONFIRMADO','REGRESO CONFIRMADO')
   else:
    required=('DETACH CONFIRMADO','REGRESO CONFIRMADO')
   if not all(x in res.stdout for x in required):
    raise RuntimeError('Rutina D8 salió sin las confirmaciones requeridas; NO asumir operación física')
   payload['status']='PHYSICAL_ACK_REQUIRES_GAZEBO_INSPECTION'
   payload['ack_markers']=list(required)
 except Exception as exc:
  payload['problems'].append(str(exc))
  payload['status']='REJECT_STOPPED'
 finally:
  atomic_json(a.output,payload)
  print(json.dumps(payload,indent=2,ensure_ascii=False))
 return 0 if payload['status'] in ('PREFLIGHT_READY_NO_COMMANDS','DELIVERY_PREFLIGHT_NEEDS_LIVE_D8_CHECK_NO_COMMANDS','PHYSICAL_ACK_REQUIRES_GAZEBO_INSPECTION') else 2

if __name__=='__main__':raise SystemExit(main())
