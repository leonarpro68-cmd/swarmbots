#!/usr/bin/env python3
"""Postprocess supermarket auto mission: original D9.9 route and V3 telemetry plotters.

Reads recorded ROS odometry CSV; creates JSONL compatible with D9931; no Gazebo commands.
"""
import argparse,csv,json,math,pathlib,re,subprocess,sys
STAGES=('pickup1','delivery1','pickup2','delivery2')

def find_scenario(prep, scripts):
 """Resolve the exact scenario for legacy seed100 and prepared per-seed plans.

 Do not silently use seed100 or an unrelated scenario for a different seed.
 """
 seeds=[]
 for parent in (prep, *prep.parents):
  m=re.fullmatch(r'seed_(\d+)',parent.name)
  if m:
   seeds.append(int(m.group(1)))
   break
 if seeds:
  seed=seeds[0]
  workspace=scripts.parent
  plan_seed=next(parent for parent in (prep,*prep.parents) if parent.name==f'seed_{seed}')
  map_name=plan_seed.parent.name if plan_seed.parent.name in ('supermarket','tugbot_warehouse') else 'supermarket'
  candidate=(workspace/'config'/'supermarket'/f'seed_{seed}.json' if map_name=='supermarket'
             else workspace/'config'/'scenarios'/map_name/f'seed_{seed}.json')
  if candidate.is_file():
   return candidate
  # Legacy seed100 plans sometimes hold their own exact scenario.
  if seed==100:
   older=sorted(prep.glob('scenario_for_original_D8_seed100_*.json'))
   if len(older)==1:return older[0]
  raise FileNotFoundError(f'Escenario de semilla {seed} no encontrado: {candidate}')
 legacy=sorted(prep.glob('scenario_for_original_D8_seed100_*.json'))
 if len(legacy)==1:return legacy[0]
 if len(legacy)>1:raise RuntimeError(f'Múltiples escenarios legacy en {prep}; especificar uno no ambiguo')
 raise FileNotFoundError(f'No se pudo identificar semilla desde {prep}; no se elegirá un escenario arbitrario')

def main():
 p=argparse.ArgumentParser();p.add_argument('--prepared',required=True);p.add_argument('--run-dir',required=True);p.add_argument('--scripts-dir',default='scripts');a=p.parse_args()
 prep=pathlib.Path(a.prepared).resolve();run=pathlib.Path(a.run_dir).resolve();scripts=pathlib.Path(a.scripts_dir).resolve()
 conf=json.loads((prep/'phase5_static_preflight.json').read_text());events={}
 with (run/'events.csv').open(newline='') as f:
  for e in csv.DictReader(f):
   if e['robot']=='all':events.setdefault(e['event'],[]).append(float(e['sim_time_s']))
 windows=[]
 for st in STAGES:
  s=events.get(st+'_nav_start',[]);e=events.get(st+'_nav_end',[])
  if len(s)!=1 or len(e)!=1 or e[0]<=s[0]:raise RuntimeError(f'{st}: no valid recorded navigation window')
  nav=conf['stages'][st]
  windows.append({'stage':st,'trajectory':nav['trajectory'],'clock_json':nav['successive_clock'],
    'time_model':'common_collective_successive_v1','start_time_s':s[0], 'end_time_s':e[0],
    'max_duration_s':e[0]-s[0]})
 payload={'stages':windows};(run/'stage_windows_v3.json').write_text(json.dumps(payload,indent=2)+'\n')
 with (run/'odom.csv').open(newline='') as f, (run/'odom_for_d9931.jsonl').open('w') as g:
  count=0
  for o in csv.DictReader(f):
   t=float(o['stamp_s']); vx=float(o['vx_local_m_s']);vy=float(o['vy_local_m_s']);
   qz=float(o['qz']);qw=float(o['qw']);qx=float(o['qx']);qy=float(o['qy'])
   # Euler yaw from normalized quaternion (no flat-world yaw shortcut).
   yaw=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz));c=math.cos(yaw);s=math.sin(yaw)
   wx=vx*c-vy*s;wy=vx*s+vy*c
   row={'robot':o['robot'],'clock_s':t,'x_m':float(o['x_m']),'y_m':float(o['y_m']),
        'vx_m_s':wx,'vy_m_s':wy,'speed_m_s':math.hypot(vx,vy)}
   g.write(json.dumps(row)+'\n');count+=1
 if count<12:raise RuntimeError('Insufficient odometry records')
 plots=run/'plots';plots.mkdir(exist_ok=True)
 plot_dir=pathlib.Path(__file__).resolve().parent
 script_route=plot_dir/'plot_routes_with_obstacles.py';script_profile=plot_dir/'plot_v3_aligned.py'
 if not script_route.is_file() or not script_profile.is_file():raise RuntimeError('New plotting scripts are missing')
 scenario_file=find_scenario(prep,scripts)
 map_name=json.loads(scenario_file.read_text(encoding='utf-8'))['world']
 geometry=scripts.parent/'config'/'maps'/map_name/'obstaculos.json'
 if not geometry.is_file():raise RuntimeError(f'No se encontró la geometría del mapa activo: {geometry}')
 cmd1=[sys.executable,str(script_route),'--stage-windows',str(run/'stage_windows_v3.json'),
       '--scenario',str(scenario_file), '--geometry',str(geometry),
       '--output',str(plots/'rutas_teoricas.png'),'--mission-label',run.name]
 cmd2=[sys.executable,str(script_profile),'--stage-windows',str(run/'stage_windows_v3.json'),
       '--odom-jsonl',str(run/'odom_for_d9931.jsonl'),'--output',str(plots/'velocidad_aceleracion_teoria_vs_odometria.png'),
       '--nav-speed-limit','2.0','--nav-accel-limit','2.0','--alignment-report',str(run/'plots'/'alineacion_temporal.json')]
 for cmd in (cmd1,cmd2):subprocess.run(cmd,check=True)
 print(json.dumps({'status':'PLOTS_GENERATED','files':[str(plots/'rutas_teoricas.png'),str(plots/'velocidad_aceleracion_teoria_vs_odometria.png')]}))
if __name__=='__main__':main()
