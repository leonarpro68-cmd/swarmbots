#!/usr/bin/env python3
"""Postprocess supermarket auto mission: original D9.9 route and V3 telemetry plotters.

Reads recorded ROS odometry CSV; creates JSONL compatible with D9931; no Gazebo commands.
"""
import argparse,csv,json,math,pathlib,re,subprocess,sys
STAGES=('pickup1','delivery1','pickup2','delivery2')

def find_scenario(prep, scripts):
 """Identify the exact map and seed from the prepared-plan directory.

 Supported layouts:
   plans/seed_N[/fixed]/prepared                  (supermarket)
   plans/<map>/seed_N[/fixed]/prepared            (other maps)
 No implicit fallback to supermarket for an unrecognized map.
 """
 from swarm_maps import scenario_file, map_files

 workspace = scripts.parent
 plans_root = (scripts / 'supermarket_seed_pipeline' / 'plans').resolve()
 prep = pathlib.Path(prep).resolve()
 try:
  rel = prep.relative_to(plans_root)
 except ValueError:
  rel = None

 if rel is not None:
  parts = rel.parts
  if parts and re.fullmatch(r'seed_(\d+)', parts[0]):
   map_name = 'supermarket'
   seed = int(parts[0][5:])
  elif len(parts) >= 2 and re.fullmatch(r'seed_(\d+)', parts[1]):
   map_name = parts[0]
   seed = int(parts[1][5:])
  else:
   raise ValueError(f'No se pudo identificar mapa/semilla de los planes: {prep}')

  # Validate that the selected map has its own data, instead of silently
  # reading supermarket or an unrelated scenario with the same seed.
  map_files(workspace, map_name)
  candidate = scenario_file(workspace, map_name, seed)
  if candidate.is_file():
   scenario = json.loads(candidate.read_text(encoding='utf-8'))
   if scenario.get('world') != map_name or scenario.get('seed') != seed:
    raise ValueError(f'Escenario no corresponde a {map_name}/seed_{seed}: {candidate}')
   return candidate
  if seed == 100 and map_name == 'supermarket':
   legacy = sorted(prep.glob('scenario_for_original_D8_seed100_*.json'))
   if len(legacy) == 1:
    return legacy[0]
  raise FileNotFoundError(f'Escenario de {map_name}/seed_{seed} no encontrado: {candidate}')

 # Preserve support for original D9.9 seed100 runs with no standard plan tree.
 legacy = sorted(prep.glob('scenario_for_original_D8_seed100_*.json'))
 if len(legacy) == 1:
  return legacy[0]
 if len(legacy) > 1:
  raise RuntimeError(f'Múltiples escenarios legacy en {prep}; no se elegirá uno arbitrariamente')
 raise FileNotFoundError(f'No se pudo identificar mapa y semilla desde {prep}')

def generate_routes(prep, run, scripts):
 """Plot routes against the actual scenario and collision geometry of its map."""
 from swarm_maps import map_files
 scenario_path=find_scenario(prep,scripts)
 map_name=json.loads(scenario_path.read_text(encoding='utf-8'))['world']
 _,geometry=map_files(scripts.parent,map_name)
 stage_windows=run/'stage_windows_v3.json'
 if not stage_windows.is_file():
  raise FileNotFoundError(f'No existe el registro de etapas: {stage_windows}')
 script=pathlib.Path(__file__).resolve().parent/'plot_routes_with_obstacles.py'
 output=run/'plots'/'rutas_teoricas.png'
 output.parent.mkdir(parents=True,exist_ok=True)
 subprocess.run([sys.executable,str(script),'--stage-windows',str(stage_windows),
                 '--scenario',str(scenario_path),'--geometry',str(geometry),
                 '--output',str(output),'--mission-label',run.name],check=True)
 return output

def main():
 p=argparse.ArgumentParser()
 p.add_argument('--prepared',required=True)
 p.add_argument('--run-dir',required=True)
 p.add_argument('--scripts-dir',default='scripts')
 p.add_argument('--only-routes',action='store_true',
                help='Regenera solo rutas_teoricas.png; reutiliza stage_windows_v3.json y no requiere ROS/Gazebo')
 a=p.parse_args()
 prep=pathlib.Path(a.prepared).resolve()
 run=pathlib.Path(a.run_dir).resolve()
 scripts=pathlib.Path(a.scripts_dir).resolve()
 sys.path.insert(0,str(scripts))

 if a.only_routes:
  output=generate_routes(prep,run,scripts)
  print(json.dumps({'status':'ROUTES_PLOT_GENERATED','map':json.loads(find_scenario(prep,scripts).read_text())['world'],
                    'files':[str(output)]},ensure_ascii=False))
  return

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
 output=generate_routes(prep,run,scripts)
 plots=run/'plots'
 script_profile=pathlib.Path(__file__).resolve().parent/'plot_v3_aligned.py'
 if not script_profile.is_file():raise RuntimeError('Profile plotting script is missing')
 subprocess.run([sys.executable,str(script_profile),'--stage-windows',str(run/'stage_windows_v3.json'),
                 '--odom-jsonl',str(run/'odom_for_d9931.jsonl'),
                 '--output',str(plots/'velocidad_aceleracion_teoria_vs_odometria.png'),
                 '--nav-speed-limit','2.0','--nav-accel-limit','2.0',
                 '--alignment-report',str(plots/'alineacion_temporal.json')],check=True)
 print(json.dumps({'status':'PLOTS_GENERATED','files':[str(output),
             str(plots/'velocidad_aceleracion_teoria_vs_odometria.png')]},ensure_ascii=False))
if __name__=='__main__':main()
