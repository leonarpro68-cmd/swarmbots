#!/usr/bin/env python3
"""Construye contrato FIXED seed100 sin alterar planes, escenas ni controladores existentes."""
import argparse, hashlib, json, pathlib, math, sys
from shapely.geometry import LineString,Point,box

STAGES=('pickup1','delivery1','pickup2','delivery2')
ROBOTS=('summit0','summit1','summit2')
PICKUPS={'pickup1':('trash_A2_azul','trash_V2_verde','trash_N1_naranja'),
         'pickup2':('trash_A1_azul','trash_N2_naranja','trash_V1_verde')}
EXPECTED_SHA={'pickup1':'4a3015c4ac3d64da0555b485a3bfac77b7243340d251c8f6388c19ac35f19bd4',
 'delivery1':'500234f35039471f15f42801d3c76ebc5686cbe4c480fac6eda98c142fdec0e3',
 'pickup2':'7f05c330ef25bb2c0b7bb84c3b1acb4661134919c39480d39cf22cf59435ed91',
 'delivery2':'33aff6468eea915cc8f3e6da3d3df896392ca2fc996737bde7c02918dba65b70'}

def read(p):return json.loads(p.read_text())
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def audit_local(start,center,stage,target,pending,scene,geom):
 d=math.dist(start,center)
 stop=0.65 if stage.startswith('pickup') else 0.80
 end=[center[i]+(start[i]-center[i])*stop/d for i in range(2)]
 segment=LineString([start,end]);hits=[]
 for o in geom['obstacles']:
  if segment.distance(box(o['xmin'],o['ymin'],o['xmax'],o['ymax'])) < 0.60-1e-6:
   hits.append(f"shelf:{o['name']}")
 exclusion=0.55+math.sqrt(2)*0.09/2+0.05
 for t in scene['trash']:
  if t['name']!=target and t['name'] in pending and segment.distance(Point(t['xy_m']))<exclusion-1e-6:
   hits.append('trash:'+t['name'])
 for dep in scene['deposits']:
  if dep['name']!=target and segment.distance(Point(dep['xy_m']))<0.50-1e-6:
   hits.append('deposit:'+dep['name'])
 return {'straight_line_geometric_screen_pass':not hits,'hits':hits,'stop_m':stop,'inspection_endpoint_xy_m':end,
         'note':'Cribado de línea ideal, no confirma maniobra visual D8 en Gazebo'}

def run(ws, out, geometry):
 import numpy as np
 src=read(ws/'config/supermarket/seed_100.json')
 if src.get('seed')!=100:raise ValueError('Semilla inesperada')
 elements=src.get('elements') or src['trash']+src['deposits']
 geom=read(geometry)
 scene=dict(src);scene['elements']=elements;scene['fixed_origin']='seed100_original_mission'
 objects={x['name']:x for x in elements}
 if len([x for x in elements if x['type']=='trash'])!=6:raise ValueError('Se esperaban seis basuritas')
 if len([x for x in elements if x['type']=='deposit'])!=3:raise ValueError('Se esperaban tres depósitos')
 trajectories=ws/'scripts/supermarket_v3/data'
 info={'schema':'supermarket.fixed.phase6b.preflight.v1','status':'PASS_STATIC_PRECHECK_NOT_EXECUTABLE',
  'execution_authorized':False,'errors':[],'assignment_mode':'fixed','stages':{},'d8_tasks':[]}
 assignments={}
 prev_ends=None
 for stage in STAGES:
  npz=(trajectories/f'{stage}_supermarket_seed100_v3.npz').resolve()
  clock=(trajectories/f'{stage}_supermarket_seed100_clock.json').resolve()
  for p in (npz,clock):
   if not p.is_file():raise FileNotFoundError(f'{stage}: {p}')
  sha=digest(npz)
  if sha!=EXPECTED_SHA[stage]:raise ValueError(f'{stage}: SHA inesperado {sha}; no usar datos distintos')
  c=read(clock)
  if c.get('input_sha256')!=sha:raise ValueError(f'{stage}: reloj V3 no coincide')
  starts=[];ends=[]
  with np.load(npz,allow_pickle=False) as data:
   for i in range(3):
    cps=data[f'R{i}_controls']
    if cps.ndim!=3 or cps.shape[1:]!=(6,2) or not np.isfinite(cps).all():raise ValueError(f'{stage} R{i}: curvas inválidas')
    starts.append(list(map(float,cps[0,0])));ends.append(list(map(float,cps[-1,-1])))
  if prev_ends is not None:
   for i in range(3):
    if math.dist(starts[i],prev_ends[i])>1e-5:raise ValueError(f'{stage} summit{i}: no hay continuidad de etapas')
  else:
   for i in range(3):
    initial=src['robots'][i]['xy_m']
    if math.dist(starts[i],initial)>1e-4:raise ValueError(f'pickup1 summit{i}: inicio no coincide con semilla')
  prev_ends=ends
  info['stages'][stage]={'stage':stage,'trajectory':str(npz),'successive_clock':str(clock),'sha256':sha,
      'start_xy_m':starts,'goal_xy_m':ends,'duration_v3_s':c.get('duration_successive_s')}
  slots={}
  pending=set(PICKUPS['pickup1']+PICKUPS['pickup2'])
  if stage!='pickup1':pending-=set(PICKUPS['pickup1'])
  if stage in ('pickup2','delivery2'):pending-=set(PICKUPS['pickup2']) if stage=='delivery2' else set()
  for i,robot in enumerate(ROBOTS):
   pickstage='pickup1' if stage.endswith('1') else 'pickup2'
   trash=PICKUPS[pickstage][i]
   t=objects[trash];color=t['color']; target=trash if stage.startswith('pickup') else f'deposit_{color}'
   if target not in objects:raise ValueError(f'{stage}/{robot}: depósito faltante')
   target_xy=objects[target]['xy_m']; d=math.dist(ends[i],target_xy)
   lower,upper=(0.55,2.2) if stage.startswith('pickup') else (0.9,1.8)
   if not lower<=d<=upper:raise ValueError(f'{stage}/{robot}: distancia D8 {d:.3f} fuera del rango [{lower},{upper}]')
   slots[robot]={'trash':trash,'color':color,'target':target,'goal_xy_m':ends[i],
                'target_xy_m':target_xy,'distance_m':round(d,6)}
   local=audit_local(ends[i],target_xy,stage,target,pending,src,geom)
   if not local['straight_line_geometric_screen_pass']:
    info['errors'].append(f'{stage}/{robot}: corredor local potencialmente ocupado {local["hits"]}')
   info['d8_tasks'].append({'stage':stage,'robot':robot,'target':target,'color':color,'d8_distance_gate':True,
      'local_corridor_screen':local,'distance_m':round(d,6)})
  assignments[stage]={'schema':'supermarket.fixed.physical.assignment.v1','stage':stage,'robots':slots,'navigation_sha256':sha}
 info['status']='PASS_STATIC_PRECHECK_NOT_EXECUTABLE' if not info['errors'] else 'REJECTED_STATIC_PRECHECK'
 out.mkdir(parents=True,exist_ok=True)
 (out/'scenario_for_original_D8_seed100_FIXED.json').write_text(json.dumps(scene,ensure_ascii=False,indent=2)+'\n')
 for stage,ass in assignments.items():
  (out/f'assignment_{stage}_FIXED.json').write_text(json.dumps(ass,ensure_ascii=False,indent=2)+'\n')
 (out/'phase5_static_preflight.json').write_text(json.dumps(info,ensure_ascii=False,indent=2)+'\n')
 return info

def main():
 p=argparse.ArgumentParser();p.add_argument('--workspace',type=pathlib.Path,default=pathlib.Path.home()/'swarm_ws');p.add_argument('--output-dir',type=pathlib.Path,required=True);p.add_argument('--geometry',type=pathlib.Path,default=pathlib.Path(__file__).with_name('supermarket_obstaculos.json'))
 a=p.parse_args()
 try:d=run(a.workspace.expanduser().resolve(),a.output_dir.expanduser().resolve(),a.geometry.expanduser().resolve())
 except Exception as e:print('FAIL:',e,file=sys.stderr);return 2
 print(json.dumps({'status':d['status'],'stages':list(d['stages']),'reason':d['errors']},ensure_ascii=False,indent=2))
 return 0
if __name__=='__main__':sys.exit(main())
