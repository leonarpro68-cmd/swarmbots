#!/usr/bin/env python3
"""MILP -> CBS cuatro etapas (OFFLINE, NO ejecutable)."""
import argparse, hashlib, json, math, sys
from pathlib import Path
from supermarket_planning import cbs,geometry

STAGES=('pickup1','delivery1','pickup2','delivery2')
DEP_APPROACH={'azul': (16,12),'verde': (10,19),'naranja': (8,32)}
# Paradas registradas en la prueba anterior; modelo discreto de validación.

def load(path):return json.loads(Path(path).read_text(encoding='utf-8'))
def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser(description=__doc__)
 here=Path(__file__).resolve().parent
 p.add_argument('--assignment',default=str(here/'assignment_milp_reference.json'))
 p.add_argument('--scenario',default=str(here/'seed_100_reference.json'))
 p.add_argument('--output',default=str(here/'results/milp_cbs_seed100.json'))
 p.add_argument('--max-time',type=int,default=130)
 p.add_argument('--max-ct',type=int,default=10000)
 a=p.parse_args()
 if Path(a.output).exists():p.error('No se sobrescribe un reporte existente: '+a.output)
 assignment=load(a.assignment);scene=load(a.scenario)
 if assignment.get('status')!='OPTIMAL_ASTAR_PROXY_OFFLINE_NOT_EXECUTABLE':p.error('Estado MILP inesperado')
 if scene.get('seed')!=assignment.get('seed'):p.error('Semillas diferentes')
 chosen=assignment['chosen']; robots=sorted(chosen)
 if robots!=['summit0','summit1','summit2']:p.error('Se requieren summit0, summit1, summit2')
 robotdata={r['name']:r for r in scene['robots']}
 trashdata={t['name']:t for t in scene['trash']}
 deposits={d['name']:d for d in scene['deposits']}
 if len(trashdata)!=6 or len(set(chosen[r][s] for r in robots for s in ('pickup1','pickup2')))!=6:p.error('Las seis basuras deben ser distintas')
 for r in robots:
  for k in (1,2):
   t=trashdata[chosen[r][f'pickup{k}']]
   if chosen[r][f'delivery{k}']!=f"deposit_{t['color']}":p.error('Depósito incorrecto para '+r)
 graph=cbs.Graph(); starts=tuple(tuple(robotdata[r]['grid']) for r in robots)
 report={'schema':'supermarket.milp.cbs.discrete.offline.v1','status':'OFFLINE_CBS_ONLY_NOT_EXECUTABLE', 'execution_authorized':False,'stage_sequence':list(STAGES),'seed':scene['seed'], 'assignment':chosen,'inputs_sha256':{'assignment':digest(a.assignment),'scenario':digest(a.scenario)},'graph':{'nodes':len(graph.vertices),'edges':len(graph.edges),'min_center_separation_m':cbs.MIN_SEPARATION},'stages':{},'problems':[]}
 total_ticks=0
 try:
  for stage in STAGES:
   if stage.startswith('pickup'):
    goals=tuple(tuple(trashdata[chosen[r][stage]]['approach_grid']) for r in robots)
   else:
    goals=tuple(DEP_APPROACH[deposits[chosen[r][stage]]['color']] for r in robots)
   if any(v not in graph.vertices for v in starts+goals):raise ValueError(f'{stage}: meta o inicio fuera del grafo')
   if len(set(goals))!=3:raise ValueError(f'{stage}: metas compartidas, no se puede certificar espera final')
   res=cbs.solve(starts,goals,graph=graph,max_ct=a.max_ct,max_time=a.max_time)
   if cbs.first_conflict(res['paths']) is not None:raise RuntimeError(f'{stage}: conflicto persistente')
   paths=[]
   for rid,(r,path) in enumerate(zip(robots,res['paths'])):
    if path[0]!=starts[rid] or path[-1]!=goals[rid]:raise RuntimeError('Extremos incorrectos')
    paths.append({'robot':r,'start_grid':list(path[0]),'goal_grid':list(path[-1]),'ticks':len(path)-1,'grid':[list(v) for v in path], 'xy_m':[list(geometry.node_position(v)) for v in path]})
   makespan=max(x['ticks'] for x in paths);total_ticks+=makespan
   report['stages'][stage]={'status':'CBS_CONFLICT_FREE_DISCRETE','starts_grid':[list(s) for s in starts],'goals_grid':[list(g) for g in goals],'cbs_sum_of_costs_ticks':res['cost'],'makespan_ticks':makespan,'ct_expanded':res['expanded'],'ct_generated':res['generated'],'paths':paths}
   print(f'{stage}: CBS OK, ticks individuales {[x["ticks"] for x in paths]}, makespan {makespan}, CT {res["expanded"]}')
   starts=goals
  report['total_discrete_makespan_ticks']=total_ticks
  print('PASS_OFFLINE_CBS_FOUR_STAGES_NOT_EXECUTABLE; ticks=',total_ticks)
 except (ValueError,RuntimeError) as exc:
  report['status']='FAILED_OFFLINE_CBS';report['problems'].append(f'{type(exc).__name__}: {exc}');print('FAILED:',exc,file=sys.stderr)
 path=Path(a.output);path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
 print('Reporte:',path)
 if report['problems']:sys.exit(2)
if __name__=='__main__':main()
