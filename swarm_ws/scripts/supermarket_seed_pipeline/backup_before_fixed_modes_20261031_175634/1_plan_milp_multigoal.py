#!/usr/bin/env python3
"""Planifica asignación MILP con CBS multi-meta, obstáculos dinámicos.
Solo offline; no produce trayectorias continuas ni autoriza ejecución ROS 2.
"""
from __future__ import annotations
import argparse, hashlib, json, sys
from pathlib import Path
from supermarket_planning import cbs, geometry
from supermarket_planning.dynamic_map import DynamicTrashGraph
from supermarket_planning.approach import approach_vertices, deposit_goals
from supermarket_planning.multigoal import solve_multi

STAGES=('pickup1','delivery1','pickup2','delivery2')
ROBOTS=('summit0','summit1','summit2')
def read(p): return json.loads(Path(p).read_text(encoding='utf8'))
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def plan(assignment, scenario, *, max_ct=6000, max_time=110):
    if assignment.get('status')!='OPTIMAL_ASTAR_PROXY_OFFLINE_NOT_EXECUTABLE' or assignment.get('execution_authorized') is not False:
        raise ValueError('Se requiere una asignación MILP offline reconocida')
    if assignment.get('seed')!=scenario.get('seed') or not isinstance(scenario.get('seed'), int) or scenario['seed'] < 0:
        raise ValueError('Semillas de asignación/escenario distintas o inválidas')
    chosen=assignment['chosen']
    if tuple(sorted(chosen))!=ROBOTS:raise ValueError('Los tres robots no coinciden')
    bytrash={t['name']:t for t in scenario['trash']}
    bydep={d['name']:d for d in scenario['deposits']}
    picks=[chosen[r][f'pickup{k}'] for k in (1,2) for r in ROBOTS]
    if len(bytrash)!=6 or set(picks)!=set(bytrash):raise ValueError('Asignación de basuras no biyectiva')
    for r in ROBOTS:
        for k in (1,2):
            trash=bytrash[chosen[r][f'pickup{k}']]
            dep=bydep[chosen[r][f'delivery{k}']]
            if dep['color']!=trash['color']:raise ValueError(f'{r}: depósito incorrecto')
    byrobot={r['name']:r for r in scenario['robots']}
    starts=tuple(tuple(byrobot[r]['grid']) for r in ROBOTS)
    pending=set(bytrash)
    report={'schema':'supermarket.milp.cbs.multigoal.dynamic.offline.v1',
        'status':'OFFLINE_CBS_ONLY_NOT_EXECUTABLE','execution_authorized':False,
        'stage_sequence':list(STAGES),'seed':scenario['seed'],'assignment':chosen,
        'graph_model':'DynamicTrashGraph, multi-goal CBS, pending trash obstacles',
        'stages':{},'problems':[]}
    makespans=[]
    for stage in STAGES:
        graph=DynamicTrashGraph(scenario,pending=pending,ignored_trash=(),accessible_deposits=())
        if any(start not in graph.vertices for start in starts):
            raise ValueError(f'{stage}: el inicio está bloqueado')
        if stage.startswith('pickup'):
            targets=[chosen[r][stage] for r in ROBOTS]
            options=tuple(approach_vertices(graph,n) for n in targets)
        else:
            targets=[chosen[r][stage] for r in ROBOTS]
            options=tuple(deposit_goals(graph,bydep[n]['color']) for n in targets)
        if any(not goals for goals in options):raise ValueError(f'{stage}: metas accesibles vacías')
        sol=solve_multi((graph,)*3,starts,options,max_ct=max_ct,max_time=max_time)
        paths=sol['paths']
        if cbs.first_conflict(paths) is not None:raise RuntimeError('CBS con conflictos persistentes')
        for p,goals in zip(paths,options):
            if p[-1] not in goals or any(v not in graph.vertices for v in p):
                raise RuntimeError('La ruta utiliza nodos bloqueados o meta inválida')
            if any(a!=b and b not in graph.neighbors[a] for a,b in zip(p,p[1:])):
                raise RuntimeError('La ruta atraviesa aristas bloqueadas')
        goals=tuple(p[-1] for p in paths)
        row={'status':'CBS_CONFLICT_FREE_DISCRETE','starts_grid':[list(x) for x in starts],
            'goals_grid':[list(x) for x in goals],
            'goal_options':[[list(x) for x in g] for g in options],
            'pending_before':sorted(pending),
            'targets':targets,
            'cbs_sum_of_costs_ticks':sol['cost'],
            'makespan_ticks':max(len(p)-1 for p in paths),
            'ct_expanded':sol['expanded'],'ct_generated':sol['generated'],
            'dynamic_nodes':len(graph.vertices),'dynamic_edges':len(graph.edges),
            'paths':[{'robot':r,'start_grid':list(p[0]),'goal_grid':list(p[-1]),
                      'ticks':len(p)-1,'grid':[list(v) for v in p],
                      'xy_m':[list(geometry.node_position(v)) for v in p]}
                    for r,p in zip(ROBOTS,paths)]}
        report['stages'][stage]=row
        print(f"{stage}: CBS OK, ticks {[len(p)-1 for p in paths]}, metas {[list(g) for g in goals]}",flush=True)
        starts=goals
        makespans.append(row['makespan_ticks'])
        if stage.startswith('pickup'):pending.difference_update(targets)
    if pending:raise RuntimeError('Quedaron basuras pendientes después de pickup2')
    report['total_discrete_makespan_ticks']=sum(makespans)
    return report

def main():
    p=argparse.ArgumentParser(description=__doc__)
    here=Path(__file__).resolve().parent
    p.add_argument('--assignment',type=Path,default=here/'assignment_milp_reference.json')
    p.add_argument('--scenario',type=Path,default=here/'seed_100_reference.json')
    p.add_argument('--output',type=Path,default=here/'results/milp_cbs_multigoal_seed100.json')
    p.add_argument('--max-time',type=int,default=110)
    p.add_argument('--max-ct',type=int,default=6000)
    a=p.parse_args()
    if a.output.exists():p.error(f'No se sobrescribe: {a.output}')
    report=plan(read(a.assignment),read(a.scenario),max_time=a.max_time,max_ct=a.max_ct)
    report['inputs_sha256']={'assignment':sha(a.assignment),'scenario':sha(a.scenario)}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
    print('PASS_OFFLINE_CBS_DYNAMIC_MULTIGOAL_NOT_EXECUTABLE',report['total_discrete_makespan_ticks'])
    print('REPORTE:',a.output)
    return 0
if __name__=='__main__':sys.exit(main())
