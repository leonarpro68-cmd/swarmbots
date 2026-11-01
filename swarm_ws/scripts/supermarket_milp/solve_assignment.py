#!/usr/bin/env python3
"""FASE 2 OFFLINE: MILP binario de parejas ordenadas + makespan por etapa.

La optimalidad certificada afecta SOLO al proxy de A*, NO a CBS/V3/Gazebo.
"""
from __future__ import annotations
import argparse
import itertools
import json
import math
from pathlib import Path
import numpy as np
from scipy.optimize import milp, Bounds, LinearConstraint
from scipy.sparse import lil_matrix
from build_costs import STAGES, HERE, _read, _write

BASELINE={
    'summit0':('trash_A2_azul','trash_A1_azul'),
    'summit1':('trash_V2_verde','trash_N2_naranja'),
    'summit2':('trash_N1_naranja','trash_V1_verde'),
}


def score(selection):
    st={s:max(p['time_by_stage_s'][s] for p in selection) for s in STAGES}
    dist=sum(p['total_distance_m'] for p in selection)
    return {'stage_makespan_s':{s:round(v,6) for s,v in st.items()},
            'sum_stage_makespan_s':round(sum(st.values()),6),
            'sum_robot_distance_m':round(dist,6),
            'robot_stage_estimated_s':{p['robot']:p['time_by_stage_s'] for p in selection}}


def _exact_candidates(rows,robots,names,one_color_per_round):
    lookup={(p['robot'],p['pickup1'],p['pickup2']):p for p in rows}
    colors={p['pickup1']:p['color1'] for p in rows}
    best=None; best_lines=None; count=0
    for order in itertools.permutations(names):
        first=order[:3];second=order[3:]
        if one_color_per_round:
            if len({colors[t] for t in first})!=3 or len({colors[t] for t in second})!=3:continue
        pairs=[lookup[(r,a,b)] for r,a,b in zip(robots,first,second)]
        s=score(pairs); count+=1
        key=(s['sum_stage_makespan_s'],s['sum_robot_distance_m'])
        if best is None or key < best:
            best=key;best_lines=pairs
    return count,best,best_lines


def solve(data,one_color_per_round=False,verify_exhaustive=True):
    if data.get('schema')!='supermarket.milp.phase1.astar_costs.v1':
        raise ValueError('Formato de costos inesperado')
    rows=data['pairs']; robots=sorted(data['robots']);names=sorted(data['trash'])
    if len(robots)!=3 or len(names)!=6 or len(rows)!=90:raise ValueError('Se requieren 90 parejas, 3 robots, 6 basuritas')
    if len({(p['robot'],p['pickup1'],p['pickup2']) for p in rows})!=len(rows):
        raise ValueError('Parejas duplicadas')
    n=len(rows); total_vars=n+len(STAGES)
    cols=[]; lows=[]; ups=[]
    # Restricciones exactamente una pareja por robot.
    for r in robots:
        cols.append({j:1. for j,p in enumerate(rows) if p['robot']==r});lows.append(1.);ups.append(1.)
    # Cada basurita aparece una vez entre las dos rondas.
    for t in names:
        cols.append({j:float((p['pickup1']==t)+(p['pickup2']==t)) for j,p in enumerate(rows)
                     if p['pickup1']==t or p['pickup2']==t});lows.append(1.);ups.append(1.)
    # Cada T(stage) >= costo de cualquier robot para esa etapa.
    for si,stage in enumerate(STAGES):
        for r in robots:
            c={j:p['time_by_stage_s'][stage] for j,p in enumerate(rows) if p['robot']==r}
            c[n+si]=-1.
            cols.append(c);lows.append(-math.inf);ups.append(0.)
    # Política opcional: un objeto de cada color en cada ronda.
    if one_color_per_round:
        colors=sorted({data['trash'][t]['color'] for t in names})
        for stg in ('pickup1','pickup2'):
            for color in colors:
                cols.append({j:1. for j,p in enumerate(rows) if p['color1' if stg=='pickup1' else 'color2']==color})
                lows.append(1.);ups.append(1.)

    A=lil_matrix((len(cols),total_vars),dtype=float)
    for k,c in enumerate(cols):
        for j,v in c.items():A[k,j]=v
    obj=np.zeros(total_vars)
    # Tiny tie-break for overall path length; never claim exact lexicographic optimization.
    obj[:n]=np.array([p['total_distance_m'] for p in rows])*1e-8
    obj[n:]=1.
    lb=np.zeros(total_vars)
    ub=np.array([1.]*n+[math.inf]*len(STAGES))
    integer=np.array([1]*n+[0]*len(STAGES))
    result=milp(c=obj,integrality=integer,bounds=Bounds(lb,ub),
                constraints=LinearConstraint(A.tocsr(),np.array(lows),np.array(ups)),
                options={'time_limit':120.0,'mip_rel_gap':1e-9})
    if result.x is None or result.status!=0:
        raise RuntimeError(f'MILP sin solución óptima certificada: status={result.status}, msg={result.message}')
    chosen=[rows[j] for j in range(n) if result.x[j]>0.5]
    if len(chosen)!=3 or len({p['robot'] for p in chosen})!=3:
        raise RuntimeError('MILP no seleccionó exactamente una pareja por robot')
    chosen.sort(key=lambda p:p['robot'])
    if sorted([p['pickup1'] for p in chosen]+[p['pickup2'] for p in chosen])!=names:
        raise RuntimeError('MILP no cubrió cada basurita una vez')
    lookup={(p['robot'],p['pickup1'],p['pickup2']):p for p in rows}
    baseline=[lookup[(r,*BASELINE[r])] for r in robots]
    s_milp=score(chosen);s_baseline=score(baseline)
    exhaustive=None
    if verify_exhaustive:
        count,best,best_rows=_exact_candidates(rows,robots,names,one_color_per_round)
        # Igualdad primaria independiente del diminuto desempate numérico.
        if abs(s_milp['sum_stage_makespan_s']-best[0]) > 1e-5:
            raise RuntimeError(f'MILP vs enumeración: {s_milp}, óptimo={best}')
        exhaustive={'evaluated_full_assignments':count,
                    'global_proxy_makespan_optimum_verified':True,
                    'best_proxy_makespan_s':best[0],
                    'best_proxy_distance_m_under_exact_tie':best[1]}
    return {
        'schema':'supermarket.milp.phase2.assignment.v1',
        'status':'OPTIMAL_ASTAR_PROXY_OFFLINE_NOT_EXECUTABLE',
        'execution_authorized':False,
        'world':data['world'],'seed':data['seed'],
        'objective':'minimize sum of four per-stage max A* travel-time proxies',
        'milp':{'binary_pair_variables':n,'continuous_stage_makespans':len(STAGES),
                'linear_constraints':len(cols),'solver':'scipy.optimize.milp (HiGHS)',
                'solver_status':int(result.status),'solver_message':result.message,
                'one_color_per_round':one_color_per_round,
                'time_limit_s':120.0,'relative_gap_tolerance':1e-9},
        'chosen':{p['robot']:{'pickup1':p['pickup1'],'delivery1':f"deposit_{p['color1']}",
                                'pickup2':p['pickup2'],'delivery2':f"deposit_{p['color2']}"}
                  for p in chosen},
        'optimized_proxy':s_milp,
        'validated_fixed_assignment':{
            'chosen':{r:{'pickup1':BASELINE[r][0],'pickup2':BASELINE[r][1]} for r in robots},
            'proxy':s_baseline},
        'proxy_gain_vs_fixed_s':round(s_baseline['sum_stage_makespan_s']-s_milp['sum_stage_makespan_s'],6),
        'proxy_gain_vs_fixed_percent':round(100*(s_baseline['sum_stage_makespan_s']-s_milp['sum_stage_makespan_s'])/s_baseline['sum_stage_makespan_s'],3),
        'independent_exhaustive_verification':exhaustive,
        'methodological_warning':[
            'Óptimo para esta matriz A*, no es un óptimo de la misión con CBS/Bezier/V3/Gazebo.',
            'No es un permiso para ejecutar los robots: faltan planificación CBS, Bezier, V3 y validación física.',
            'La matriz A* no modela esperas, congestión de depósitos, giro ni agarre/liberación.',
            'Para pasar a ejecución se deben reconstruir planes y reportes, nunca reutilizar los NPZ de la asignación anterior.'
        ],
    }


def main():
    ap=argparse.ArgumentParser(description='Fase 2 MILP offline. NO emite comandos ROS')
    ap.add_argument('--costs',type=Path,default=HERE/'results'/'seed_100'/'astar_costs.json')
    ap.add_argument('--output',type=Path,default=HERE/'results'/'seed_100'/'assignment_milp.json')
    ap.add_argument('--one-color-per-round',action='store_true',help='evitar que dos robots compartan depósito en misma ronda')
    ap.add_argument('--no-exhaustive',action='store_true',help='desactivar comprobación independiente de las 720 asignaciones')
    args=ap.parse_args()
    d=solve(_read(args.costs),args.one_color_per_round,not args.no_exhaustive)
    _write(args.output,d)
    print(json.dumps({'status':d['status'],'chosen':d['chosen'],
                      'optimal_proxy_s':d['optimized_proxy']['sum_stage_makespan_s'],
                      'baseline_proxy_s':d['validated_fixed_assignment']['proxy']['sum_stage_makespan_s'],
                      'proxy_gain_s':d['proxy_gain_vs_fixed_s'],
                      'exhaustive':d['independent_exhaustive_verification'],
                      'output':str(args.output)},ensure_ascii=False,indent=2))

if __name__=='__main__':main()
