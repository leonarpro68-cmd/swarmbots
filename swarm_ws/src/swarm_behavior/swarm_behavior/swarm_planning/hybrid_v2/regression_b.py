"""Offline reference regression for Entrega ronda 2; no ROS imports.

Use --reference-dir to point to original experiment's NPZ and reporte.json.
Reference-only comparison is quick; --solve reruns potentially costly SLSQP.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from .scenario import reference_inputs
from .slsqp import solve_all
from .refinement import refine, cost, sampled_corridors
from . import geometry_corridors as geom


def load_controls(path):
    with np.load(path, allow_pickle=False) as f:
        keys=sorted(f.files)
        if keys != ['R0','R1','R2']:
            raise ValueError('NPZ must have R0, R1, R2 arrays')
        return [np.asarray(f[k], float) for k in keys]


def diagnostics(controls, paths, points, obstacles, *, samples=160):
    n=len(controls); k=points.shape[1]-1
    curves=[]; lengths=[]; continuity={}
    for i, ctrl in enumerate(controls):
        if ctrl.shape!=(k,6,2):raise ValueError(f'R{i}: unexpected control shape {ctrl.shape}')
        if not np.isfinite(ctrl).all():raise ValueError('Nonfinite Bézier points')
        dense=np.vstack([np.asarray([geom.bernstein(5,t)@seg for t in np.linspace(0,1,samples)]) for seg in ctrl])
        curves.append(dense)
        lengths.append(float(sum(np.linalg.norm(np.diff(np.asarray([geom.bernstein(5,t)@seg for t in np.linspace(0,1,samples)]),axis=0),axis=1).sum() for seg in ctrl)))
        jumps0=[];jumps1=[];jumps2=[]
        for a,b in zip(ctrl[:-1],ctrl[1:]):
            jumps0.append(float(np.linalg.norm(a[5]-b[0])))
            jumps1.append(float(np.linalg.norm(5*(a[5]-a[4])-5*(b[1]-b[0]))))
            jumps2.append(float(np.linalg.norm(20*(a[5]-2*a[4]+a[3])-20*(b[2]-2*b[1]+b[0]))))
        continuity[f'R{i}']={f'C{order}_max':max(v,default=0.) for order,v in enumerate((jumps0,jumps1,jumps2))}
    min_obs=[min(float(np.linalg.norm(curves[i]-c,axis=1).min()-radius) for c,radius in obstacles) for i in range(n)]
    from .geometry_reference import CLEARANCE_WALL_M
    min_pair={f'{i}-{j}':float(np.linalg.norm(curves[i]-curves[j],axis=1).min()-2*CLEARANCE_WALL_M)
              for i in range(n) for j in range(i+1,n)}
    return {'cost_total':cost(controls),'cost_per_robot':[float(geom._acceleration_cost(c,1)) for c in controls],
            'path_length_m':lengths,'continuity':continuity,
            'sampled_obstacle_clearance_m':min_obs,'sampled_pair_clearance_m':min_pair}


def compare(actual, expected, tolerance=2e-5):
    diffs={}
    def check(name, x,y):
        delta=float(abs(x-y));diffs[name]=delta
        if delta>tolerance:raise AssertionError(f'{name} differs by {delta:.6g} (max {tolerance})')
    check('initial_cost',actual['initial']['cost_total'],expected['cost_total'])
    check('final_cost',actual['final']['cost_total'],expected['final_cost_total'])
    for i in range(3):
        check(f'final_robot_{i}',actual['final']['cost_per_robot'][i],expected['refinement_final_costs'][i])
        # sampled path length approximation has a numerical discretization error; report instead of strict match
    return diffs


def run(reference_dir, *, solve=False, iterations=5, maxiter=450):
    d=Path(reference_dir)
    expected=json.loads((d/'reporte.json').read_text(encoding='utf8'))
    paths,points,obstacles=reference_inputs()
    if expected['intervals'] != points.shape[1]-1 or expected['obstacles'] != {'columns':10,'trash':3,'deposits':3}:
        raise ValueError('Reference scenario mismatch')
    baseline=load_controls(d/'controles_slsqp.npz')
    final=load_controls(d/'controles_slsqp_refinados.npz')
    report={'mode':'reference_only','scenario':'Entrega ronda 2',
            'initial':diagnostics(baseline,paths,points,obstacles),
            'final':diagnostics(final,paths,points,obstacles)}
    report['reference_cost_differences']=compare(report,expected)
    if solve:
        hs=geom.corridors(points,obstacles)
        calc,metadata=solve_all(points,paths,hs,require_success=True,maxiter=maxiter)
        solved={'metadata':metadata,'initial':diagnostics(calc,paths,points,obstacles)}
        solved['difference_initial_cost_vs_reference']=abs(solved['initial']['cost_total']-expected['cost_total'])
        refined,hist=refine(points,paths,obstacles,calc,iterations=iterations,samples=32)
        solved['history']=hist
        solved['final']=diagnostics(refined,paths,points,obstacles)
        solved['difference_final_cost_vs_reference']=abs(solved['final']['cost_total']-expected['final_cost_total']) if iterations==5 else None
        report['solve']=solved;report['mode']='recomputed'
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--reference-dir',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--solve',action='store_true',help='Run expensive numerical optimization')
    p.add_argument('--iterations',type=int,default=5)
    p.add_argument('--maxiter',type=int,default=450)
    a=p.parse_args()
    report=run(a.reference_dir,solve=a.solve,iterations=a.iterations,maxiter=a.maxiter)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf8')
    print('Etapa B:',report['mode'],'J inicial',report['initial']['cost_total'],'J final',report['final']['cost_total'])
    if a.solve:print('Nuevo J inicial',report['solve']['initial']['cost_total'],'Nuevo J final',report['solve']['final']['cost_total'])
    print('Informe:',a.output)

if __name__=='__main__':main()
