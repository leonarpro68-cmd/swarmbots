"""Offline Hybrid V2 safety-aware optimization (no ROS imports).

The reference's SLSQP + sampled corridor refinement are preserved.
Extra margin is added to *existing exclusion radii* exactly once.
Nothing is approved for execution without a successful convex-hull certificate.
"""
from __future__ import annotations
import argparse
import json
import numpy as np
from pathlib import Path
from .scenario import reference_inputs
from .geometry_corridors import corridors
from .slsqp import solve_all
from .refinement import refine, cost
from .safety import SafetySettings, certify


def run(points, node_paths, obstacles, *, margin=0.05, iterations=5,
        maxiter=450, samples=32, max_speed=1.2, max_accel=1.0,
        time_safety_factor=1.08):
    result={'stage':'C.2','scenario':'external_cbs','certified':False,
            'margin_requested_m':margin,'iteration_count_requested':iterations,
            'wait_policy':'relaxed_intermediate_fixed_post_arrival'}
    if iterations < 0:
        raise ValueError('iterations must be nonnegative')
    try:
        hs=corridors(points,obstacles,operational_margin_m=margin)
        baseline,meta=solve_all(points,node_paths,hs,maxiter=maxiter)
        result['initial_cost']=cost(baseline)
        result['initial_solver']=meta
        controls,hist=refine(points,node_paths,obstacles,baseline,iterations=iterations,
                             samples=samples,operational_margin_m=margin,maxiter=maxiter)
        result['history']=hist
        result['final_cost']=cost(controls)
        settings=SafetySettings(margin,max_speed,max_accel,time_safety_factor)
        certificate=certify(controls,points,node_paths,obstacles,settings=settings,
                            preserve_intermediate_waits=False)
        result.update(certificate)
        result['status']='CERTIFIED'
    except (ValueError,RuntimeError) as ex:
        result['status']='REJECTED'
        result['reason']=str(ex)
        controls=None
    return result,controls


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--controls-output',type=Path,help='Write certified Bezier control points to NPZ')
    parser.add_argument('--operational-margin',type=float,default=0.05)
    parser.add_argument('--iterations',type=int,default=5)
    parser.add_argument('--maxiter',type=int,default=450)
    parser.add_argument('--max-speed',type=float,default=1.2)
    parser.add_argument('--max-accel',type=float,default=1.0)
    parser.add_argument('--time-safety-factor',type=float,default=1.08)
    args=parser.parse_args()
    paths,points,obstacles=reference_inputs()
    report,controls=run(points,paths,obstacles,margin=args.operational_margin,
                        iterations=args.iterations,maxiter=args.maxiter,
                        max_speed=args.max_speed,max_accel=args.max_accel,
                        time_safety_factor=args.time_safety_factor)
    report['scenario']='Entrega ronda 2'
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf8')
    if args.controls_output and report['certified']:
        args.controls_output.parent.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(args.controls_output, **{f'R{i}': np.asarray(c) for i,c in enumerate(controls)}, dt_s=report['dt_s'])
        print('Certified controls:',args.controls_output)
    print(report['status'],report.get('reason',''),args.output)
    return 0 if report['certified'] else 2


if __name__=='__main__':
    raise SystemExit(main())
