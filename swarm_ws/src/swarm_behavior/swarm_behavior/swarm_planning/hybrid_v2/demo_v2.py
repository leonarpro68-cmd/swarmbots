"""Offline validation for the V2 reference; no ROS or Gazebo."""
import argparse
from pathlib import Path
import numpy as np
from .scenario import reference_inputs
from . import geometry_corridors as opt
from .slsqp import solve_all
from .refinement import refine,cost

def main():
    p=argparse.ArgumentParser();p.add_argument('--reference-npz',required=True)
    p.add_argument('--solve',action='store_true');p.add_argument('--iterations',type=int,default=0)
    a=p.parse_args()
    paths,points,obs=reference_inputs()
    with np.load(a.reference_npz) as file:
        controls=[file[f'R{i}'] for i in range(len(paths))]
    print('Referencia NPZ: ',len(controls),'robots,',len(controls[0]),'piezas, J=',cost(controls))
    if a.solve:
        corridors=opt.corridors(points,obs)
        controls,metadata=solve_all(points,paths,corridors)
        print('SLSQP:',metadata)
        if a.iterations:controls,hist=refine(points,paths,obs,controls,iterations=a.iterations);print('Historial:',hist)
        print('J final:',cost(controls))
if __name__=='__main__':main()
