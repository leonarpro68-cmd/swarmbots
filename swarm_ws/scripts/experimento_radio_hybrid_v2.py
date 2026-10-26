#!/usr/bin/env python3
"""Experimento OFFLINE: cambia únicamente el radio circular efectivo V2.

No altera geometry_reference.py ni el CBS ni persiste valores. NO usar
sus certificados para conducir robots sin verificar la huella física.
"""
import argparse
import json
from pathlib import Path
import plot_hybrid_v2_mission as mission
from swarm_behavior.swarm_planning.hybrid_v2 import geometry_reference as geom
from swarm_behavior.swarm_planning.hybrid_v2 import geometry_corridors as corridor


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--seed',type=int,default=123)
    ap.add_argument('--scenario',type=Path,help='Usar el JSON EXACTO de D.8 en vez de regenerarlo desde la semilla')
    ap.add_argument('--radius',type=float,default=0.55)
    ap.add_argument('--margin',type=float,default=0.05)
    ap.add_argument('--iterations',type=int,default=2)
    ap.add_argument('--max-speed',type=float,default=1.20)
    ap.add_argument('--max-accel',type=float,default=1.00)
    ap.add_argument('--output',type=Path,default=Path('scripts/resultados/mision_123_hybrid_v2_r055.png'))
    ap.add_argument('--json-output',type=Path)
    ap.add_argument('--controls-dir',type=Path,help='Guardar NPZ de etapas certificadas para D.3')
    a=ap.parse_args()
    if not (0<a.radius<2) or a.margin<0: ap.error('Radio inválido o margen negativo')
    original_radius=geom.CLEARANCE_WALL_M
    # These two module globals govern the corridor constructor; refinement and
    # certificate read geom.CLEARANCE_WALL_M dynamically.
    geom.CLEARANCE_WALL_M=a.radius
    corridor.R=a.radius
    try:
        import subprocess,sys,tempfile
        from swarm_behavior.swarm_planning.scenario import load as load_scenario
        if a.scenario:
            scenario=load_scenario(a.scenario)
        else:
            with tempfile.TemporaryDirectory(prefix='v2_radius_') as tmp:
                scen=Path(tmp)/'scenario.json'
                generator=Path(mission.original.__file__).resolve().with_name('escenario_aleatorio_swarm.py')
                subprocess.run([sys.executable,str(generator),'--seed',str(a.seed),'--output',str(scen)],check=True)
                scenario=load_scenario(scen)
        products,contexts,summary=mission.plan_four(scenario,margin=a.margin,iterations=a.iterations,
                    max_speed=a.max_speed,max_accel=a.max_accel)
        mission.draw(scenario,products,contexts,summary,a.output)
        out=a.json_output or a.output.with_suffix('.json')
        data=mission.export_report(scenario,products,summary,out,controls_dir=a.controls_dir)
        data['experiment']={'robot_radius_effective_m':a.radius,
            'original_reference_radius_m':original_radius,
            'additional_operational_margin_m':a.margin,
            'pair_separation_required_m':2*a.radius+a.margin,
            'warning':'EXPERIMENTAL: not validated against Summit physical collision geometry'}
        out.write_text(json.dumps(data,ensure_ascii=False,indent=2,default=str),encoding='utf-8')
        print('PNG:',a.output,'JSON:',out)
        print('RADIO',a.radius,'MARGEN',a.margin,'SEPARACION',2*a.radius+a.margin)
        print('ESTADOS',{s:data['stages'][s]['hybrid_v2']['status'] for s in mission.STAGES})
        return 0 if all(data['stages'][s]['certified'] for s in mission.STAGES) else 2
    finally:
        geom.CLEARANCE_WALL_M=original_radius
        corridor.R=original_radius

if __name__=='__main__':raise SystemExit(main())
