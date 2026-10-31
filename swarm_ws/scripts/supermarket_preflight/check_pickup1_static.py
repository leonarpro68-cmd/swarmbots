#!/usr/bin/env python3
"""Preflight offline for supermarket seed100 pickup1. Read-only; no ROS imports."""
import argparse, hashlib, json, math, sys
from pathlib import Path
import numpy as np

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def check(args):
    scenario=json.loads(args.scenario.read_text())
    mission=json.loads(args.mission.read_text())
    clock=json.loads(args.clock.read_text())
    st=next(s for s in mission['best']['stages'] if s['stage']=='pickup1')
    checkouts={}
    checkouts['scenario_world_seed']=scenario['world']=='supermarket' and scenario['seed']==mission['seed']==100
    checkouts['three_robots_six_trash_three_deposits']=tuple(len(scenario[x]) for x in ('robots','trash','deposits'))==(3,6,3)
    checkouts['trajectory_clock_sha']=sha(args.trajectory)==clock.get('input_sha256')
    checkouts['v3_status']=clock.get('status')=='OFFLINE_SCHEDULE_CERTIFIED_NOT_EXECUTABLE'
    checkouts['two_m_s_config']=args.speed==2.0 and args.accel==2.0
    with np.load(args.trajectory,allow_pickle=False) as timed, np.load(args.refined,allow_pickle=False) as refined:
        for i in range(3):
            a=np.asarray(timed[f'R{i}_controls']); b=np.asarray(refined[f'R{i}']); d=timed[f'R{i}_durations_s']
            checkouts[f'R{i}_v3_uses_exact_multigoal_refined_controls']=a.shape==b.shape and np.array_equal(a,b)
            checkouts[f'R{i}_durations_valid']=d.shape==(len(a),) and bool(np.isfinite(d).all() and (d>0).all())
            start=np.array(scenario['robots'][i]['xy_m'],float)
            checkouts[f'R{i}_start_matches_scenario']=bool(np.allclose(a[0,0], start,atol=1e-8))
            goal=np.array([-15.0+1.25*st['selected_goals'][i][0],-25.0+1.25*st['selected_goals'][i][1]])
            checkouts[f'R{i}_goal_matches_multigoal_CBS']=bool(np.allclose(a[-1,-1],goal,atol=1e-8))
    report={'schema':'swarm.supermarket.pickup1.preflight.static.v1','seed':100,'stage':'pickup1',
            'status':'PASS_OFFLINE_ONLY' if all(checkouts.values()) else 'BLOCKED',
            'checks':checkouts,'files':{'trajectory':str(args.trajectory),'clock':str(args.clock),'refined':str(args.refined),'scenario':str(args.scenario)},
            'sha256':sha(args.trajectory),'expected_ros_robots':['rb_0','rb_1','rb_2'],
            'warnings':['No se verificó la coincidencia de posiciones reales de Gazebo con el JSON.',
                       'Sin una instancia ROS 2 activa, no se verifican TF, odometría, /clock ni cmd_vel.',
                       'El preflight no publica comandos ni garantiza ejecución segura.']}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(report,indent=2,ensure_ascii=False))
    return 0 if all(checkouts.values()) else 2

def main():
    root=Path(__file__).resolve().parents[1]
    # Defaults are the validated artifact paths from the project workspace; allow overriding after copying.
    p=argparse.ArgumentParser()
    p.add_argument('--trajectory',type=Path,default=Path('/mnt/data/supermarket_v3_sucesivo_v1/results/seed_100_pickup1_TIMED_INPUT_OFFLINE.npz'))
    p.add_argument('--clock',type=Path,default=Path('/mnt/data/supermarket_v3_sucesivo_v1/results/seed_100_pickup1_clock.json'))
    p.add_argument('--refined',type=Path,default=Path('/mnt/data/supermarket_multigoal_hybrid_v2/results/seed_100_pickup1_refined.npz'))
    p.add_argument('--scenario',type=Path,default=Path('/mnt/data/supermarket_multigoal_hybrid_v2/examples/seed_100.json'))
    p.add_argument('--mission',type=Path,default=Path('/mnt/data/supermarket_multigoal_hybrid_v2/examples/mission_100.json'))
    p.add_argument('--output',type=Path,default=root/'reports'/'static_pickup1_seed100.json')
    p.add_argument('--speed',type=float,default=2.0); p.add_argument('--accel',type=float,default=2.0)
    return check(p.parse_args())
if __name__=='__main__':raise SystemExit(main())
