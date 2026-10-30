#!/usr/bin/env python3
"""D9.9.5 — Independent read-only preflight for converted D9.9 collective paths.

NO ROS, no motor commands, no Gazebo. Checks source/converted hashes, elapsed
clock, C2 joins, start/finish conditions, dynamic hull, approximation error
certificates, and exact temporal separation with conservative interpolation.
Explicitly does not authorize execution until map clearance is recertified.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from d9_offline_retime import load, active
from d99_collective_phase_retime import solve
from d994_convert_collective import (sha256, converted_joins, error_bounds,
                                     converted_values, polynomial_bounds)


def preflight(run,stage,conversion,out,sep=1.15,vmax=1.2,amax=1.,step=.0005,
              eps_pos=.03,eps_speed=.12,eps_accel=.25):
    src=run/'mission'/stage/'protocol'/f'{stage}_protocol_controls.npz'
    npz=conversion/f'{stage}_d994_NON_EXECUTABLE.npz'
    rep=conversion/f'{stage}_d994_conversion_report.json'
    r=json.loads(rep.read_text())
    fails=[]
    if r['stage']!=stage or r['schema']!='d99_collective_timed_quintic.preview.v1':fails.append('wrong_stage_or_schema')
    if r['ros_executable'] or r['obstacle_map_recertified']:fails.append('incorrect_executable_flag')
    if r['source_sha256']['original_protocol_controls']!=sha256(src):fails.append('original_protocol_hash_mismatch')
    if r['source_sha256']['converted_npz']!=sha256(npz):fails.append('converted_protocol_hash_mismatch')
    cs,dt=load(run,stage)
    base=solve(cs,dt)
    if base.get('status')!='CERTIFIED_OFFLINE_COLLECTIVE':fails.append('source_collective_rejected')
    if abs(r['source_duration_s']-base['optimized_makespan_s'])>1e-6:fails.append('source_duration_mismatch')
    if len(r['phase_scales'])!=len(base['phase_scales_after_events']) or not np.allclose(r['phase_scales'],base['phase_scales_after_events'],rtol=0,atol=1e-9):
        fails.append('phase_scale_mismatch')
    events=list(zip(base['events_q'],base['transition_width_in_original_pieces']))
    targets=base['phase_scales_after_events']
    polygons=[];durations=[];certs=[]
    with np.load(npz,allow_pickle=False) as data:
        for i in range(3):
            p=np.asarray(data[f'R{i}_controls'],dtype=float)
            h=np.asarray(data[f'R{i}_durations_s'],dtype=float)
            if p.ndim!=3 or p.shape[1:]!=(6,2) or len(p)!=len(h) or len(h)<1 or not np.isfinite(p).all() or not np.isfinite(h).all() or np.any(h<=0):
                fails.append(f'R{i}:invalid_quintic');continue
            expected=base['finishing_times_s'][str(i)]
            if abs(float(h.sum())-expected)>1e-7:fails.append(f'R{i}:finish_time')
            joins=converted_joins(p,h)
            if max(joins.values())>1e-5:fails.append(f'R{i}:C2')
            v,a=polynomial_bounds(p,h)
            if v>vmax+1e-9 or a>amax+1e-9:fails.append(f'R{i}:dynamic_limits')
            if np.linalg.norm(p[0,0]-cs[i][0,0])>1e-6 or np.linalg.norm(p[-1,-1]-cs[i][active(cs[i])-1,-1])>1e-6:
                fails.append(f'R{i}:start_or_goal')
            if max(np.linalg.norm(5*(p[0,1]-p[0,0])/h[0]),
                   np.linalg.norm(5*(p[-1,-1]-p[-1,-2])/h[-1]),
                   np.linalg.norm(20*(p[0,2]-2*p[0,1]+p[0,0])/h[0]**2),
                   np.linalg.norm(20*(p[-1,5]-2*p[-1,4]+p[-1,3])/h[-1]**2))>1e-5:
                fails.append(f'R{i}:not_stopped')
            e=error_bounds(cs[i],active(cs[i]),dt,events,targets,p,h,step)
            sv=base['dynamics'][i]['v_upper_m_s'];sa=base['dynamics'][i]['a_upper_m_s2']
            ep=e['sampled_position_error_m']+(sv+v)*e['sample_step_max_s']/2
            ev=e['sampled_velocity_error_m_s']+(sa+a)*e['sample_step_max_s']/2
            ea=e['accel_error_upper_m_s2']
            if ep>eps_pos or ev>eps_speed or ea>eps_accel:fails.append(f'R{i}:approximation')
            polygons.append(p);durations.append(h)
            certs.append({'robot':i,'finish_s':float(h.sum()),'C2_joins':joins,
                         'speed_upper_m_s':v,'accel_upper_m_s2':a,
                         'position_error_upper_m':ep,'velocity_error_upper_m_s':ev,
                         'accel_error_upper_m_s2':ea,'source_jerk_bound_m_s3':e['source_jerk_upper_m_s3'],
                         'converted_jerk_bound_m_s3':e['converted_jerk_upper_m_s3']})
    pair=[]; min_distance=float('inf')
    if len(polygons)==3:
        T=max(float(h.sum()) for h in durations)
        times=np.linspace(0,T,max(2,int(np.ceil(T/.005))+1))
        hh=float(np.max(np.diff(times)))
        pos=[converted_values(p,h,times)[0] for p,h in zip(polygons,durations)]
        for i in range(3):
            for j in range(i+1,3):
                sampled=float(np.linalg.norm(pos[i]-pos[j],axis=1).min())
                low=sampled-(certs[i]['speed_upper_m_s']+certs[j]['speed_upper_m_s'])*hh/2
                pair.append({'robots':[i,j],'sampled_m':sampled,'certified_lower_m':low})
                min_distance=min(min_distance,low)
                if low<sep-1e-9:fails.append(f'R{i}-R{j}:separation')
        # Separate conversion-vs-source perturbation argument.
        for i in range(3):
            for j in range(i+1,3):
                alt=base['geometry']['certified_lower_m']-certs[i]['position_error_upper_m']-certs[j]['position_error_upper_m']
                if alt<sep-1e-9:fails.append(f'R{i}-R{j}:source_perturbation')
    # This is not a new map check. It is a mathematically inherited margin:
    # any converted curve is within ep_i of its source curve at ALL times;
    # distance to any fixed obstacle/wall is 1-Lipschitz in position.
    original_report=run/'mission'/stage/'protocol'/f'{stage}_protocol_report.json'
    original_cert=json.loads(original_report.read_text())['stages'][stage]['hybrid_v2']
    extra_obst=float(original_cert['min_obstacle_extra_clearance_m'])
    extra_wall=float(original_cert['min_wall_extra_clearance_m'])
    max_deviation=max((b['position_error_upper_m'] for b in certs),default=float('inf'))
    inherited={
       'original_hybrid_v2_certified':original_cert.get('certified') is True,
       'original_requested_margin_m':original_cert['operational_margin_m'],
       'source_obstacle_extra_clearance_m':extra_obst,
       'source_wall_extra_clearance_m':extra_wall,
       'converted_max_position_error_upper_m':max_deviation,
       'remaining_obstacle_extra_margin_lower_m':extra_obst-max_deviation,
       'remaining_wall_extra_margin_lower_m':extra_wall-max_deviation,
       'valid_only_for_same_static_map_and_frame':True,
       'passed':original_cert.get('certified') is True and min(extra_obst,extra_wall)>max_deviation
    }
    if not inherited['passed']:
        fails.append('inherited_static_map_clearance')
    result={'stage':stage,'schema':'d99_collective_timed_quintic.preview.v1',
      'status':'PASS_OFFLINE_READ_ONLY' if not fails else 'REJECT',
      'ros_executable':False,'gazebo_authorized':False,'obstacle_map_recertified':False,
      'failures':fails,'robot_checks':certs,'separation_pairs':pair,
      'separation_lower_m':min_distance,'inherited_source_map_clearance':inherited,
      'source_duration_s':base['optimized_makespan_s'],
      'warnings':['Obstacle clearance is inherited from original Hybrid V2 for same static map/frame; not independently re-mapped',
                  'Robot controller behavior not tested'],
      'checked_sha256':{'source':sha256(src),'converted':sha256(npz),'conversion_report':sha256(rep)}}
    out.mkdir(parents=True,exist_ok=True)
    (out/f'{stage}_d995_preflight.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--run',type=Path,required=True)
    ap.add_argument('--stage',required=True)
    ap.add_argument('--conversion',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    args=ap.parse_args()
    r=preflight(args.run,args.stage,args.conversion,args.out)
    print(args.stage,r['status'],'separation_lower=',r['separation_lower_m'],r['failures'])
    if r['status']!='PASS_OFFLINE_READ_ONLY':raise SystemExit(2)

if __name__=='__main__':main()
