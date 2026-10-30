#!/usr/bin/env python3
"""D9.5 read-only protocol draft and preflight (NEVER sends robot commands).

Inputs: D9.4 per-stage NON_EXECUTABLE NPZ, conversion reports and D9.2 report.
The preflight revalidates the converted polynomial envelopes, time continuity,
C2 joins, and a conservative inter-robot minimum separation over the entire
horizon (including destination holds). Source files are integrity hashed.
No legacy timed_bezier.v1 command is ever created or published.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np

ROBOTS = ('summit0','summit1','summit2')
STAGES = ('pickup1','delivery1','pickup2','delivery2')
SCHEMA = 'independent_timed_bezier.preview.v1'

def digest(path):
    h = hashlib.sha256()
    with open(path,'rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()

def evaluate(c, durations, t):
    from d94_convert_offline import converted_values
    return converted_values(c,durations,np.asarray(t,dtype=float))

def check_stage(run,stage,output,vmax=1.2,amax=1.0,sep=1.15,step=0.005,join_tol=1e-5):
    base=run/'d94_conversion'
    src=base/f'{stage}_converted_NON_EXECUTABLE.npz'
    rep=base/f'{stage}_conversion_report.json'
    d92=run/'d92_offline_report.json'
    for p in (src,rep,d92):
        if not p.is_file():raise FileNotFoundError(str(p))
    report=json.loads(rep.read_text())
    assert report.get('status')=='CONVERTED_OFFLINE_CHECKS_PASSED', 'D9.4 not passed'
    assert not report.get('ros_executable',True), 'unexpected ROS-executable input'
    assert report.get('stage')==stage, 'stage mismatch'
    source_d92=json.loads(d92.read_text())['stages'][stage]
    assert source_d92['status']=='CERTIFIED_CONSERVATIVE_OFFLINE'
    manifest={'schema':SCHEMA,'stage':stage,'ros_executable':False,
              'publisher_enabled':False,'legacy_v1_compatible':False,
              'source_sha256':{'npz':digest(src),'d94_report':digest(rep),'d92_report':digest(d92)},
              'limits':{'vmax_m_s':vmax,'amax_m_s2':amax,'min_interrobot_m':sep},'robots':{}}
    paths=[];Ts=[]; checks=[]
    with np.load(src,allow_pickle=False) as archive:
        for i,robot in enumerate(ROBOTS):
            c=np.array(archive[f'R{i}_controls'],dtype=np.float64)
            dt=np.array(archive[f'R{i}_durations_s'],dtype=np.float64)
            assert c.ndim==3 and c.shape[1:]==(6,2) and len(dt)==len(c) and len(dt)>0
            assert np.isfinite(c).all() and np.isfinite(dt).all() and np.all(dt>0)
            from d94_convert_offline import polynomial_bounds
            speed,acc=polynomial_bounds(c,dt)
            T=float(np.sum(dt))
            target=float(source_d92['duration_s'][i])
            assert abs(T-target)<1e-6, f'{robot}: duration differs from D9.2'
            # C2 joins evaluated from time-domain quintic endpoint derivatives.
            delta_p=np.linalg.norm(c[1:,0]-c[:-1,5],axis=1)
            vin=5*(c[:-1,5]-c[:-1,4])/dt[:-1,None]
            vout=5*(c[1:,1]-c[1:,0])/dt[1:,None]
            ain=20*(c[:-1,5]-2*c[:-1,4]+c[:-1,3])/dt[:-1,None]**2
            aout=20*(c[1:,2]-2*c[1:,1]+c[1:,0])/dt[1:,None]**2
            dp=float(max(delta_p,default=0.))
            dv=float(max(np.linalg.norm(vin-vout,axis=1),default=0.))
            da=float(max(np.linalg.norm(ain-aout,axis=1),default=0.))
            _,v0,a0=evaluate(c,dt,[0,T])
            end_speed=float(np.linalg.norm(v0[-1]))
            # Actual polynomial terminal speed must be zero (evaluate() clamps t>=T)
            polynomial_end_speed=float(np.linalg.norm(5*(c[-1,5]-c[-1,4])/dt[-1]))
            valid=(speed<=vmax+1e-9 and acc<=amax+1e-9 and
                   dp<=join_tol and dv<=join_tol and da<=join_tol and
                   polynomial_end_speed<=join_tol)
            checks.append(valid)
            manifest['robots'][robot]={
                'segments':len(dt),'duration_s':T,
                'segment_durations_s':dt.tolist(),
                'speed_upper_m_s':speed,'accel_upper_m_s2':acc,
                'max_join_position_m':dp,'max_join_speed_m_s':dv,
                'max_join_accel_m_s2':da,'terminal_speed_m_s':polynomial_end_speed,
                'checks_passed':bool(valid)}
            paths.append((c,dt));Ts.append(T)
    horizon=max(Ts)
    times=np.linspace(0,horizon,max(2,math.ceil(horizon/step)+1))
    xyz=[evaluate(c,dt,times)[0] for c,dt in paths]
    pair=[];lowmin=float('inf')
    sample_h=float(np.max(np.diff(times)))
    for i in range(3):
        for j in range(i+1,3):
            dist=float(np.min(np.linalg.norm(xyz[i]-xyz[j],axis=1)))
            vi=manifest['robots'][ROBOTS[i]]['speed_upper_m_s']
            vj=manifest['robots'][ROBOTS[j]]['speed_upper_m_s']
            lower=dist-(vi+vj)*sample_h/2
            pair.append({'pair':[ROBOTS[i],ROBOTS[j]],'sample_min_m':dist,'conservative_lower_m':lower})
            lowmin=min(lowmin,lower)
    manifest['separation']={'pairs':pair,'conservative_lower_m':lowmin,'step_s':sample_h}
    manifest['checks']={'polynomial_and_joins':all(checks),'separation':lowmin>=sep,
                        'd94_approximation':bool(report['checks']['approximation']),
                        'd94_acceleration_error_continuous_certified':False}
    manifest['status']='PASS_READ_ONLY' if all(checks) and lowmin>=sep and manifest['checks']['d94_approximation'] else 'REJECT'
    manifest['notes']='Preflight only, no ROS commands. Continuous acceleration difference to D9.2 is not yet certified.'
    output.mkdir(parents=True,exist_ok=True)
    target=output/f'{stage}_INDEPENDENT_TIMED_BEZIER_PREVIEW_ONLY.json'
    target.write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest

def inspect_ros_once(seconds=3.0):
    """Optional passive ROS graph probe; no subscriptions/publishers of commands."""
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    rclpy.init()
    node=Node('d95_read_only_preflight')
    seen=set()
    subs=[]
    for robot in ROBOTS:
        subs.append(node.create_subscription(Odometry,f'/{robot}/odom',
                      lambda msg,r=robot:seen.add(r),10))
    import time
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=min(.2,max(0,deadline-time.monotonic())))
    result={'odom_received':{r:r in seen for r in ROBOTS},'all_odom_seen':len(seen)==3,
            'command_publishers_created':0,'commands_sent':0}
    node.destroy_node();rclpy.shutdown()
    return result

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--stage',choices=STAGES)
    p.add_argument('--step',type=float,default=.005)
    p.add_argument('--ros-observe',action='store_true',help='Read-only odometry probe, requires sourced ROS 2')
    p.add_argument('--ros-seconds',type=float,default=3.)
    args=p.parse_args()
    if not 0<args.step<=.04:p.error('--step must be in (0,0.04]')
    out=args.run/'d95_preflight'
    reports={}
    for stage in ([args.stage] if args.stage else STAGES):
        m=check_stage(args.run,stage,out,step=args.step)
        reports[stage]={'status':m['status'],'separation_lower_m':m['separation']['conservative_lower_m']}
        print(stage,m['status'], 'separation_lower=',round(m['separation']['conservative_lower_m'],4))
    result={'schema':SCHEMA,'ros_executable':False,'stage_reports':reports}
    if args.ros_observe:result['ros_observation']=inspect_ros_once(args.ros_seconds)
    (out/'d95_summary.json').write_text(json.dumps(result,indent=2)+'\n')
    if any(x['status']!='PASS_READ_ONLY' for x in reports.values()):raise SystemExit(2)
    print('Saved',out)
if __name__=='__main__':main()
