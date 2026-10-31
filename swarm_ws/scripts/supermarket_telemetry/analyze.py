#!/usr/bin/env python3
"""Offline comparisons to NPZ/clock and rectangle geometry (model estimates, NOT physics collision validation)."""
import argparse,csv,json,math,hashlib
from pathlib import Path
import numpy as np

def bezier(ctrl,t):
    n=len(ctrl)-1
    return sum(math.comb(n,k)*(t**k)*((1-t)**(n-k))*ctrl[k] for k in range(n+1))

def rectangle_clearance(x,y,o):
    dx=max(o['xmin']-x,0,x-o['xmax']);dy=max(o['ymin']-y,0,y-o['ymax'])
    outside=math.hypot(dx,dy)
    if outside>0:return outside
    return -min(x-o['xmin'],o['xmax']-x,y-o['ymin'],o['ymax']-y)

def trajectory_at(data,robot,tau):
    c=data[f'R{robot}_controls']; dur=data[f'R{robot}_durations_s'];end=float(sum(dur))
    t=min(max(0.0,tau),end)
    j=int(np.searchsorted(np.cumsum(dur),t,side='right'));j=min(j,len(dur)-1)
    prev=float(sum(dur[:j]));u=max(0.,min(1.,(t-prev)/float(dur[j])))
    return bezier(c[j],u)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--csv',required=True);ap.add_argument('--npz',required=True);ap.add_argument('--clock',required=True);ap.add_argument('--obstacles',required=True);ap.add_argument('--coordinator-report',required=True);ap.add_argument('--out',required=True)
    a=ap.parse_args(); out=Path(a.out);out.mkdir(parents=True,exist_ok=True)
    report=json.loads(Path(a.coordinator_report).read_text());clock=json.loads(Path(a.clock).read_text()); obstacles=json.loads(Path(a.obstacles).read_text());npz=Path(a.npz)
    if hashlib.sha256(npz.read_bytes()).hexdigest()!=clock['input_sha256']:raise SystemExit('NPZ/CLOCK HASH MISMATCH')
    if report.get('trajectory_sha256')!=clock['input_sha256']:raise SystemExit('REAL REPORT / NPZ HASH MISMATCH')
    if report.get('execution_requested') is not True:raise SystemExit('REPORT IS NOT A REAL EXECUTION')
    if report.get('stage')!='pickup1':raise SystemExit('INCORRECT STAGE')
    t0=float(report['start_time_s']); data=np.load(npz)
    knots=clock['timewarp_knots']; times=np.array([k['wall_time_s'] for k in knots],float);taus=np.array([k['tau_original_s'] for k in knots],float)
    per={i:[] for i in range(3)}
    with open(a.csv,newline='') as f:
        for row in csv.DictReader(f):
            label=row['robot'];
            if label not in ('summit0','summit1','summit2'):continue
            i=int(label[-1]); t=float(row['stamp_s']); p=np.array([float(row['x_m']),float(row['y_m'])]);v=math.hypot(float(row['vx_local_m_s']),float(row['vy_local_m_s']))
            per[i].append((t,p,v))
    for i in per:per[i].sort(key=lambda v:v[0])
    bounds=obstacles['bounds'];obs=obstacles['obstacles']; radius=.55
    metrics={};allrows=[];active={}
    for i,rows in per.items():
        full=rows
        if not full: metrics[f'summit{i}']={'error':'No odom records'};continue
        # measure only within planned active duration
        end=float(report['robots'][f'summit{i}']['duration_s'])
        active[i]=[(t,p,v) for t,p,v in rows if t0<=t<=t0+end]
        errs=[];dobs=[];dwalls=[];vel=[]
        for t,p,v in active[i]:
            twall=t-t0
            tau=float(np.interp(twall,times,taus)); target=trajectory_at(data,i,tau)
            err=float(np.linalg.norm(p-target));errs.append(err);vel.append(v)
            near=min(rectangle_clearance(p[0],p[1],o) for o in obs)
            wall=min(p[0]-bounds['xmin'],bounds['xmax']-p[0],p[1]-bounds['ymin'],bounds['ymax']-p[1])
            dobs.append(near-radius);dwalls.append(wall-radius)
            allrows.append((f'summit{i}',t,twall,float(p[0]),float(p[1]),v,float(target[0]),float(target[1]),err,near-radius,wall-radius))
        final_actual=full[-1][1];final_goal=trajectory_at(data,i,1e12)
        # Endpoint here is last recorded (may include other motion if recorder not stopped); choose closest after stage end
        end_samples=[e for e in rows if e[0]>=t0+end]
        final_actual=min(end_samples,key=lambda r:abs(r[0]-(t0+end)))[1] if end_samples else (active[i][-1][1] if active[i] else final_actual)
        metrics[f'summit{i}']={ 'odom_samples_total':len(full),'samples_during_stage':len(active[i]),'max_measured_speed_m_s':max(vel) if vel else None, 'max_tracking_error_m':max(errs) if errs else None,'mean_tracking_error_m':float(np.mean(errs)) if errs else None,'min_body_to_shelf_m_model':min(dobs) if dobs else None,'min_body_to_wall_m_model':min(dwalls) if dwalls else None,'endpoint_error_m_approx':float(np.linalg.norm(final_actual-final_goal)),'planned_duration_s':end}
    pair={}
    # Interpolate pairs onto samples for each robot with strict stamp coverage and nearest time proximity
    for i,j in ((0,1),(0,2),(1,2)):
        aa=active.get(i,[]);bb=active.get(j,[])
        mind=None;n=0
        for t,p,_ in aa:
            if len(bb)<2 or t<bb[0][0] or t>bb[-1][0]:continue
            ts=np.array([x[0] for x in bb]);idx=int(np.searchsorted(ts,t));
            if idx==0 or idx>=len(bb):continue
            t1,p1,_=bb[idx-1];t2,p2,_=bb[idx]
            if max(t-t1,t2-t)>.25:continue
            q=p1+(p2-p1)*(t-t1)/(t2-t1)
            d=float(np.linalg.norm(p-q));mind=d if mind is None else min(mind,d);n+=1
        pair[f'summit{i}_summit{j}']={'min_center_distance_m_interpolated':mind,'min_body_gap_m_model':mind-2*radius if mind is not None else None,'samples_compared':n}
    with open(out/'trajectory_samples.csv','w',newline='') as f:
        w=csv.writer(f);w.writerow(['robot','sim_stamp_s','stage_t_s','actual_x_m','actual_y_m','measured_speed_m_s','expected_x_m','expected_y_m','tracking_error_m','shelf_body_clearance_m_model','wall_body_clearance_m_model']);w.writerows(sorted(allrows,key=lambda row:(row[1],row[0])))
    summary={'status':'ANALYSIS_ONLY_NOT_COLLISION_CERTIFICATE','stage':'pickup1','report_status':report.get('status'),'start_time_sim_s':t0,'robot_metrics':metrics,'pairwise_metrics':pair,'notes':['Uses reported start_time and V3 timewarp knots; temporal interpolation of the knots is approximate.','Shelf and wall distances use rectangular obstacle model and a circular 0.55 m robot radius; these are NOT Gazebo collision contacts or full footprint checks.','Robot-to-robot distances estimated from timestamp-interpolated odometry.','Only runs within each robot planned duration are used for tracking errors.','Requires odometry recorder to be running throughout the complete stage.']}
    (out/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False));print(json.dumps(summary,indent=2,ensure_ascii=False))
if __name__=='__main__':main()
