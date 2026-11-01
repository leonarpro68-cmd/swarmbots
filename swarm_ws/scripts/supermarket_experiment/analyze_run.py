#!/usr/bin/env python3
"""Offline comparison inputs: odometry and manually confirmed events, no motion commands."""
import argparse,csv,json,math,pathlib,statistics
STAGES=('pickup1','delivery1','pickup2','delivery2')
def read_csv(p):
 with p.open(newline='') as f:return list(csv.DictReader(f))
def summarize(run):
 out={'run':run.name,'status':'OBSERVATIONAL_NOT_PHYSICAL_CERTIFICATE','issues':[],'stages':{},'robots':{},'events':{},'mission_duration_sim_s':None}
 ev=run/'events.csv';odom=run/'odom.csv'
 if not ev.exists():out['issues'].append('MISSING_EVENTS')
 else:
  for row in read_csv(ev):
   if row['robot']=='all':out['events'].setdefault(row['event'],[]).append(float(row['sim_time_s']))
  for stage in STAGES:
   start=out['events'].get(stage+'_nav_start',[]);end=out['events'].get(stage+'_nav_end',[])
   out['stages'][stage]={'navigation_observed_s':end[-1]-start[-1] if len(start)==1 and len(end)==1 and end[0]>=start[0] else None,'physical_confirmed':bool(out['events'].get(stage+'_physical_done'))}
  a=out['events'].get('mission_start',[]);b=out['events'].get('mission_end',[])
  if len(a)==len(b)==1 and b[0]>=a[0]:out['mission_duration_sim_s']=b[0]-a[0]
  else:out['issues'].append('MISSING_OR_INVALID_MISSION_BOUNDARY')
 if not odom.exists():out['issues'].append('MISSING_ODOMETRY')
 else:
  rows=read_csv(odom)
  for robot in ('summit0','summit1','summit2'):
   pts=[]
   for r in rows:
    if r['robot']!=robot:continue
    try:pts.append((float(r['stamp_s']),float(r['x_m']),float(r['y_m']),math.hypot(float(r['vx_local_m_s']),float(r['vy_local_m_s']))))
    except(ValueError,KeyError):continue
   pts.sort();groups=[]
   for p in pts:
    if not groups or p[0]-groups[-1][-1][0]>1.:groups.append([p])
    else:groups[-1].append(p)
   # Do not connect large gaps, detect timestamp resets/duplicate stamps.
   dist=sum(math.hypot(b[1]-a[1],b[2]-a[2]) for g in groups for a,b in zip(g,g[1:]) if 0<b[0]-a[0]<=1.)
   out['robots'][robot]={'samples':len(pts),'distance_m_observed':round(dist,4) if len(pts)>1 else None,'peak_speed_m_s_observed':max((p[3] for p in pts),default=None),'gaps_over_1s':max(0,len(groups)-1)}
   if len(pts)<2:out['issues'].append('INSUFFICIENT_ODOM_'+robot)
 return out
p=argparse.ArgumentParser();p.add_argument('--run-dir',required=True);p.add_argument('--output');a=p.parse_args();run=pathlib.Path(a.run_dir).expanduser().resolve();s=summarize(run);dest=pathlib.Path(a.output) if a.output else run/'summary.json';dest.parent.mkdir(parents=True,exist_ok=True);dest.write_text(json.dumps(s,indent=2,ensure_ascii=False)+'\n');print(json.dumps(s,indent=2,ensure_ascii=False))
