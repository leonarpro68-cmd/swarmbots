#!/usr/bin/env python3
import argparse,json,pathlib,csv
p=argparse.ArgumentParser();p.add_argument('--fixed',required=True);p.add_argument('--milp',required=True);p.add_argument('--output-dir',required=True);a=p.parse_args()
f=json.loads(pathlib.Path(a.fixed).read_text());m=json.loads(pathlib.Path(a.milp).read_text());out=pathlib.Path(a.output_dir);out.mkdir(parents=True,exist_ok=True)
def delta(x,y):return round(y-x,4) if isinstance(x,(int,float)) and isinstance(y,(int,float)) else None
def sumdist(s):
 v=[r.get('distance_m_observed') for r in s.get('robots',{}).values()];return round(sum(v),4) if len(v)==3 and all(isinstance(x,(float,int)) for x in v) else None
metrics=[('mission_duration_sim_s',f.get('mission_duration_sim_s'),m.get('mission_duration_sim_s')),('total_odom_distance_m',sumdist(f),sumdist(m))]
for robot in ('summit0','summit1','summit2'):
 metrics.append((robot+'_distance_m',f.get('robots',{}).get(robot,{}).get('distance_m_observed'),m.get('robots',{}).get(robot,{}).get('distance_m_observed')))
for stage in ('pickup1','delivery1','pickup2','delivery2'):
 metrics.append((stage+'_navigation_observed_s',f.get('stages',{}).get(stage,{}).get('navigation_observed_s'),m.get('stages',{}).get(stage,{}).get('navigation_observed_s')))
rows=[{'metric':k,'fixed':x,'milp':y,'difference_milp_minus_fixed':delta(x,y),'percent_change':round(100*(y-x)/x,3) if isinstance(x,(int,float)) and isinstance(y,(int,float)) and x else None} for k,x,y in metrics]
with (out/'comparison.csv').open('w',newline='') as h:
 w=csv.DictWriter(h,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
result={'status':'COMPARISON_ONLY','comparability_warning':'Runs must share seed, geometry, controller parameters and recording coverage; this tool cannot verify those conditions.','fixed_issues':f.get('issues',[]),'milp_issues':m.get('issues',[]),'rows':rows}
(out/'comparison.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
print('Saved:',out/'comparison.csv');print('Mission time:',rows[0]);print('Distance:',rows[1])
