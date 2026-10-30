#!/usr/bin/env python3
"""D9.8C.1: read-only live pickup/load snapshot; never commands robots or commits ledger."""
import argparse, json, math, subprocess
from pathlib import Path
from d98b_grasp_preflight import parse_gz_pose_info, read_ros_odom
ROBOTS=('summit0','summit1','summit2')

def assess(names, poses, objects, *, palm=.45, radius=.45, speed_limit=.03, other_margin=.07):
    issues=[]; result={}
    if len(names)!=3 or len(set(names))!=3: issues.append('INVALID_ASSIGNED_OBJECTS')
    for r,n in zip(ROBOTS,names):
        if r not in poses:issues.append(f'{r}:NO_FRESH_ODOMETRY');continue
        if n not in objects:issues.append(f'{r}:ASSIGNED_OBJECT_NOT_IN_POSE_INFO');continue
        x,y,yaw,speed=poses[r]
        if not all(math.isfinite(v) for v in (x,y,yaw,speed)):
            issues.append(f'{r}:INVALID_ODOMETRY');continue
        tx,ty=x+palm*math.cos(yaw),y+palm*math.sin(yaw)
        dist=math.hypot(objects[n][0]-tx,objects[n][1]-ty)
        others=sorted((math.dist((tx,ty),xy),k) for k,xy in objects.items() if k!=n)
        if speed>speed_limit:issues.append(f'{r}:ROBOT_MOVING')
        if dist>radius:issues.append(f'{r}:OBJECT_NOT_NEAR_TIP')
        if others and others[0][0] - dist < other_margin:issues.append(f'{r}:CLOSER_OR_AMBIGUOUS_TRASH')
        result[r]={'object':n,'odom_xy_m':[x,y],'odom_yaw_rad':yaw,'observed_speed_m_s':speed,
                   'tip_xy_m':[tx,ty],'object_xy_m':objects[n], 'tip_object_distance_m':round(dist,6),
                   'distance_compatible_with_load':dist<=radius}
    return {'schema':'swarm.d98c1.live_delivery_snapshot.v1',
            'status':'LOAD_GEOMETRY_COMPATIBLE_NOT_JOINT_CERTIFIED' if not issues else 'BLOCKED',
            'issues':issues,'robots':result,
            'proposed_pickup_objects':dict(zip(ROBOTS,names)),
            'gazebo_joint_attachment_verified':False,
            'mission_ledger_updated':False,'delivery1_planned':False,'delivery1_execution_authorized':False,
            'commands_sent':0,'publishers_created':0,
            'note':'Proximity cannot prove DetachableJoint state. Grasp manager held state is private; retrieve live joint evidence or instrument manager before mission commit.'}

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--handoff',required=True,type=Path);p.add_argument('--world',default='world_demo')
    p.add_argument('--output',type=Path);p.add_argument('--palm-forward',type=float,default=.45)
    p.add_argument('--radius',type=float,default=.45)
    a=p.parse_args(); h=json.loads(a.handoff.read_text())
    if h.get('status')!='READY_FOR_PHYSICAL_CONFIRMATION_ONLY' or h.get('stage')!='pickup1':
        p.error('invalid original pickup1 handoff')
    names=h.get('stage_object_names_from_D8_plan',[])
    gz=subprocess.run(['gz','topic','-e','-t',f'/world/{a.world}/pose/info','-n','1'],
                      capture_output=True,text=True,timeout=15,check=True)
    obs=parse_gz_pose_info(gz.stdout)
    odom=read_ros_odom(timeout=8.)
    report=assess(names,odom,obs,palm=a.palm_forward,radius=a.radius)
    report['world']=a.world
    out=a.output or a.handoff.parent.parent/'d98c1'/'delivery1_live_snapshot.json'
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if not report['issues'] else 2
if __name__=='__main__':raise SystemExit(main())
