#!/usr/bin/env python3
"""D9.8B.1: preorient -> RGB-D approach -> grip, reusing D8 nodes.

Preview is inert. Explicit --execute-in-gazebo enables commands on SIMULATION ONLY.
NEVER changes the D6 mission ledger or releases trash.
"""
import argparse
import json
import math
import subprocess
import time
from pathlib import Path

from d98b_grasp_preflight import ROBOTS, parse_gz_pose_info


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def plan_turn(names, poses, objects, *, min_distance=.55, max_distance=2.2,
              margin=.07, max_linear=.03, max_angular=.05, palm_forward=.45, selected=None):
    """Eligibility for the *first* phase, NOT immediate gripping.

    poses dict values: (x,y,yaw,linear_speed,angular_speed).  All three robots
    must be in the scene and stopped. Nearest target from center must belong to
    the expected D8 object set and be one-to-one. Recheck the actual fingertip
    distance and all other objects immediately AFTER visual_grasp.
    """
    reasons=[]
    selected=tuple(selected) if selected is not None else ROBOTS
    if not selected or any(r not in ROBOTS for r in selected) or len(set(selected))!=len(selected):
        reasons.append('INVALID_SELECTED_ROBOTS')
    if not isinstance(names,list) or len(names)!=3 or len(set(names))!=3 or not all(isinstance(n,str) for n in names):
        reasons.append('INVALID_STAGE_NAMES')
    if set(poses)!=set(ROBOTS):reasons.append('MISSING_OR_EXTRA_ROBOT_ODOMETRY')
    if any(n not in objects for n in names):reasons.append('MISSING_PLANNED_OBJECT_IN_GAZEBO')
    if reasons:return {'status':'BLOCKED','reasons':reasons,'mapping':{},'robots':{}}
    mapping={};out={}
    for r in ROBOTS:
        pose=poses[r]
        if len(pose)!=5 or not all(math.isfinite(v) for v in pose):
            reasons.append(r+':INVALID_ODOMETRY');continue
        x,y,yaw,v,w=pose
        if v>max_linear or abs(w)>max_angular:reasons.append(r+':NOT_STOPPED')
        if r not in selected:continue
        nearest=sorted(((math.hypot(ox-x,oy-y),n) for n,(ox,oy) in objects.items()))
        if not nearest:
            reasons.append(r+':NO_TRASH');continue
        dist,name=nearest[0]
        if name not in names:reasons.append(r+':NEAREST_NOT_PLANNED')
        if len(nearest)>1 and nearest[1][0]-dist < margin:
            reasons.append(r+':OBJECT_AMBIGUOUS')
        if dist < min_distance:reasons.append(r+':TOO_CLOSE_FOR_VISUAL_APPROACH')
        if dist > max_distance:reasons.append(r+':TOO_FAR_FOR_VISUAL_APPROACH')
        mapping[r]=name
        tx,ty=objects[name]
        target_yaw=math.atan2(ty-y,tx-x)
        out[r]={'object':name,'center_distance_m':round(dist,6),
                'initial_yaw_rad':round(yaw,6),'target_yaw_rad':target_yaw,
                'turn_degrees':round(math.degrees(wrap(target_yaw-yaw)),3),
                'tip_distance_after_turn_without_approach_m':round(abs(dist-palm_forward),6),
                'requires_rgbd_approach': abs(dist-palm_forward)>.45}
    if len(set(mapping.values()))!=len(selected):reasons.append('NOT_ONE_TO_ONE_MAPPING')
    if mapping and mapping!={r:n for r,n in zip(ROBOTS,names) if r in selected}:
        reasons.append('ROBOT_ASSIGNMENT_DIFFERS_FROM_D8_STAGE_ORDER')
    return {'status':'READY_FOR_TURN_AND_RGBD_ONLY' if not reasons else 'BLOCKED',
            'reasons':reasons,'mapping':mapping,'robots':out,
            'commands_sent':0,'mission_ledger_updated':False,
            'note':'Center-distance check precedes the yaw turn and RGB-D approach; fingertip check follows alignment.'}


def tip_check(robot, name, pose, objects, radius=.45, margin=.07, palm_forward=.45):
    """Immediately-before-grasp verification against EVERY Gazebo trash object."""
    x,y,yaw,v,w=pose
    if not all(map(math.isfinite,pose)) or v>.055 or abs(w)>.08:
        return {'ok':False,'reason':'NOT_STOPPED_OR_INVALID_ODOM'}
    tx=x+palm_forward*math.cos(yaw);ty=y+palm_forward*math.sin(yaw)
    ordered=sorted((math.hypot(ox-tx,oy-ty),n) for n,(ox,oy) in objects.items())
    if not ordered:return {'ok':False,'reason':'NO_OBJECTS'}
    d,nearest=ordered[0]
    ok=(nearest==name and d<=radius and (len(ordered)<2 or ordered[1][0]-d>=margin))
    return {'ok':ok, 'reason':'READY_TO_GRASP' if ok else 'WRONG_OBJECT_OR_DISTANCE',
            'nearest_object':nearest,'distance_m':round(d,5),'requested_object':name}


def gazebo_objects(world):
    cmd=['gz','topic','-e','-t',f'/world/{world}/pose/info','-n','1']
    p=subprocess.run(cmd,capture_output=True,text=True,check=True,timeout=12)
    objects=parse_gz_pose_info(p.stdout)
    if len(objects)<3:raise RuntimeError('GAZEBO_POSE_INFO_OBJECTS_MISSING')
    return objects


class RosSession:
    def __init__(self):
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import Int8, Float64, Empty
        from geometry_msgs.msg import PoseStamped
        self.rclpy=rclpy;self.Float64=Float64;self.Empty=Empty;self.PoseStamped=PoseStamped
        rclpy.init()
        self.node=Node('d98b1_d8_physical_pickup')
        self.pose={};self.odom_seen={};self.results={}
        self.pubs={};self.subs=[]
        for robot in ROBOTS:
            self.subs.append(self.node.create_subscription(Odometry,f'/{robot}/odom',
                                 lambda msg,r=robot:self._odom(r,msg),qos_profile_sensor_data))
            for phase, topic in (('turn','local_turn/result'),('visual','visual_grasp/result'),('grasp','gripper/result')):
                self.subs.append(self.node.create_subscription(Int8,f'/{robot}/{topic}',
                                     lambda msg,r=robot,p=phase:self.results.__setitem__((r,p),int(msg.data)),10))
            self.pubs[robot]={
                'turn':self.node.create_publisher(Float64,f'/{robot}/local_turn/target_yaw',10),
                'visual':self.node.create_publisher(PoseStamped,f'/{robot}/visual_grasp/target',10),
                'grasp':self.node.create_publisher(Empty,f'/{robot}/gripper/grasp',10)}

    def _odom(self,r,m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        v=m.twist.twist.linear;w=m.twist.twist.angular
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.pose[r]=(float(p.x),float(p.y),yaw,math.hypot(v.x,v.y),float(w.z))
        self.odom_seen[r]=time.monotonic()

    def spin_until(self,predicate,seconds):
        deadline=time.monotonic()+seconds
        while self.rclpy.ok() and time.monotonic()<deadline:
            self.rclpy.spin_once(self.node,timeout_sec=.08)
            if predicate():return True
        return False

    def fresh(self,robot,age=.5):
        return robot in self.pose and time.monotonic()-self.odom_seen[robot]<age

    def wait_odom(self):
        return self.spin_until(lambda:all(self.fresh(r) for r in ROBOTS),10.)

    def subscribers_ready(self,robot):
        for p in ('turn','visual','grasp'):
            if not self.spin_until(lambda key=p:self.pubs[robot][key].get_subscription_count()>0,4.):
                raise RuntimeError(f'{robot}:{p}:D8_SUBSCRIBER_MISSING')
        # D8 grasp path follower MUST be the *sole* active publisher for body cmds.
        endpoints=self.node.get_publishers_info_by_topic(f'/{robot}/cmd_vel')
        if len(endpoints)!=1 or 'path_follower' not in endpoints[0].node_name:
            raise RuntimeError(f'{robot}:CMD_VEL_PUBLISHER_OWNERSHIP_UNSAFE:{[(e.node_name,e.node_namespace) for e in endpoints]}')

    def publish_and_wait(self,robot,phase,message,timeout):
        self.results.pop((robot,phase),None)
        self.pubs[robot][phase].publish(message)
        if not self.spin_until(lambda:(robot,phase) in self.results,timeout):
            raise RuntimeError(f'{robot}:{phase}:ACK_TIMEOUT')
        ack=self.results[(robot,phase)]
        if ack!=1:raise RuntimeError(f'{robot}:{phase}:ACK_FAILED:{ack}')

    def close(self):
        try:self.node.destroy_node()
        finally:
            if self.rclpy.ok():self.rclpy.shutdown()


def execute_robot(sess, robot, target, objects, world):
    """Executed only after phase-A preflight. No return or mission ledger writes."""
    stage={'robot':robot,'object':target,'steps':[]}
    sess.subscribers_ready(robot)
    if not sess.fresh(robot):raise RuntimeError(robot+':STALE_ODOMETRY_BEFORE_TURN')
    x0,y0,yaw0,v,w=sess.pose[robot]
    if v>.03 or abs(w)>.05:raise RuntimeError(robot+':NOT_STOPPED_BEFORE_TURN')
    tx,ty=objects[target]
    yaw=math.atan2(ty-y0,tx-x0)
    sess.publish_and_wait(robot,'turn',sess.Float64(data=yaw),19.)
    if not sess.spin_until(lambda:sess.fresh(robot),2.):raise RuntimeError(robot+':NO_ODOM_AFTER_TURN')
    x1,y1,heading,vs,ws=sess.pose[robot]
    angle_err=abs(wrap(yaw-heading))
    drift=math.hypot(x1-x0,y1-y0)
    stage['steps'].append({'phase':'TURN_ACK','yaw_error_deg':math.degrees(angle_err),'xy_drift_m':drift})
    if angle_err>.10 or drift>.10 or vs>.05 or abs(ws)>.08:
        raise RuntimeError(robot+':TURN_POSITION_OR_YAW_OUTSIDE_TOLERANCE')
    # Reconfirm target has not moved before turning over control to the camera.
    live=gazebo_objects(world)
    if target not in live or math.dist(live[target],objects[target])>.15:
        raise RuntimeError(robot+':TARGET_MOVED_AFTER_TURN')
    msg=sess.PoseStamped(); msg.header.frame_id=f'{robot}/odom'
    msg.header.stamp=sess.node.get_clock().now().to_msg()
    msg.pose.position.x,msg.pose.position.y=live[target]
    msg.pose.orientation.w=1.
    sess.publish_and_wait(robot,'visual',msg,25.)
    stage['steps'].append({'phase':'RGBD_ALIGN_ACK'})
    # D8 follower must consume visual_grasp/active=False before any next action.
    sess.spin_until(lambda:False,.4)
    if not sess.spin_until(lambda:sess.fresh(robot) and sess.pose[robot][3]<.05,4.):
        raise RuntimeError(robot+':ROBOT_NOT_STOPPED_AFTER_VISUAL')
    live=gazebo_objects(world)
    check=tip_check(robot,target,sess.pose[robot],live)
    stage['final_tip_check']=check
    if not check['ok']:raise RuntimeError(robot+':FINAL_TIP_CHECK_BLOCKED:'+check['reason'])
    sess.publish_and_wait(robot,'grasp',sess.Empty(),18.)
    stage['steps'].append({'phase':'GRASP_ACK_ATTACHED_BY_D8_MANAGER'})
    stage['status']='GRASP_MANAGER_ACK_NO_LEDGER_COMMIT'
    return stage


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--handoff',required=True,type=Path)
    p.add_argument('--world',required=True)
    p.add_argument('--robot',choices=ROBOTS+('all',),default='summit0')
    p.add_argument('--execute-in-gazebo',action='store_true')
    p.add_argument('--output',type=Path)
    a=p.parse_args()
    handoff=json.loads(a.handoff.read_text())
    if handoff.get('status')!='READY_FOR_PHYSICAL_CONFIRMATION_ONLY' or handoff.get('stage')!='pickup1':
        raise SystemExit('BLOCKED: valid pickup1 handoff is required')
    report={'schema':'swarm.d98b1.turn_visual_grasp.v1','requested_robot':a.robot,
            'world':a.world,'commands_may_have_been_sent':False,'mission_ledger_updated':False,
            'execution_authorized':a.execute_in_gazebo,'status':'BLOCKED','robot_results':[]}
    out=a.output or a.handoff.parent.parent/'d98b'/'pickup1_turn_visual_grasp.json'
    sess=None
    try:
        objects=gazebo_objects(a.world)
        sess=RosSession()
        if not sess.wait_odom():raise RuntimeError('ODOMETRY_TIMEOUT')
        chosen=ROBOTS if a.robot=='all' else (a.robot,)
        initial=plan_turn(handoff['stage_object_names_from_D8_plan'],sess.pose,objects,selected=chosen)
        report['preflight']=initial
        if initial['status']!='READY_FOR_TURN_AND_RGBD_ONLY':
            raise RuntimeError('PREORIENTATION_PREFLIGHT_BLOCKED')
        report['status']='READY_FOR_TURN_AND_RGBD_ONLY'
        if a.execute_in_gazebo:
            # Check ownership and all 3 stopped before the first command.
            robots=ROBOTS if a.robot=='all' else (a.robot,)
            for robot in robots:
                # Re-read live scene and recalc mapping just before each robot.
                objects=gazebo_objects(a.world)
                if not sess.wait_odom():raise RuntimeError('ODOMETRY_TIMEOUT_BETWEEN_ROBOTS')
                if robot not in initial['mapping']:
                    raise RuntimeError(robot+':ASSIGNMENT_MISSING')
                expected=initial['mapping'][robot]
                current=sess.pose[robot]
                closest=min(objects,key=lambda n:math.hypot(objects[n][0]-current[0],objects[n][1]-current[1]))
                if closest!=expected:raise RuntimeError(robot+':TARGET_NO_LONGER_NEAREST')
                try:
                    report['commands_may_have_been_sent']=True
                    report['robot_results'].append(execute_robot(sess,robot,expected,objects,a.world))
                except Exception as e:
                    report['robot_results'].append({'robot':robot,'object':expected,'status':'FAILED','error':str(e)})
                    raise
            report['status']='GRASP_ACK_RECEIVED_NO_LEDGER_COMMIT'
    except Exception as e:
        report['status']='BLOCKED_OR_FAILED'
        report['reason']=str(e)
    finally:
        if sess is not None:sess.close()
        out.parent.mkdir(parents=True,exist_ok=True)
        out.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
        print(json.dumps(report,indent=2,ensure_ascii=False))
    return 0 if report['status'] in ('READY_FOR_TURN_AND_RGBD_ONLY','GRASP_ACK_RECEIVED_NO_LEDGER_COMMIT') else 2


if __name__=='__main__':raise SystemExit(main())
