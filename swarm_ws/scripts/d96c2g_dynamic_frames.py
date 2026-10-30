#!/usr/bin/env python3
"""D9.6C.2G: passive time-matched Gazebo world pose vs ROS odom comparison.

Never creates a publisher, issues Gazebo service calls, publishes TF or commands.
The result is observational only, not a TF certificate or execution authority.
"""
import argparse
import bisect
import csv
import json
import math
import re
import subprocess
import threading
import time
from pathlib import Path

ROBOTS = ('summit0', 'summit1', 'summit2')
_FLOAT = r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?'


def nested_blocks(text, name):
    pat = re.compile(r'(?<![\w])' + re.escape(name) + r'\s*\{')
    out = []
    for match in pat.finditer(text):
        start, depth, quoted, escaped = match.end(), 1, False, False
        for k in range(start, len(text)):
            c = text[k]
            if quoted:
                if escaped: escaped = False
                elif c == '\\': escaped = True
                elif c == '"': quoted = False
            elif c == '"': quoted = True
            elif c == '{': depth += 1
            elif c == '}':
                depth -= 1
                if depth == 0:
                    out.append(text[start:k]); break
    return out


def number(text, field, default=None):
    m = re.search(r'\b' + re.escape(field) + r'\s*:\s*(' + _FLOAT + r')\b', text)
    return float(m.group(1)) if m else default


def timestamp(block):
    stamps = nested_blocks(block, 'stamp')
    if not stamps: return None
    sec = number(stamps[0], 'sec')
    if sec is None: return None
    nano = number(stamps[0], 'nsec', 0.)
    stamp = sec + 1e-9 * nano
    return stamp if math.isfinite(stamp) and stamp >= 0 else None


def yaw(q):
    x,y,z,w = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def angle(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def parse_pose(text, inherited_stamp=None):
    name = re.search(r'\bname\s*:\s*"(summit[012])"', text)
    if not name: return None
    positions = nested_blocks(text, 'position')
    orientations = nested_blocks(text, 'orientation')
    if not positions or not orientations: return None
    p = [number(positions[0], c) for c in ('x', 'y', 'z')]
    q = [number(orientations[0], c, 0. if c != 'w' else 1.) for c in ('x', 'y', 'z', 'w')]
    if None in p or not all(math.isfinite(x) for x in p+q): return None
    q_norm = math.sqrt(sum(x*x for x in q))
    if not (0.99 < q_norm < 1.01): return None
    q = [x/q_norm for x in q]
    headers = nested_blocks(text, 'header')
    pose_stamp = timestamp(headers[0]) if headers else None
    t = pose_stamp if pose_stamp is not None else inherited_stamp
    return {'robot': name.group(1), 'stamp_s': t, 'xyz':p, 'yaw_rad':yaw(q),
            'quaternion_xyzw':q, 'stamp_source':'pose.header' if pose_stamp is not None else ('vector.header' if inherited_stamp is not None else 'missing')}


def brace_delta(text):
    depth, quoted, escaped = 0, False, False
    for c in text:
        if quoted:
            if escaped:escaped=False
            elif c=='\\':escaped=True
            elif c=='"':quoted=False
        elif c=='"':quoted=True
        elif c=='{':depth+=1
        elif c=='}':depth-=1
    return depth


class GazeboStreamParser:
    """Parse top-level `header {...}` and `pose {...}` fields from gz topic -e.

    It does not require Gazebo's output to contain `---` delimiters. Pose.header
    timestamps take priority over Pose_V.header (which may be empty).
    """
    def __init__(self, max_samples_per_robot=20000):
        self.samples={r:[] for r in ROBOTS}
        self.top_stamp=None
        self.current=None
        self.buffer=[]
        self.depth=0
        self.invalid=0
        self.unstamped=0
        self.truncated=0
        self.max_samples=max_samples_per_robot

    def feed(self, line):
        if self.current is None:
            if re.match(r'^\s*-{3,}\s*$',line):
                self.top_stamp=None
                return
            found = re.match(r'^\s*(header|pose)\s*\{',line)
            if not found:return
            self.current=found.group(1)
            self.buffer=[line]
            self.depth=brace_delta(line)
        else:
            self.buffer.append(line)
            self.depth+=brace_delta(line)
        if self.depth<=0:
            value=''.join(self.buffer)
            if self.current=='header':
                self.top_stamp=timestamp(value)
            elif self.current=='pose':
                p=parse_pose(value,self.top_stamp)
                if p:
                    if p['stamp_s'] is None:self.unstamped+=1
                    if len(self.samples[p['robot']])<self.max_samples:
                        self.samples[p['robot']].append(p)
                    else:self.truncated+=1
                elif re.search(r'\bname\s*:\s*"summit[012]"',value):
                    self.invalid+=1
            self.current=None;self.buffer=[];self.depth=0


def record_odom(msg):
    pos=msg.pose.pose.position
    quat=msg.pose.pose.orientation
    q=[quat.x,quat.y,quat.z,quat.w]
    n=math.sqrt(sum(x*x for x in q))
    if not (math.isfinite(n) and .99<n<1.01):return None
    q=[x/n for x in q]
    return {'stamp_s':msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9,
            'xyz':[pos.x,pos.y,pos.z], 'yaw_rad':yaw(q),
            'frame_id':msg.header.frame_id, 'child_frame_id':msg.child_frame_id,
            'recv_mono_s':time.monotonic()}


class ClockAccumulator:
    """Constant-memory, full-window /clock integrity tracker.

    Unlike keeping only the first 40,000 Clock messages, every observed sample
    contributes to the monotonicity check and count. Designed for long missions.
    """
    def __init__(self):
        self.count=0
        self.first_s=None
        self.last_s=None
        self.resets=0
        self.max_gap_s=0.0
        self.nonfinite=0

    def append(self, stamp_s):
        self.count+=1
        if not math.isfinite(stamp_s) or stamp_s<0:
            self.nonfinite+=1
            return
        if self.first_s is None:
            self.first_s=stamp_s
        if self.last_s is not None:
            delta=stamp_s-self.last_s
            if delta<-1e-9:self.resets+=1
            if delta>self.max_gap_s:self.max_gap_s=delta
        self.last_s=stamp_s

    def __len__(self):return self.count


def summarize_clock(clock):
    """Return full-window integrity for either the accumulator or test lists."""
    if isinstance(clock,ClockAccumulator):
        bounds=(None if clock.first_s is None or clock.last_s is None
                else [clock.first_s,clock.last_s])
        good=(clock.count>=2 and clock.nonfinite==0 and clock.resets==0
              and bounds is not None and bounds[1]>bounds[0])
        details={'reset_count':clock.resets,'invalid_stamp_count':clock.nonfinite,
                 'max_consecutive_sample_gap_s':clock.max_gap_s}
        return good,bounds,details
    values=list(clock)
    finite=all(isinstance(v,(int,float)) and math.isfinite(v) and v>=0 for v in values)
    resets=sum(b<a-1e-9 for a,b in zip(values,values[1:]))
    bounds=[values[0],values[-1]] if values else None
    gaps=[b-a for a,b in zip(values,values[1:]) if b>=a]
    good=len(values)>=2 and finite and resets==0 and bounds[1]>bounds[0]
    return good,bounds,{'reset_count':resets,'invalid_stamp_count':sum(not isinstance(v,(int,float)) or not math.isfinite(v) or v<0 for v in values),
                         'max_consecutive_sample_gap_s':max(gaps,default=0.)}


def capture(duration, gz_topic):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    check=subprocess.run(['gz','topic','-l'],capture_output=True,text=True,timeout=12)
    if check.returncode or gz_topic not in check.stdout.splitlines():
        raise RuntimeError('Gazebo world pose topic unavailable: '+gz_topic)
    parser=GazeboStreamParser()
    rclpy.init(args=None)
    class PassiveObserver(Node):
        def __init__(self):
            super().__init__('d96c2g_passive_dynamic_frames')
            self.odom={r:[] for r in ROBOTS};self.clock=ClockAccumulator()
            self.subs=[self.create_subscription(Odometry,f'/{r}/odom',lambda msg,r=r:self.on_odom(r,msg),qos_profile_sensor_data) for r in ROBOTS]
            self.subs.append(self.create_subscription(Clock,'/clock',self.on_clock,QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)))
        def on_odom(self,robot,msg):
            datum=record_odom(msg)
            if datum and len(self.odom[robot])<40000:self.odom[robot].append(datum)
        def on_clock(self,msg):
            val=msg.clock.sec+msg.clock.nanosec*1e-9
            self.clock.append(val)
    obs=PassiveObserver()
    proc=None
    errors=[]
    threads=[]
    try:
        proc=subprocess.Popen(['gz','topic','-e','-t',gz_topic],stdout=subprocess.PIPE,stderr=subprocess.PIPE,
                              text=True,bufsize=1,start_new_session=True,errors='replace')
        def pose_reader():
            try:
                for line in proc.stdout:parser.feed(line)
            except Exception as exc:errors.append('Gazebo pose read: '+repr(exc))
        def err_reader():
            try:
                for line in proc.stderr:
                    if len(errors)<10:errors.append(line.strip()[:256])
            except Exception:pass
        threads=[threading.Thread(target=pose_reader,daemon=True),threading.Thread(target=err_reader,daemon=True)]
        for th in threads:th.start()
        deadline=time.monotonic()+duration
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(obs,timeout_sec=0.07)
            if proc.poll() is not None:
                errors.append('gz topic -e exited early, code '+str(proc.returncode));break
    finally:
        if proc is not None:
            proc.terminate()
            try:proc.wait(timeout=3)
            except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=2)
            for th in threads:th.join(timeout=1)
        odom=obs.odom;clock=obs.clock
        obs.destroy_node();rclpy.shutdown()
    return parser,odom,clock,errors


def interpolate_odom(odoms, t, max_skew):
    """Interpolation requires two bracketing odometry stamps, each sufficiently close."""
    stamps=[o['stamp_s'] for o in odoms]
    if not stamps:return None
    index=bisect.bisect_left(stamps,t)
    if index<len(stamps) and abs(stamps[index]-t)<1e-7:
        return dict(odoms[index], matching_dt_s=abs(stamps[index]-t), matching_method='exact')
    if index==0 or index==len(stamps):return None
    before,after=odoms[index-1],odoms[index]
    left,right=before['stamp_s'],after['stamp_s']
    if right<=left or max(t-left,right-t)>max_skew:return None
    alpha=(t-left)/(right-left)
    xyz=[before['xyz'][i]*(1-alpha)+after['xyz'][i]*alpha for i in range(3)]
    heading=before['yaw_rad']+alpha*angle(after['yaw_rad'],before['yaw_rad'])
    if before['frame_id'] != after['frame_id'] or before['child_frame_id'] != after['child_frame_id']:
        return None
    return {'xyz':xyz,'yaw_rad':heading,'stamp_s':t,
            'frame_id':before['frame_id'],'child_frame_id':before['child_frame_id'],
            'matching_dt_s':max(t-left,right-t),'matching_method':'interpolated'}


def analyze(parser, odom, clock, min_pairs=10, min_motion=.25, pos_tol=.075,
            yaw_tol_deg=4., max_skew=.04):
    report={'schema':'swarm.d96c2g.dynamic_compare.readonly.v1',
            'status':'REJECT_DYNAMIC_FRAME_COMPARISON',
            'robots':{},'sim_clock_messages':len(clock),'sim_clock_progressing':False,
            'gazebo_unstamped_pose_count':parser.unstamped,
            'gazebo_invalid_pose_count':parser.invalid,
            'gazebo_truncated_samples':parser.truncated,
            'dynamic_frame_consistency_observed':False,
            'coordinate_frame_equivalence_proven':False,
            'execution_authorized':False,'ros_executable':False,
            'publishers_created':0,'commands_sent':0,
            'note':'Time-matched world model vs ROS odom measurements only. This is NOT TF certification, does not authorize motion, and does not verify D9.2 tracking.'}
    valid_clock,clock_bounds,clock_details=summarize_clock(clock)
    report['sim_clock_progressing']=valid_clock
    report['clock_integrity']=clock_details
    if clock_bounds is not None:report['sim_clock_bounds_s']=clock_bounds
    rows=[];all_ok=valid_clock and parser.truncated==0
    for robot in ROBOTS:
        gz=sorted((g for g in parser.samples[robot] if g['stamp_s'] is not None),key=lambda g:g['stamp_s'])
        ros=sorted(odom[robot],key=lambda d:d['stamp_s'])
        expected_labels=all(o['frame_id']==f'{robot}/odom' and o['child_frame_id']==f'{robot}/base_footprint' for o in ros)
        matched=[]
        for pose in gz:
            near=interpolate_odom(ros,pose['stamp_s'],max_skew)
            if not near:continue
            if near['frame_id']!=f'{robot}/odom' or near['child_frame_id']!=f'{robot}/base_footprint':continue
            delta=math.dist(pose['xyz'][:2],near['xyz'][:2])
            yaw_delta=math.degrees(abs(angle(pose['yaw_rad'],near['yaw_rad'])))
            matched.append({'robot':robot,'sim_time_s':pose['stamp_s'],
                'gz_x':pose['xyz'][0],'gz_y':pose['xyz'][1],
                'odom_x':near['xyz'][0],'odom_y':near['xyz'][1],
                'gz_yaw_rad':pose['yaw_rad'],'odom_yaw_rad':near['yaw_rad'],
                'xy_delta_m':delta,'yaw_delta_deg':yaw_delta,
                'max_bracket_skew_s':near['matching_dt_s'],
                'matching_method':near['matching_method']})
        if matched:
            span=matched[-1]['sim_time_s']-matched[0]['sim_time_s']
            path=sum(math.hypot(b['gz_x']-a['gz_x'],b['gz_y']-a['gz_y']) for a,b in zip(matched,matched[1:]))
            extent=math.hypot(max(x['gz_x'] for x in matched)-min(x['gz_x'] for x in matched),
                              max(x['gz_y'] for x in matched)-min(x['gz_y'] for x in matched))
            pos_err=max(x['xy_delta_m'] for x in matched)
            yaw_err=max(x['yaw_delta_deg'] for x in matched)
            skew=max(x['max_bracket_skew_s'] for x in matched)
            offset_sin=sum(math.sin(angle(x['gz_yaw_rad'],x['odom_yaw_rad'])) for x in matched)
            offset_cos=sum(math.cos(angle(x['gz_yaw_rad'],x['odom_yaw_rad'])) for x in matched)
            theta=math.atan2(offset_sin,offset_cos)
            cs,sn=math.cos(theta),math.sin(theta)
            txy=[(x['gz_x']-(cs*x['odom_x']-sn*x['odom_y']),
                  x['gz_y']-(sn*x['odom_x']+cs*x['odom_y'])) for x in matched]
            t_est=[sum(v[k] for v in txy)/len(txy) for k in (0,1)]
            fit_err=max(math.hypot(x[0]-t_est[0],x[1]-t_est[1]) for x in txy)
            details={'gz_pose_samples':len(parser.samples[robot]),'timestamped_gz_samples':len(gz),
                'odom_samples':len(ros),'odom_frames_expected':expected_labels,'matched_pairs':len(matched),'matched_span_s':round(span,6),
                'motion_path_length_m':round(path,6),'motion_extent_m':round(extent,6),
                'max_xy_delta_m':pos_err,'max_yaw_delta_deg':yaw_err,
                'max_timestamp_bracket_s':skew,
                'estimated_world_from_odom_xy_m':t_est,
                'estimated_world_from_odom_yaw_deg':math.degrees(theta),
                'rigid_fit_translation_variation_max_m':fit_err}
            sufficient=len(matched)>=min_pairs and span>=1. and path>=min_motion and extent>=min_motion/2
            near_identity=pos_err<=pos_tol and yaw_err<=yaw_tol_deg
            stable_fit=fit_err<=pos_tol and max(abs(math.degrees(angle(x['gz_yaw_rad']-x['odom_yaw_rad'],theta))) for x in matched)<=yaw_tol_deg
            details.update(sufficient_motion=sufficient, near_identity_observed=near_identity,
                           consistent_rigid_transform_observed=stable_fit,
                           status='DYNAMIC_CONSISTENT' if sufficient and near_identity and expected_labels else ('INSUFFICIENT_MOTION_OR_PAIRS' if not sufficient else 'DYNAMIC_DISAGREEMENT'))
            ok=sufficient and near_identity and expected_labels
            rows.extend(matched)
        else:
            details={'gz_pose_samples':len(parser.samples[robot]),'timestamped_gz_samples':len(gz),
                     'odom_samples':len(ros),'odom_frames_expected':expected_labels,'matched_pairs':0,
                     'status':'NO_TIME_MATCHED_PAIRS'}
            ok=False
        all_ok=all_ok and ok
        report['robots'][robot]=details
    report['total_matched_pairs']=len(rows)
    if all_ok:
        report['status']='DYNAMIC_WORLD_ODOM_CONSISTENT_OBSERVED_NOT_TF_PROVEN'
        report['dynamic_frame_consistency_observed']=True
    elif not valid_clock:
        report['status']='REJECT_CLOCK_NOT_PROGRESSING_OR_RESET'
    elif parser.unstamped and not rows:
        report['status']='REJECT_GAZEBO_POSE_TIMESTAMPS_MISSING'
    elif any(report['robots'][r]['status']=='INSUFFICIENT_MOTION_OR_PAIRS' for r in ROBOTS):
        report['status']='REJECT_INSUFFICIENT_DYNAMIC_EVIDENCE'
    return report,rows


def output_csv(filename,rows):
    columns=('robot','sim_time_s','gz_x','gz_y','odom_x','odom_y',
             'gz_yaw_rad','odom_yaw_rad','xy_delta_m','yaw_delta_deg',
             'max_bracket_skew_s','matching_method')
    with filename.open('w',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=columns)
        writer.writeheader();writer.writerows(rows)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--pose-topic',default='/world/world_demo/pose/info')
    p.add_argument('--observe-seconds',type=float,default=120.)
    p.add_argument('--max-skew',type=float,default=.04)
    p.add_argument('--min-motion',type=float,default=.25)
    args=p.parse_args()
    if not (5<=args.observe_seconds<=360):p.error('observe-seconds must be 5..360')
    if not (.005<=args.max_skew<=.1):p.error('max-skew must be 0.005..0.1')
    if not (.1<=args.min_motion<=1.):p.error('min-motion must be 0.1..1.0')
    try:
        parser,ros,clock,errors=capture(args.observe_seconds,args.pose_topic)
        result,rows=analyze(parser,ros,clock,max_skew=args.max_skew,min_motion=args.min_motion)
        result['capture_warnings']=errors[:10]
    except Exception as e:
        rows=[]
        result={'schema':'swarm.d96c2g.dynamic_compare.readonly.v1',
            'status':'REJECT_CAPTURE_ERROR','capture_error':repr(e),
            'coordinate_frame_equivalence_proven':False,'execution_authorized':False,
            'ros_executable':False,'publishers_created':0,'commands_sent':0}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    csvfile=args.output.with_name(args.output.stem+'_matched.csv')
    output_csv(csvfile,rows)
    result['csv']=str(csvfile)
    args.output.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(result,indent=2,ensure_ascii=False))
    if not result['status'].startswith('DYNAMIC_WORLD_ODOM_CONSISTENT'):
        raise SystemExit(2)


if __name__=='__main__':main()
