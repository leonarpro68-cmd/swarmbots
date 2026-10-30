#!/usr/bin/env python3
"""D9.6C.1: passive ROS clock+odometry shadow controller (NO publishers).

Only computes and records hypothetical body-frame velocity commands. Neither
this script nor any imported module instantiates a ROS publisher.
"""
import argparse
import csv
import json
import math
import time
from pathlib import Path

from d96a_readonly_follower import load_stage, sample

ROBOTS=('summit0','summit1','summit2')
GAZEBO_FRAMES={r: f'{r}/odom' for r in ROBOTS}


def reference_start_offsets(start_refs, scenario_positions):
    """Check planned starts against scenario. Purely diagnostic; not a TF calibration.

    A slight difference between a CBS/Bezier path start and the spawned model
    is NOT an odometry/scenario error. Those two tolerances must stay separate.
    """
    if set(scenario_positions) != set(ROBOTS) or set(start_refs) != set(ROBOTS):
        raise ValueError('Expected precisely three robots in trajectory and scenario')
    result = {}
    for robot in ROBOTS:
        ref = start_refs[robot]
        sx, sy = scenario_positions[robot]
        rx, ry = float(ref['x_m']), float(ref['y_m'])
        if not all(map(math.isfinite, (rx, ry, sx, sy))):
            raise ValueError(f'{robot}: nonfinite trajectory/scenario start')
        result[robot] = {
            'trajectory_xy_m': [rx, ry],
            'scenario_xy_m': [float(sx), float(sy)],
            'offset_m': math.hypot(rx-sx, ry-sy),
        }
    return result


def inspect_reference_starts(run, stage):
    """Inspect plan/scenario mismatch without starting any ROS process."""
    _, trajectories = load_stage(run, stage)
    scenario = json.loads((run/'scenario.json').read_text())
    positions = {r['name']: tuple(map(float, r['xy_m'])) for r in scenario['robots']}
    refs = sample(trajectories, 0.0)
    return reference_start_offsets(refs, positions)


def yaw_from_quaternion(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def shadow_command(ref, observation, kp=0.8, vmax=1.2, amax=1.0, previous=None, dt=None):
    """Reference feedforward + proportional feedback, world->base, rate limited.

    A shadow command is not a certified dynamically feasible trajectory. If
    tracking is unsafe or observations are stale the caller MUST reject it.
    """
    ex=ref['x_m']-observation['x_m'];ey=ref['y_m']-observation['y_m']
    wx=ref['vx_m_s']+kp*ex;wy=ref['vy_m_s']+kp*ey
    yaw=observation['yaw_rad'];c=math.cos(yaw);s=math.sin(yaw)
    vx=c*wx+s*wy;vy=-s*wx+c*wy
    norm=math.hypot(vx,vy)
    if norm>vmax:vx*=vmax/norm;vy*=vmax/norm
    if previous is not None and dt is not None and dt>0:
        dx=vx-previous[0];dy=vy-previous[1];d=math.hypot(dx,dy)
        if d>amax*dt:
            vx=previous[0]+dx*amax*dt/d
            vy=previous[1]+dy*amax*dt/d
    return (vx,vy),math.hypot(ex,ey)


def decide(refs, observations, clock_s, receipt_monotonic, last_odom_mono,
           expected_frames=None, odom_timeout=.5, max_error=.6, min_sep=1.15, expected_reference_frame='map', gazebo_aligned=False, alignment_ok=False):
    for name in ROBOTS:
        if name not in observations or name not in last_odom_mono:
            return 'BLOCKED_NO_ODOM'
        if receipt_monotonic-last_odom_mono[name]>odom_timeout:
            return 'BLOCKED_STALE_ODOM'
        ob=observations[name]
        if not all(math.isfinite(ob[k]) for k in ('x_m','y_m','yaw_rad')):
            return 'BLOCKED_INVALID_ODOM'
        if expected_frames is not None and ob.get('frame_id')!=expected_frames.get(name):
            return 'BLOCKED_FRAME_CHANGE'
        if ob.get('frame_id') != expected_reference_frame:
            if not (gazebo_aligned and alignment_ok and ob.get('frame_id')==GAZEBO_FRAMES[name]):
                return 'BLOCKED_REFERENCE_FRAME_MISMATCH'
        if math.hypot(refs[name]['x_m']-ob['x_m'],refs[name]['y_m']-ob['y_m'])>max_error:
            return 'BLOCKED_TRACKING_ERROR'
    for i in range(3):
        for j in range(i+1,3):
            a=observations[ROBOTS[i]];b=observations[ROBOTS[j]]
            if math.hypot(a['x_m']-b['x_m'],a['y_m']-b['y_m'])<min_sep:
                return 'BLOCKED_REAL_SEPARATION'
    return 'SHADOW_OK'


def run(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    manifest, trajectories=load_stage(args.run,args.stage)
    # Independently validate start positions against the original scenario and
    # the actual converted trajectories. Never infer a TF from proximity alone.
    scenario=json.loads((args.run/'scenario.json').read_text())
    specified={r['name']:tuple(map(float,r['xy_m'])) for r in scenario['robots']}
    if set(specified)!=set(ROBOTS):
        raise ValueError('Scenario must contain exactly the three expected robots')
    start_refs=sample(trajectories,0.0)
    reference_offsets=reference_start_offsets(start_refs,specified)
    # Distinct validation gates: odometry-to-scenario stays at 0.10 m by
    # default. A planned Bézier start may differ slightly from the spawn
    # coordinate due to how the geometric route was generated. In shadow mode
    # only, permit up to 0.15 m and report the exact discrepancy. Do NOT
    # translate the trajectory, publish TF, or infer coordinate-frame equality.
    ref_limit = args.reference_start_tolerance if args.gazebo_aligned_shadow else args.initial_tolerance
    bad={n:row['offset_m'] for n,row in reference_offsets.items() if row['offset_m']>ref_limit}
    if bad:
        details=', '.join(f'{n}={d:.3f} m' for n,d in bad.items())
        raise ValueError(f'Trajectory/scenario planned-start mismatch ({details}); limit={ref_limit:.3f} m. '
                         'Use --inspect-starts to inspect all three, do not relax odometry tolerances.')

    horizon=max(float(x[1].sum()) for x in trajectories.values())
    output=args.run/'d96c1_shadow';output.mkdir(parents=True,exist_ok=True)
    csvpath=output/f'{args.stage}_SHADOW_NO_COMMANDS.csv'
    reportpath=output/f'{args.stage}_shadow_report.json'
    fields=['clock_s','mission_s','robot','status','ref_x_m','ref_y_m','odom_x_m','odom_y_m',
            'error_m','hypothetical_vx_body_m_s','hypothetical_vy_body_m_s','hypothetical_speed_m_s',
            'odom_frame_id','odom_age_wall_s']
    clock_qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
    rclpy.init(args=None)
    class Shadow(Node):
        def __init__(self):
            super().__init__('d96c1_shadow_no_publishers')
            self.last_clock_wall=time.monotonic();self.boot=self.last_clock_wall
            self.clock_start=None;self.clock_last=None;self.odom={};self.odom_seen={};self.frames={}
            self.prev_cmd={};self.prev_cmd_mission={};self.status_counts={};self.rows=0;self.clock_count=0
            self.stop=None;self.last_log_t=-1e30;self.alignment_ok=False;self.alignment_errors={};self.alignment_checked=False
            self.out=csvpath.open('w',newline='');self.writer=csv.DictWriter(self.out,fieldnames=fields);self.writer.writeheader()
            self.subclock=self.create_subscription(Clock,'/clock',self.on_clock,clock_qos)
            self.subodom=[self.create_subscription(Odometry,f'/{name}/odom',lambda msg,n=name:self.on_odom(n,msg),qos_profile_sensor_data) for name in ROBOTS]
            from rclpy.clock import Clock as RosClock
            from rclpy.clock_type import ClockType
            self.watch=self.create_timer(.2,self.watchdog,clock=RosClock(clock_type=ClockType.STEADY_TIME))
            self.get_logger().info('D9.6C.1 SHADOW. NO PUBLISHERS. Gazebo robot odometry REQUIRED; will block on mismatch.')
        def on_odom(self,name,msg):
            p=msg.pose.pose.position
            q=msg.pose.pose.orientation
            qnorm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
            yaw=yaw_from_quaternion(q) if math.isfinite(qnorm) and abs(qnorm-1.)<0.02 else float('nan')
            obs={'x_m':float(p.x),'y_m':float(p.y),'yaw_rad':yaw,'frame_id':msg.header.frame_id,
                 'child_frame_id':msg.child_frame_id}
            if name in self.frames and self.frames[name]!=obs['frame_id']:
                self.stop='ABORT_ODOM_FRAME_CHANGE';return
            self.frames.setdefault(name,obs['frame_id'])
            self.odom[name]=obs;self.odom_seen[name]=time.monotonic()
        def on_clock(self,msg):
            if self.stop:return
            stamp=float(msg.clock.sec)+1e-9*float(msg.clock.nanosec)
            if not math.isfinite(stamp) or stamp<=0:return
            now=time.monotonic();self.last_clock_wall=now;self.clock_count+=1
            if self.clock_last is not None and stamp<self.clock_last-1e-6:
                self.stop='ABORT_CLOCK_REWIND';return
            self.clock_last=stamp
            # Do not anchor mission until all three robots have odometry.
            if self.clock_start is None:
                if len(self.odom)!=3:return
                self.alignment_errors={name:math.hypot(self.odom[name]['x_m']-specified[name][0],
                                                       self.odom[name]['y_m']-specified[name][1])
                                       for name in ROBOTS}
                self.alignment_checked=True
                self.alignment_ok=all(math.isfinite(d) and d<=args.initial_tolerance for d in self.alignment_errors.values()) and all(
                    self.odom[name]['frame_id']==GAZEBO_FRAMES[name] and
                    self.odom[name]['child_frame_id']==f'{name}/base_footprint' and
                    math.isfinite(self.odom[name]['yaw_rad']) for name in ROBOTS)
                self.clock_start=stamp
            t=min(stamp-self.clock_start,horizon)
            if t-self.last_log_t<args.log_step and t<horizon:return
            refs=sample(trajectories,t)
            status=decide(refs,self.odom,stamp,now,self.odom_seen,self.frames,
                          args.odom_timeout,args.max_error,args.min_sep,args.reference_frame,
                          args.gazebo_aligned_shadow,self.alignment_ok)
            self.status_counts[status]=self.status_counts.get(status,0)+1
            for name in ROBOTS:
                ref=refs[name];ob=self.odom.get(name)
                if ob:
                    previous=self.prev_cmd.get(name)
                    delta=t-self.prev_cmd_mission.get(name,t)
                    command,error=shadow_command(ref,ob,args.kp,args.vmax,args.amax,
                                                 previous if status=='SHADOW_OK' else None,
                                                 delta if status=='SHADOW_OK' else None)
                    if status=='SHADOW_OK':
                        self.prev_cmd[name]=command;self.prev_cmd_mission[name]=t
                    else:command=(0.,0.)
                else:command=(0.,0.);error=float('nan')
                self.writer.writerow(dict(clock_s=f'{stamp:.9f}',mission_s=f'{t:.9f}',robot=name,
                    status=status,ref_x_m=ref['x_m'],ref_y_m=ref['y_m'],
                    odom_x_m=ob['x_m'] if ob else '',odom_y_m=ob['y_m'] if ob else '',
                    error_m=error,hypothetical_vx_body_m_s=command[0],
                    hypothetical_vy_body_m_s=command[1],hypothetical_speed_m_s=math.hypot(*command),
                    odom_frame_id=ob.get('frame_id','') if ob else '',
                    odom_age_wall_s=now-self.odom_seen[name] if name in self.odom_seen else ''))
                self.rows+=1
            self.last_log_t=t
            if t>=horizon:self.stop='COMPLETE_SHADOW_ONLY'
        def watchdog(self):
            now=time.monotonic()
            if now-self.boot>args.max_wall:self.stop='ABORT_MAX_WALL'
            elif now-self.last_clock_wall>args.clock_timeout:self.stop='ABORT_CLOCK_STALLED'
            elif self.clock_start is None and now-self.boot>args.odom_wait:self.stop='ABORT_NO_THREE_ODOM'
    node=None
    try:
        node=Shadow()
        while rclpy.ok() and not node.stop:rclpy.spin_once(node,timeout_sec=.2)
    finally:
        if node:
            node.out.flush();node.out.close()
            report={'stage':args.stage,'status':node.stop or 'INTERRUPTED',
                    'shadow_only':True,'ros_executable':False,'publisher_count':0,'commands_sent':0,
                    'planned_horizon_s':horizon,'clock_messages':node.clock_count,
                    'odom_seen':{n:n in node.odom_seen for n in ROBOTS},
                    'rows_written':node.rows,'shadow_status_counts':node.status_counts,
                    'mission_clock_anchor_s':node.clock_start,'csv':str(csvpath),
                    'gazebo_alignment_opt_in':bool(args.gazebo_aligned_shadow),
                    'initial_alignment_checked':node.alignment_checked,
                    'initial_alignment_ok':node.alignment_ok,
                    'initial_alignment_errors_m':node.alignment_errors,
                    'reference_start_checked':True,
                    'reference_start_tolerance_m':ref_limit,
                    'reference_scenario_start_offsets_m':{n:d['offset_m'] for n,d in reference_offsets.items()},
                    'reference_start_details':reference_offsets,
                    'observed_odom_frames':node.frames,
                    'note':'Clock anchor is first stamp after all odom arrive. Robots are stationary unless another controller moves them; computed commands are NOT sent.'}
            reportpath.write_text(json.dumps(report,indent=2)+'\n')
            node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',required=True,type=Path)
    p.add_argument('--stage',default='pickup1',choices=('pickup1','delivery1','pickup2','delivery2'))
    p.add_argument('--log-step',type=float,default=.05)
    p.add_argument('--clock-timeout',type=float,default=5.)
    p.add_argument('--odom-wait',type=float,default=15.)
    p.add_argument('--odom-timeout',type=float,default=.5)
    p.add_argument('--max-wall',type=float,default=180.)
    p.add_argument('--kp',type=float,default=.8)
    p.add_argument('--vmax',type=float,default=1.2)
    p.add_argument('--amax',type=float,default=1.)
    p.add_argument('--max-error',type=float,default=.6)
    p.add_argument('--min-sep',type=float,default=1.15)
    p.add_argument('--reference-frame',default='map',help='Marco global del plan; por defecto map')
    p.add_argument('--gazebo-aligned-shadow',action='store_true',help='EXCLUSIVAMENTE MODO SOMBRA: comparar posiciones si las 3 odometrías coinciden con scenario.json y con los nombres de marcos esperados. No establece equivalencia TF ni autoriza movimiento.')
    p.add_argument('--initial-tolerance',type=float,default=0.10,
                   help='Tolerancia de odometría real frente a scenario.json (m), predeterminado 0.10')
    p.add_argument('--reference-start-tolerance',type=float,default=0.15,
                   help='SOLO EN SOMBRA ALINEADA: límite de inicio Bézier frente a escenario (m), predeterminado 0.15')
    p.add_argument('--inspect-starts',action='store_true',
                   help='Mostrar diferencias inicio Bézier/escenario para los 3 robots y salir SIN ROS')
    args=p.parse_args()
    if any(x<=0 or not math.isfinite(x) for x in (args.log_step,args.clock_timeout,args.odom_wait,args.odom_timeout,args.max_wall,args.kp,args.vmax,args.amax,args.max_error,args.min_sep,args.initial_tolerance,args.reference_start_tolerance)):
        p.error('All numeric arguments must be positive and finite')
    if args.initial_tolerance>0.10+1e-12:
        p.error('Odometry/scenario alignment tolerance cannot exceed 0.10 m in this shadow experiment')
    if args.reference_start_tolerance>0.20+1e-12 or args.reference_start_tolerance>args.max_error:
        p.error('Reference start tolerance must be <= 0.20 m and <= max tracking error')
    if args.inspect_starts:
        details=inspect_reference_starts(args.run,args.stage)
        print(json.dumps({'stage':args.stage,'mode':'INSPECT_ONLY_NO_ROS',
                          'reference_scenario_start_offsets':details},indent=2))
        return
    result=run(args);print(json.dumps(result,indent=2))
    if result['status']!='COMPLETE_SHADOW_ONLY':raise SystemExit(2)

if __name__=='__main__':main()
