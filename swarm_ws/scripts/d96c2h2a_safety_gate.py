#!/usr/bin/env python3
"""D9.6C.2H.2A — inert ROS safety gate for independent-time trajectories.

This is NOT an executor. Creates only ROS subscriptions; NEVER creates a
publisher, publishes cmd_vel, injects TF, or authorizes motion. Graph publisher
check is a snapshot, NOT an exclusivity lock. All checks fail closed.
"""
import argparse
import hashlib
import json
import math
import time
from pathlib import Path

ROBOTS = ('summit0', 'summit1', 'summit2')
SCHEMA = 'swarm.d96c2h2a.safety_shadow.v1'
CMD_TOPICS = tuple(dict.fromkeys(
    ['/cmd_vel'] + [f'/{r}/cmd_vel' for r in ROBOTS] +
    [f'/model/{r}/cmd_vel' for r in ROBOTS] +
    [f'/{r}/cmd_vel_stamped' for r in ROBOTS]
))


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def validate_config_report(report, sdf_dir=Path('/tmp/swarm_summit'),
                           source_xacro=Path('src/summit_xl_description/robots/summit_xl_omni.urdf.xacro'),
                           launch=Path('src/swarm_worlds/launch/sim_summit.launch.py'),
                           dynamic_report=Path('scripts/resultados/d9_live/d96c2g_dynamic_frames.json')):
    """Validate H.1 audit and that its generated SDF files have not changed."""
    reasons = []
    if report.get('schema') != 'swarm.d96c2h1.configuration.readonly.v1':
        reasons.append('CONFIG_SCHEMA')
    if report.get('status') != 'PASS_CONFIG_AND_DYNAMIC_OBSERVATION_NOT_TF_CERTIFIED':
        reasons.append('CONFIG_NOT_PASSED')
    if report.get('errors') or report.get('source_configuration', {}).get('passed') is not True:
        reasons.append('CONFIG_SOURCE_ERRORS')
    if report.get('dynamic_observation', {}).get('passed') is not True:
        reasons.append('CONFIG_DYNAMIC_EVIDENCE_MISSING')
    if report.get('execution_authorized') is not False or report.get('commands_sent') != 0:
        reasons.append('CONFIG_UNEXPECTED_EXECUTABLE_FLAG')
    if report.get('tf_tree_certified') is not False:
        reasons.append('CONFIG_UNEXPECTED_TF_CERTIFICATION')
    for kind,path,expect in (
        ('XACRO',source_xacro, report.get('source_configuration',{}).get('xacro_sha256')),
        ('LAUNCH',launch, report.get('source_configuration',{}).get('launch_sha256')),
        ('DYNAMIC_REPORT',dynamic_report, report.get('dynamic_observation',{}).get('sha256')),
    ):
        if not path.is_file() or not expect or sha256(path) != expect:
            reasons.append(kind+':SOURCE_CHANGED_OR_MISSING')
    for name in ROBOTS:
        row = report.get('generated_sdf', {}).get(name, {})
        path = sdf_dir / f'{name}.sdf'
        if row.get('passed') is not True or not path.is_file():
            reasons.append(f'{name}:SDF_MISSING_OR_REJECTED')
        elif row.get('sdf_sha256') != sha256(path):
            reasons.append(f'{name}:SDF_CHANGED')
    return reasons


def validate_manifest(manifest):
    reasons = []
    if manifest.get('status') != 'PASS_READ_ONLY':
        reasons.append('MANIFEST_NOT_PASSED')
    if manifest.get('ros_executable') is not False or manifest.get('publisher_enabled') is not False:
        reasons.append('MANIFEST_EXECUTABLE')
    checks = manifest.get('checks', {})
    if checks.get('polynomial_and_joins') is not True or checks.get('separation') is not True or checks.get('d94_approximation') is not True:
        reasons.append('MANIFEST_CHECKS')
    envelope = manifest.get('separation', {}).get('conservative_lower_m', float('nan'))
    if not math.isfinite(float(envelope)) or float(envelope) < 1.15:
        reasons.append('PLAN_MIN_SEPARATION')
    limits = manifest.get('limits', {})
    if not (float(limits.get('vmax_m_s', float('inf'))) <= 1.20000001 and
            float(limits.get('amax_m_s2', float('inf'))) <= 1.00000001 and
            float(limits.get('min_interrobot_m', 0)) >= 1.15):
        reasons.append('PLAN_LIMITS')
    for name in ROBOTS:
        row = manifest.get('robots', {}).get(name, {})
        if (row.get('checks_passed') is not True or
            float(row.get('speed_upper_m_s', float('inf'))) > 1.20000001 or
            float(row.get('accel_upper_m_s2', float('inf'))) > 1.00000001):
            reasons.append(f'{name}:PLAN_ENVELOPE')
    return reasons


def assess_live(planning, starts, observation, max_capture_age_s=120.0,
                max_start_error_m=0.05, max_stationary_speed_m_s=0.03):
    """Pure functional gate: never authorizes movement even if all pass."""
    reasons = []
    details = {}
    clock = observation.get('clock', {})
    current = clock.get('current_sim_s')
    snapshot = planning.get('snapshot_time_s')
    age = None
    if isinstance(current, (int, float)) and isinstance(snapshot, (int, float)):
        age = float(current) - float(snapshot)
    if (clock.get('progressing') is not True or clock.get('rewound') is True or
            age is None or not math.isfinite(age) or not 0 <= age <= max_capture_age_s):
        reasons.append('CLOCK_OR_CAPTURE_STALE')
    pub = observation.get('command_publishers', {})
    if observation.get('graph_queries', 0) < 2:
        reasons.append('ROS_GRAPH_NOT_OBSERVED')
    if any(pub.get(topic) for topic in CMD_TOPICS):
        reasons.append('OTHER_CMD_VEL_PUBLISHER_PRESENT')
    odoms = observation.get('odoms', {})
    if set(odoms) != set(ROBOTS):
        reasons.append('MISSING_ODOMETRY')
    points = {}
    for name in ROBOTS:
        d = odoms.get(name)
        if d is None:
            continue
        ref = starts.get(name)
        if ref is None:
            reasons.append(f'{name}:MISSING_REFERENCE');continue
        x, y = float(d.get('x_m', float('nan'))), float(d.get('y_m', float('nan')))
        sx, sy = float(ref['x_m']), float(ref['y_m'])
        error = math.hypot(x - sx, y - sy)
        velocity = math.hypot(float(d.get('vx_m_s', float('nan'))), float(d.get('vy_m_s', float('nan'))))
        sim_stamp=float(d.get('stamp_sim_s',float('nan')))
        synchronized = (isinstance(current,(int,float)) and math.isfinite(float(current))
                        and math.isfinite(sim_stamp) and 0 <= float(current)-sim_stamp <= .25)
        valid = all([
            synchronized, math.isfinite(error), math.isfinite(velocity),
            error <= max_start_error_m,
            velocity <= max_stationary_speed_m_s,
            d.get('frame_id') == f'{name}/odom',
            d.get('child_frame_id') == f'{name}/base_footprint',
            0 <= float(d.get('age_wall_s', float('inf'))) <= 0.5,
            abs(float(d.get('quaternion_norm', float('nan'))) - 1.0) <= 0.02,
        ])
        details[name] = {'start_error_m': error, 'speed_m_s': velocity,
                         'odom_stamp_matches_clock':synchronized,'passed': valid}
        if not valid:
            reasons.append(f'{name}:INVALID_INITIAL_ODOMETRY')
        if math.isfinite(x) and math.isfinite(y):
            points[name] = (x, y)
    if len(points) == 3:
        for i, a in enumerate(ROBOTS):
            for b in ROBOTS[i + 1:]:
                if math.dist(points[a], points[b]) < 1.15:
                    reasons.append(f'{a}-{b}:LIVE_SEPARATION')
    return sorted(set(reasons)), {'robots': details, 'capture_age_sim_s': age,
                                  'graph_snapshot_only': True}


def capture_ros(seconds):
    """Passive ROS snapshot. Subscriptions only, zero velocity publishers."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    if not 2 <= seconds <= 30:
        raise ValueError('seconds must be in [2,30]')
    rclpy.init(args=None)
    clock_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE)
    class Observer(Node):
        def __init__(self):
            super().__init__('d96c2h2a_readonly_safety_gate')
            self.odoms = {};self.clocks = []
            self.graph_queries = 0
            self.cmd_publishers = {topic: [] for topic in CMD_TOPICS}
            self.subs = [self.create_subscription(Odometry, f'/{name}/odom',
                         lambda msg,n=name: self.on_odom(n,msg), qos_profile_sensor_data)
                         for name in ROBOTS]
            self.subs.append(self.create_subscription(Clock, '/clock', self.on_clock, clock_qos))
            self.get_logger().info('NO CMD PUBLISHERS. This precheck NEVER arms or moves robots.')
        def on_odom(self,name,msg):
            p = msg.pose.pose.position;v=msg.twist.twist.linear;q=msg.pose.pose.orientation
            norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
            self.odoms[name] = {'x_m':float(p.x),'y_m':float(p.y),
                'vx_m_s':float(v.x),'vy_m_s':float(v.y),
                'quaternion_norm':norm,'frame_id':msg.header.frame_id,
                'child_frame_id':msg.child_frame_id,
                'stamp_sim_s':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                'received':time.monotonic()}
        def on_clock(self,msg):
            t=msg.clock.sec+msg.clock.nanosec*1e-9
            self.clocks.append(float(t))
            if len(self.clocks)>1024:self.clocks=self.clocks[-1024:]
        def graph_check(self):
            self.graph_queries += 1
            for topic in CMD_TOPICS:
                sources=self.get_publishers_info_by_topic(topic)
                if sources:
                    existing = self.cmd_publishers[topic]
                    for s in sources:
                        identity=f'{s.node_namespace.rstrip("/")}/{s.node_name}'
                        if identity not in existing:existing.append(identity)
    node=Observer()
    try:
        end=time.monotonic()+seconds
        next_graph=time.monotonic()
        while rclpy.ok() and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=0.05)
            now=time.monotonic()
            if now>=next_graph:
                node.graph_check();next_graph=now+0.5
        node.graph_check()
        wall=time.monotonic()
        clock=node.clocks
        rewind=any(b<a-1e-6 for a,b in zip(clock,clock[1:]))
        return {'odoms': {name:{**obs,'age_wall_s':wall-obs['received']}
                          for name,obs in node.odoms.items()},
                'clock':{'current_sim_s':clock[-1] if clock else None,
                         'progressing':bool(len(clock)>=3 and clock[-1]>clock[0]+.2),
                         'rewound':rewind,'messages':len(clock)},
                'command_publishers':node.cmd_publishers,
                'graph_queries':node.graph_queries}
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--stage',default='pickup1',choices=('pickup1','delivery1','pickup2','delivery2'))
    parser.add_argument('--config-report',type=Path,default=Path('scripts/resultados/d9_live/d96c2h1_configuration_report.json'))
    parser.add_argument('--sdf-dir',type=Path,default=Path('/tmp/swarm_summit'))
    parser.add_argument('--source-xacro',type=Path,default=Path('src/summit_xl_description/robots/summit_xl_omni.urdf.xacro'))
    parser.add_argument('--launch',type=Path,default=Path('src/swarm_worlds/launch/sim_summit.launch.py'))
    parser.add_argument('--dynamic-report',type=Path,default=Path('scripts/resultados/d9_live/d96c2g_dynamic_frames.json'))
    parser.add_argument('--observe-seconds',type=float,default=5.)
    args=parser.parse_args(argv)
    report={'schema':SCHEMA,'stage':args.stage,'status':'REJECTED',
            'execution_authorized':False,'tf_tree_certified':False,
            'ros_executable':False,'publishers_created':0,'commands_sent':0,
            'other_publishers_detected':None,'reasons':[],
            'note':'ROS graph is a snapshot, not an exclusivity lease. This program contains no command publisher or stop actuator.'}
    try:
        if not (2 <= args.observe_seconds <= 30):raise ValueError('observe-seconds out of range')
        cfg=json.loads(args.config_report.read_text())
        reasons=validate_config_report(cfg,args.sdf_dir,args.source_xacro,args.launch,args.dynamic_report)
        from d96c2c_prearm_readonly import validate_static
        from d96a_readonly_follower import sample
        planning,manifest,trajs=validate_static(args.run,args.stage)
        reasons += validate_manifest(manifest)
        starts=sample(trajs,0.)
        observed=capture_ros(args.observe_seconds)
        live_reasons,details=assess_live(planning,starts,observed)
        reasons += live_reasons
        report.update(status='PASS_INERT_SAFETY_PRECHECK' if not reasons else 'REJECTED',
                      reasons=sorted(set(reasons)),details=details,
                      ros_graph={'command_publishers':observed['command_publishers'],
                                 'graph_queries':observed['graph_queries']},
                      other_publishers_detected=any(observed['command_publishers'].values()),
                      clock=observed['clock'],
                      planning_capture_sim_s=planning['snapshot_time_s'],
                      planned_horizon_s=max(float(v[1].sum()) for v in trajs.values()),
                      certified_separation_lower_m=manifest['separation']['conservative_lower_m'],
                      converted_controls_sha256=planning['converted_controls_sha256'])
    except Exception as exc:
        report['reasons'].append(repr(exc))
    out=args.run/'d96c2h2a_safety'/f'{args.stage}_safety_report.json'
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(report,indent=2,ensure_ascii=False))
    print('Report:',out)
    return 0 if report['status']=='PASS_INERT_SAFETY_PRECHECK' else 2


if __name__=='__main__':
    raise SystemExit(main())
