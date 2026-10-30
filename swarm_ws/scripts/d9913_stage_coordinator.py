#!/usr/bin/env python3
"""D9.9.13: guarded multi-stage V2 coordinator; dry-run by default.

Uses existing /summitN/path_follower and its timed_trajectory ROS interface.
Never publishes cmd_vel. Historical SHA256 is required.
"""
import argparse
import hashlib
import importlib.util
import json
import math
import sys
import time
import uuid
from pathlib import Path

import numpy as np

ROBOTS = [f'summit{i}' for i in range(3)]
HISTORICAL_SHAS = {'delivery1': '70bb3550dfb3a937f002f5b8e83d621ef8b229180f6a0b8c6bdc626de19fb40b', 'pickup2': '7f626f10002aeb40be8fd3684e8cc2ee49512ffbf01fdc969f489584c1f94e11', 'delivery2': 'ce13db58df80acdfc4bedebfc115c3a37a2bc5dad501727bfa9812ea2d1b2b60', 'pickup1': '3235223232eb1b47e36c46cc107086c933e5020917aa7b78758b9b183985e2dd'}


def load_proto(path):
    """Importar protocolo V1/V2/V3 en su paquete, no como módulo huérfano.

    V3 usa `from .successive_clock import ...`: importlib.spec_from_file_location
    con nombre suelto (d9912_timed_protocol) elimina el contexto relativo.
    Se verifica que el módulo cargado sea exactamente el archivo solicitado,
    evitando ejecutar accidentalmente otra versión instalada del protocolo.
    """
    import importlib
    path=Path(path).resolve()
    if path.name!='timed_protocol.py' or path.parent.name!='hybrid_refinement' or path.parent.parent.name!='swarm_planning':
        raise ValueError('INVALID_TIMED_PROTOCOL_PATH')
    package_root=path.parents[3]
    package_name='swarm_behavior.swarm_planning.hybrid_refinement.timed_protocol'
    root_str=str(package_root)
    if root_str not in sys.path:
        sys.path.insert(0,root_str)
    importlib.invalidate_caches()
    module=importlib.import_module(package_name)
    if Path(module.__file__).resolve()!=path:
        raise ValueError(f'TIMED_PROTOCOL_VERSION_MISMATCH: {module.__file__} != {path}')
    return module


def prepare(a, report):
    traj=Path(a.trajectory).resolve()
    digest=hashlib.sha256(traj.read_bytes()).hexdigest()
    report['trajectory_sha256']=digest
    if digest!=a.expected_sha256.lower():raise ValueError('HASH_MISMATCH_CERTIFIED_TRAJECTORY')
    proto=load_proto(Path(a.protocol_file))
    if not hasattr(proto,'SCHEMA_V2'):raise ValueError('V2_PROTOCOL_MISSING_IN_SOURCE')
    data={}
    with np.load(traj,allow_pickle=False) as z:
        for i,name in enumerate(ROBOTS):
            ctrl=np.asarray(z[f'R{i}_controls'],float)
            dur=np.asarray(z[f'R{i}_durations_s'],float)
            if ctrl.ndim!=3 or ctrl.shape[1:]!=(6,2) or dur.shape!=(len(ctrl),):
                raise ValueError(f'{name}: INVALID_SHAPE')
            if not np.isfinite(ctrl).all() or not np.isfinite(dur).all() or (dur<=0).any():
                raise ValueError(f'{name}: INVALID_VALUES')
            data[name]=(ctrl,dur)
            report['robots'][name]={'segments':len(ctrl),'duration_s':float(sum(dur)),'start_xy':ctrl[0,0].tolist()}
    # D9931 V3: no se cambia un solo control, ni se divide dt_s por otro factor.
    if a.successive_clock is not None:
        if a.auto_time_scale or abs(float(a.time_scale)-1.)>1e-9:
            raise ValueError('DOUBLE_TIME_SCALING_FORBIDDEN_IN_SUCCESSIVE_MODE')
        clock_path=Path(a.successive_clock).resolve()
        clock=json.loads(clock_path.read_text(encoding='utf8'))
        if clock.get('input_sha256')!=digest or clock.get('status')!='OFFLINE_SCHEDULE_CERTIFIED_NOT_EXECUTABLE':
            raise ValueError('SUCCESSIVE_CLOCK_MISMATCH_OR_NOT_CERTIFIED')
        if not hasattr(proto,'SCHEMA_V3'):
            raise ValueError('V3_PROTOCOL_NOT_INSTALLED')
        tdata={
            'model':'common_collective_successive_v1',
            'baseline_scale':float(clock['uniform_scale']),
            'ramps':clock['ramps'],
            'duration_original_s':float(clock['duration_original_s']),
            'duration_successive_s':float(clock['duration_successive_s'])}
        for name,(ctrl,dur) in data.items():
            report['robots'][name]['duration_s']=float(clock['finish_times_wall_s'][name])
        report['navigation_parameters']={'time_scale':1.,'time_model':'common_collective_successive_v1',
            'nav_speed_limit_m_s':a.nav_speed_limit,'nav_acceleration_limit_m_s2':a.nav_accel_limit,
            'clock_sha256':hashlib.sha256(clock_path.read_bytes()).hexdigest(),
            'duration_successive_s':tdata['duration_successive_s']}
        report['successive_timewarp']=tdata
        return proto,data
    # Reescalado temporal uniforme; puntos de control espaciales INALTERADOS.
    # En t_nuevo=t_original/f, velocidad=f*v y aceleración=f²*a.
    factor=float(a.time_scale)
    if a.auto_time_scale:
        # Cotas conservadoras sobre TODOS los tramos y robots de la etapa.
        # La propiedad de envolvente convexa garantiza los máximos globales,
        # aunque puede subestimar el factor realmente alcanzable.
        peak_v = 0.0
        peak_a = 0.0
        for ctrl,dur in data.values():
            v_cp=5*np.diff(ctrl,axis=1)/dur[:,None,None]
            a_cp=20*np.diff(ctrl,n=2,axis=1)/(dur[:,None,None]**2)
            peak_v=max(peak_v,float(np.linalg.norm(v_cp,axis=2).max()))
            peak_a=max(peak_a,float(np.linalg.norm(a_cp,axis=2).max()))
        k_v=float('inf') if peak_v<1e-10 else a.nav_speed_limit/peak_v
        k_a=float('inf') if peak_a<1e-10 else math.sqrt(a.nav_accel_limit/peak_a)
        factor=min(k_v,k_a,2.0)
        if not math.isfinite(factor) or factor<0.2:
            raise ValueError(f'AUTO_TIME_SCALE_NO_FEASIBLE_FACTOR k={factor}')
        report['auto_time_scale']={
            'active':True,'factor':factor,
            'reference_speed_envelope_m_s':peak_v,
            'reference_acceleration_envelope_m_s2':peak_a,
            'velocity_constraint_factor':k_v,
            'acceleration_constraint_factor':k_a,
            'limiting_constraint':('speed' if k_v<=k_a and k_v<=2.0 else
                                  'acceleration' if k_a<=2.0 else 'factor_cap_2'),
            'method':'uniform_stage_conservative_envelope',
            'note':'Factor único por etapa; no es el reescalado sucesivo por robots que terminan.'
        }
    for name,(ctrl,dur) in data.items():
        scaled=dur/factor
        if not np.isfinite(scaled).all() or np.min(scaled)<=1e-5:
            raise ValueError(f'{name}: DURACIONES_INVALIDAS_REESCALADAS')
        # Comprobación conservadora mediante envolvente convexa de derivadas
        # de Bézier. Nunca autorizar una trayectoria cuya cota supere el límite.
        vctrl=5*np.diff(ctrl,axis=1)/scaled[:,None,None]
        actrl=20*np.diff(ctrl,n=2,axis=1)/(scaled[:,None,None]**2)
        upper_v=float(np.linalg.norm(vctrl,axis=2).max())
        upper_a=float(np.linalg.norm(actrl,axis=2).max())
        # Para factor 1 se conserva exactamente el comportamiento histórico.
        # Con aceleración experimental se comprueba conservadoramente el perfil.
        if (a.auto_time_scale or factor>1) and upper_v>a.nav_speed_limit+1e-8:
            raise ValueError(f'{name}: SPEED_PROFILE_ENVELOPE {upper_v:.3f} > {a.nav_speed_limit:.3f} m/s')
        if (a.auto_time_scale or factor>1) and upper_a>a.nav_accel_limit+1e-8:
            raise ValueError(f'{name}: ACCEL_PROFILE_ENVELOPE {upper_a:.3f} > {a.nav_accel_limit:.3f} m/s²')
        data[name]=(ctrl,scaled)
        report['robots'][name].update(duration_s=float(scaled.sum()),
                                      control_speed_envelope_m_s=upper_v,
                                      control_accel_envelope_m_s2=upper_a)
    report['navigation_parameters']={
        'time_scale':factor,'nav_speed_limit_m_s':a.nav_speed_limit,
        'nav_acceleration_limit_m_s2':a.nav_accel_limit,
        'note':'Los límites del seguidor ROS también deben ser >= nav_speed_limit; D8 agarre no modificado.'}
    return proto,data


def run_ros(a,proto,data,report):
    import rclpy
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    class Coordinator(Node):
        def __init__(self):
            super().__init__('d9913_stage_coordinator')
            self.clock_times=[];self.odoms={};self.statuses={n:[] for n in ROBOTS}
            self.create_subscription(Clock,'/clock',self.clock_cb,10)
            for n in ROBOTS:
                self.create_subscription(Odometry,f'/{n}/odom',lambda m,n=n:self.odom_cb(n,m),10)
                self.create_subscription(String,f'/{n}/hybrid_trajectory/status',lambda m,n=n:self.status_cb(n,m),10)
            self.pubs={}
        def clock_cb(self,msg):
            t=msg.clock.sec+msg.clock.nanosec*1e-9;self.clock_times.append(t)
        def odom_cb(self,n,msg):
            p=msg.pose.pose.position;v=msg.twist.twist.linear
            self.odoms[n]={'xy':(p.x,p.y),'speed':math.hypot(v.x,v.y),'frame':msg.header.frame_id,
                            'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9}
        def status_cb(self,n,msg):
            try:
                robot,session,state=proto.decode_status(msg.data)
                if robot==n and session==report['session_id']:
                    self.statuses[n].append((state,time.monotonic()))
            except Exception:pass
    rclpy.init();node=Coordinator();published=False
    def spin_for(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    def publish_cancel():
        if not published:return
        for n in ROBOTS:
            raw=json.dumps({'schema':proto.SCHEMA_V2,'action':'cancel','session_id':report['session_id'],'robot':n})
            node.pubs[n].publish(String(data=raw))
        spin_for(.5)
    try:
        spin_for(a.observe_seconds)
        if len(node.clock_times)<3 or node.clock_times[-1]-node.clock_times[0]<1.:
            raise RuntimeError('CLOCK_NOT_ADVANCING')
        latest=node.clock_times[-1]
        for n in ROBOTS:
            pubs=node.get_publishers_info_by_topic(f'/{n}/cmd_vel')
            expected=[p for p in pubs if p.node_name=='path_follower' and p.node_namespace.rstrip('/')==f'/{n}']
            if len(pubs)!=1 or len(expected)!=1:raise RuntimeError(f'{n}: CMD_VEL_OWNER_MISMATCH')
            od=node.odoms.get(n)
            if od is None:raise RuntimeError(f'{n}: ODOM_MISSING')
            err=math.dist(od['xy'],data[n][0][0,0])
            report['robots'][n].update(error_m=err,speed_m_s=od['speed'],odom_frame=od['frame'])
            if err>a.start_tolerance or od['speed']>a.max_stationary_speed or od['frame']!=f'{n}/odom':
                raise RuntimeError(f'{n}: ALIGNMENT_OR_STATIONARY_CHECK_FAILED')
            if not 0<=latest-od['stamp']<=.5:raise RuntimeError(f'{n}: ODOM_STALE')
        report['clock_s']=latest
        if not a.execute:
            report['status']='PASS_GUARDED_DRY_RUN_NO_COMMANDS';return
        # Must be rebuilt and the live Gazebo followers restarted before this point.
        # The pre-arm test is intentionally required by explicit operator confirmation.
        if not a.confirm_runtime_v2:
            raise RuntimeError('REQUIRE_EXPLICIT_RUNTIME_V2_CONFIRMATION')
        start_time=latest+a.start_delay
        report['start_time_s']=start_time
        for n in ROBOTS:
            node.pubs[n]=node.create_publisher(String,f'/{n}/hybrid_trajectory/command',10)
        spin_for(.5)
        if any(node.pubs[n].get_subscription_count()!=1 for n in ROBOTS):
            raise RuntimeError('TIMED_COMMAND_SUBSCRIBER_MISSING_OR_DUPLICATED')
        report['commands_sent']=0
        for n in ROBOTS:
            c,d=data[n]
            cmd=dict(schema=proto.SCHEMA_V3 if 'successive_timewarp' in report else proto.SCHEMA_V2,action='start',session_id=report['session_id'],robot=n,
                     frame_id=f'{n}/odom',start_time_s=start_time,control_points_m=c.tolist(),
                     segment_durations_s=d.tolist(),speed_limit_m_s=a.nav_speed_limit,acceleration_limit_m_s2=a.nav_accel_limit,
                     max_tracking_error_m=.15,goal_tolerance_m=.05,goal_settle_s=.5,finish_grace_s=4.)
            if 'successive_timewarp' in report: cmd['timewarp']=report['successive_timewarp']
            raw=json.dumps(cmd,separators=(',',':'),allow_nan=False)
            proto.decode_command(raw)
            node.pubs[n].publish(String(data=raw));report['commands_sent']+=1
        published=True
        # Arm barrier. No robot should start until common start_time.
        deadline=time.monotonic()+min(a.start_delay-1.,15.)
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if any('failed' in [s for s,_ in node.statuses[n]] for n in ROBOTS):
                raise RuntimeError('FOLLOWER_REJECTED_START')
            if all('armed' in [s for s,_ in node.statuses[n]] for n in ROBOTS):break
        else:raise RuntimeError('ARM_BARRIER_TIMEOUT')
        report['status']='ARMED_ALL'
        deadline=time.monotonic()+max(report['robots'][n]['duration_s'] for n in ROBOTS)+a.start_delay+12.
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            states={n:[s for s,_ in node.statuses[n]] for n in ROBOTS}
            if any('failed' in states[n] or 'cancelled' in states[n] for n in ROBOTS):
                raise RuntimeError('FOLLOWER_FAILED_OR_CANCELLED')
            if all('done' in states[n] for n in ROBOTS):
                report['status']='DONE_ALL_'+a.stage.upper();break
        else:raise RuntimeError('FOLLOWER_COMPLETION_TIMEOUT')
    except BaseException:
        publish_cancel()
        raise
    finally:
        report['status_history']={n:[s for s,_ in node.statuses[n]] for n in ROBOTS}
        node.destroy_node();rclpy.shutdown()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--time-scale',type=float,default=1.,help='Factor manual (ignorado con --auto-time-scale)')
    p.add_argument('--auto-time-scale',action='store_true',help='Calcula el factor común máximo por etapa desde v_max y a_max')
    p.add_argument('--successive-clock',type=Path,default=None,help='D9931: JSON reloj V3 certificado; incompatible con reescalado uniforme')
    p.add_argument('--nav-speed-limit',type=float,default=1.2,help='Límite de rapidez del comando temporizado, m/s')
    p.add_argument('--nav-accel-limit',type=float,default=1.0,help='Límite de aceleración del comando temporizado, m/s²')
    p.add_argument('--stage',choices=['pickup1','delivery1','pickup2','delivery2'],required=True)
    p.add_argument('--trajectory',default=None)
    p.add_argument('--protocol-file',default='src/swarm_behavior/swarm_behavior/swarm_planning/hybrid_refinement/timed_protocol.py')
    p.add_argument('--expected-sha256',default=None)
    p.add_argument('--output',default='scripts/resultados/d9913_stage.json')
    p.add_argument('--observe-seconds',type=float,default=5.)
    p.add_argument('--start-tolerance',type=float,default=.05)
    p.add_argument('--max-stationary-speed',type=float,default=.01)
    p.add_argument('--start-delay',type=float,default=10.)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--confirm-runtime-v2',action='store_true',help='Explicit assertion: package rebuilt and followers relaunched with V2')
    a=p.parse_args()
    if a.trajectory is None: a.trajectory=f'scripts/d996_reference/{a.stage}_d994_NON_EXECUTABLE.npz'
    if a.expected_sha256 is None: a.expected_sha256=HISTORICAL_SHAS[a.stage]
    report={'schema':'swarm.d9913.stage.coordinator.v1','stage':a.stage,'status':'REJECT','execution_requested':a.execute,
            'commands_sent':0,'problems':[],'robots':{},'session_id':'D9913_'+uuid.uuid4().hex[:12]}
    try:
        if not (0.2<=a.time_scale<=2.0) or not (0.1<=a.nav_speed_limit<=2.0) or not (0.1<=a.nav_accel_limit<=2.0):
            raise ValueError('PARAMETROS_NAVEGACION_FUERA_DE_RANGO')
        if a.start_delay<5 or a.start_delay>30:raise ValueError('START_DELAY_OUT_OF_RANGE')
        proto,data=prepare(a,report)
        run_ros(a,proto,data,report)
    except Exception as exc:report['problems'].append(str(exc));report['status']='REJECT'
    target=Path(a.output);target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(report,indent=2,ensure_ascii=False))
    return 0 if not report['problems'] else 2
if __name__=='__main__':raise SystemExit(main())
