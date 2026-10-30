#!/usr/bin/env python3
"""D9.9.20-T: ejecutor Bézier temporizado de rutas CBS AUXILIARES.

NO crea otro cmd_vel. Únicamente publica String en el puerto híbrido del
path_follower existente. No modifica el plan CBS: los tramos Bézier se apoyan
exactamente en la polilínea CBS, con parada en cada cambio de dirección.
Se comprueba C2 y cotas dinámicas vía timed_protocol V2 antes de armar.
"""
from __future__ import annotations

import importlib.util
import json
import math
import os
import sys
import time
import uuid
from pathlib import Path

ROBOT = 'summit0'
ROBOTS = ('summit0', 'summit1', 'summit2')


def _proto_from_file(path):
    path = Path(path).resolve()
    spec = importlib.util.spec_from_file_location('d9920t_proto', path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'Protocolo no disponible: {path}')
    m = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = m
    spec.loader.exec_module(m)
    if not hasattr(m, 'SCHEMA_V2'):
        raise RuntimeError('El protocolo instalado no admite V2')
    return m


def _normalise_route(points):
    pts = []
    for p in points:
        if len(p) != 2 or not all(math.isfinite(float(v)) for v in p):
            raise ValueError('Punto CBS inválido')
        q = (float(p[0]), float(p[1]))
        if not pts or math.dist(pts[-1], q) > 1e-4:
            pts.append(q)
    if len(pts) < 2:
        raise ValueError('Ruta CBS demasiado corta para ejecutarse')
    # Eliminar solo vértices sobre el mismo segmento recto y en su misma
    # dirección: no se inventan shortcuts diagonales ni se alteran esquinas.
    res = [pts[0]]
    for i in range(1, len(pts)-1):
        a = res[-1]
        b = pts[i]
        c = pts[i+1]
        v = (b[0]-a[0], b[1]-a[1])
        w = (c[0]-b[0], c[1]-b[1])
        cross = v[0]*w[1]-v[1]*w[0]
        dot = v[0]*w[0]+v[1]*w[1]
        if abs(cross) < 1e-7 * max(1.0, math.hypot(*v)*math.hypot(*w)) and dot > 0:
            continue
        res.append(b)
    res.append(pts[-1])
    return res


def build_bezier_route(points, *, speed_limit=0.65, accel_limit=0.65):
    """Produce tramos quinticos C2 sobre segmentos CBS con v=a=0 en extremos.

    Cotas conservadoras sobre el polígono de control derivado:
      Vmax<=5*L/T, Amax<=40*L/T².
    Se comprueban después mediante el protocolo V2 real.
    """
    if not 0 < speed_limit <= 1.2 or not 0 < accel_limit <= 1.0:
        raise ValueError('Límites fuera de los autorizados')
    route = _normalise_route(points)
    ctrl = []
    durations = []
    for a,b in zip(route,route[1:]):
        dist = math.dist(a,b)
        if dist <= 0.0001:
            continue
        ctrl.append([list(a),list(a),list(a),list(b),list(b),list(b)])
        T = max(2.0, 5.0*dist/speed_limit, math.sqrt(40.0*dist/accel_limit))
        durations.append(float(T)*1.02)  # pequeño margen contra redondeo
    if not ctrl:
        raise ValueError('Ruta vacía tras compactación')
    return ctrl, durations, route


def build_start_command(points, start_time_s, session_id, proto, *, speed_limit=0.65, accel_limit=0.65):
    ctrl,durations,route = build_bezier_route(points,speed_limit=speed_limit,accel_limit=accel_limit)
    cmd = {
        'schema': proto.SCHEMA_V2,
        'action': 'start',
        'session_id': session_id,
        'robot': ROBOT,
        'frame_id': f'{ROBOT}/odom',
        'start_time_s': float(start_time_s),
        'control_points_m': ctrl,
        'segment_durations_s': durations,
        'speed_limit_m_s': float(speed_limit),
        'acceleration_limit_m_s2': float(accel_limit),
        'max_tracking_error_m': 0.15,
        'goal_tolerance_m': 0.05,
        'goal_settle_s': 0.6,
        'finish_grace_s': 6.0,
    }
    raw = json.dumps(cmd, separators=(',', ':'),allow_nan=False)
    proto.decode_command(raw)
    return raw, cmd, route


def execute_route(points, *, output_report: Path, timeout_s: float = 160.0,
                  protocol_file='src/swarm_behavior/swarm_behavior/swarm_planning/hybrid_refinement/timed_protocol.py',
                  speed_limit=0.65, accel_limit=0.65):
    """Ejecuta una maniobra auxiliar de summit0 y registra odom + cmd_vel.

    La salida consiste en campos para incorporar al reporte del caller.
    Levanta excepción en cualquier fallo; manda cancel V2 al seguidor si ya
    publicó START. Supervisión de otros Summit como obstáculos estacionarios.
    """
    import rclpy
    from std_msgs.msg import String
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from rclpy.qos import qos_profile_sensor_data

    proto = _proto_from_file(protocol_file)
    session = 'D9920T_' + uuid.uuid4().hex[:16]
    route = _normalise_route(points)
    raw_template, template, _ = build_start_command(route, 1.0, session, proto,
                                 speed_limit=speed_limit, accel_limit=accel_limit)
    trajectory_duration = sum(template['segment_durations_s'])
    if trajectory_duration + 18.0 > timeout_s:
        raise RuntimeError(f'Ruta suave requiere ~{trajectory_duration:.1f}s, '+
                           f'pero timeout={timeout_s:.1f}s. No se inicia.')

    log = Path(output_report).with_name(Path(output_report).stem + '_telemetria.jsonl')
    log.parent.mkdir(parents=True, exist_ok=True)
    details = {
        'aux_controller': 'D9920T_TIMED_BEZIER_V2',
        'session_id': session,
        'trajectory_duration_s': trajectory_duration,
        'speed_limit_m_s': speed_limit,
        'acceleration_limit_m_s2': accel_limit,
        'segments': len(template['segment_durations_s']),
        'route_simplified_xy_m': route,
        'cmd_vel_odom_log': str(log),
        'timed_status_history': [],
        'commands_may_have_been_sent': False,
    }
    rclpy.init()
    node = rclpy.create_node('d9920t_timed_aux')
    stamps = {}
    latest_clock = {'sim': None, 'wall': None, 'first': None}
    odom = {}
    statuses = []
    pubs = []
    baseline = {}
    with log.open('w',encoding='utf8') as telem:
        def emit(name, msg):
            record={'kind':name,'wall_s':time.monotonic(), 'sim_s':latest_clock['sim']}
            record.update(msg)
            telem.write(json.dumps(record,separators=(',',':'))+'\n')

        def on_clock(m):
            t=float(m.clock.sec)+float(m.clock.nanosec)*1e-9
            latest_clock['sim']=t
            latest_clock['wall']=time.monotonic()
            if latest_clock['first'] is None:
                latest_clock['first']=t
        node.create_subscription(Clock,'/clock',on_clock,10)

        for robot in ROBOTS:
            def on_odom(m, robot=robot):
                p=m.pose.pose.position
                v=m.twist.twist.linear
                data = {'wall':time.monotonic(), 'xy':(float(p.x),float(p.y)),
                        'speed':math.hypot(v.x,v.y),
                        'stamp':float(m.header.stamp.sec)+float(m.header.stamp.nanosec)*1e-9,
                        'frame':m.header.frame_id}
                odom[robot]=data
                if robot==ROBOT:
                    emit('odom',{'robot':robot,'xy_m':data['xy'],'speed_m_s':data['speed']})
            node.create_subscription(Odometry,f'/{robot}/odom',on_odom,qos_profile_sensor_data)

        def on_status(m):
            try:
                robot, sess, state = proto.decode_status(m.data)
                if robot == ROBOT and sess == session:
                    statuses.append(state)
                    details['timed_status_history'].append(state)
                    emit('status', {'state':state})
            except Exception:
                pass
        node.create_subscription(String,f'/{ROBOT}/hybrid_trajectory/status',on_status,10)

        def on_cmd(m):
            emit('cmd_vel',{'linear_x':float(m.linear.x),'linear_y':float(m.linear.y),
                            'angular_z':float(m.angular.z)})
        node.create_subscription(Twist,f'/{ROBOT}/cmd_vel',on_cmd,10)

        pub = node.create_publisher(String,f'/{ROBOT}/hybrid_trajectory/command',10)
        sent = False
        try:
            # Wait until /clock, odom and subscriber have settled.
            deadline=time.monotonic()+8.0
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.05)
                if (len(odom)==3 and latest_clock['sim'] is not None and
                    pub.get_subscription_count()==1):
                    break
            else:
                raise RuntimeError('Faltan clock/odometrías o suscripción híbrida única')
            begin_clock=latest_clock['sim']
            deadline=time.monotonic()+1.5
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.05)
            if (latest_clock['sim'] - begin_clock < 0.6 or
                    time.monotonic()-latest_clock['wall']>.5):
                raise RuntimeError('/clock no avanza antes de armar')
            sim=latest_clock['sim']
            for robot in ROBOTS:
                od=odom[robot]
                if time.monotonic()-od['wall'] > 0.5:
                    raise RuntimeError(f'{robot}: odometría sin actualizar')
                if od['frame'] != f'{robot}/odom' or not 0<=sim-od['stamp']<=0.5:
                    raise RuntimeError(f'{robot}: frame/time odom inválido')
                if od['speed'] > 0.035:
                    raise RuntimeError(f'{robot}: no está detenido ({od["speed"]:.3f}m/s)')
                baseline[robot]=od['xy']
            if math.dist(baseline[ROBOT],route[0])>.045:
                raise RuntimeError('La trayectoria auxiliar no coincide con odometría actual')
            # Single owner check, consistent with D9.9.13.
            owners=node.get_publishers_info_by_topic(f'/{ROBOT}/cmd_vel')
            if (len(owners)!=1 or owners[0].node_name!='path_follower'
                    or owners[0].node_namespace.rstrip('/')!=f'/{ROBOT}'):
                raise RuntimeError('Propietario /cmd_vel inesperado: no se arma')
            start=sim+5.0
            raw,cmd,_=build_start_command(route,start,session,proto,
                           speed_limit=speed_limit,accel_limit=accel_limit)
            command_file = Path(output_report).with_name(Path(output_report).stem + '_comando_v2.json')
            command_file.write_text(json.dumps(cmd,indent=2,ensure_ascii=False)+'\n', encoding='utf8')
            details['command_v2_json'] = str(command_file)
            pub.publish(String(data=raw))
            sent=True
            details['commands_may_have_been_sent']=True
            emit('start',{'start_time_s':start, 'goal_xy_m':route[-1]})
            # Arm barrier BEFORE the robot starts.
            deadline=time.monotonic()+3.5
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.05)
                if 'failed' in statuses or 'cancelled' in statuses:
                    raise RuntimeError('Seguidor rechazó inicio Bézier V2')
                if 'armed' in statuses: break
            else:
                raise RuntimeError('Sin confirmación ARMED de seguidor V2')
            if 'armed' not in statuses:
                raise RuntimeError('Seguidor no armado')
            # Observe until DONE; fail on clock, own odom, or other moving Summit.
            deadline=time.monotonic()+timeout_s
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.05)
                now_wall=time.monotonic()
                if latest_clock['wall'] is None or now_wall-latest_clock['wall']>1.0:
                    raise RuntimeError('/clock perdido durante despeje')
                for robot in ROBOTS:
                    od=odom[robot]
                    if now_wall-od['wall']>1.0:
                        raise RuntimeError(f'Odometría vencida de {robot}')
                    if robot != ROBOT and math.dist(od['xy'],baseline[robot])>0.10:
                        raise RuntimeError(f'{robot} se desplazó; no es obstáculo fijo')
                    if robot != ROBOT and od['speed']>0.10:
                        raise RuntimeError(f'{robot} recibió movimiento simultáneo')
                if any(s in statuses for s in ('failed','cancelled')):
                    raise RuntimeError('Seguidor abortó la maniobra: '+repr(statuses))
                if 'done' in statuses:
                    if 'running' not in statuses:
                        raise RuntimeError('DONE sin estado RUNNING registrado')
                    break
            else:
                raise RuntimeError('Timeout esperando DONE del seguidor')
            # Hold final odometry position and rest, independently of DONE status.
            still_since=None
            deadline=time.monotonic()+4.0
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.05)
                od=odom[ROBOT]
                if time.monotonic()-od['wall']>1.0:
                    raise RuntimeError('Odometría vencida al verificar destino')
                if math.dist(od['xy'],route[-1])<0.08 and od['speed']<0.045:
                    if still_since is None:still_since=time.monotonic()
                    elif time.monotonic()-still_since>=.6:break
                else:
                    still_since=None
            else:
                raise RuntimeError('El Summit no quedó detenido en destino')
            details['final_xy_m']=odom[ROBOT]['xy']
            details['final_speed_m_s']=odom[ROBOT]['speed']
            return details
        except BaseException:
            if sent:
                try:
                    cancel=json.dumps({'schema':proto.SCHEMA_V2,'action':'cancel',
                                       'session_id':session,'robot':ROBOT})
                    for _ in range(3):
                        pub.publish(String(data=cancel))
                        rclpy.spin_once(node,timeout_sec=.05)
                except Exception:
                    pass
            raise
        finally:
            telem.flush()
            node.destroy_node()
            rclpy.shutdown()
