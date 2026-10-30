#!/usr/bin/env python3
"""D.6: Plan ONE explicitly selected mission stage from live odometry, offline output.

Does not publish any ROS motion command. Requires exact scenario JSON matching
running Gazebo and a user-verified common global odometry frame.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

from swarm_behavior.swarm_planning.bridge import PlanningBridge
from swarm_behavior.swarm_planning.hybrid_v2.live_start import exact_start_inputs,nearest_free_starts
from swarm_behavior.swarm_planning.hybrid_v2.pipeline_c2 import run
from swarm_behavior.swarm_planning.hybrid_v2 import geometry_reference as geom
from swarm_behavior.swarm_planning.hybrid_v2 import geometry_corridors as corridors
from d88_margin_graph import margin_compatible_graph

ROBOTS=('summit0','summit1','summit2')

class Snapshot(Node):
    def __init__(self):
        super().__init__('hybrid_v2_live_start_snapshot')
        self.odoms={}; self.clock_ns=0
        for a in ROBOTS:
            self.create_subscription(Odometry,f'/{a}/odom',lambda m,a=a:self.cb(a,m),10)
        self.create_subscription(Clock,'/clock',lambda m:self.clock_cb(m),10)
    def clock_cb(self,m):
        self.clock_ns=int(m.clock.sec)*1_000_000_000+int(m.clock.nanosec)
    def cb(self,a,m):
        x,y=float(m.pose.pose.position.x),float(m.pose.pose.position.y)
        v=m.twist.twist.linear
        # Both linear components are used only as a conservative stop check.
        self.odoms[a]={'xy':(x,y),'frame':str(m.header.frame_id),'speed':math.hypot(v.x,v.y),'received':time.monotonic()}

def validate_odom_frames(odoms, expected_frame=None, accept_namespaced_world_odom=False):
    """Validate explicit frame policy; this is NOT a TF transform.

    Namespaced mode is only valid after the operator has verified that each
    namespaced odometry coordinate system agrees with the Gazebo world.
    """
    if accept_namespaced_world_odom:
        expected = {robot: f'{robot}/odom' for robot in ROBOTS}
    elif expected_frame:
        expected = {robot: expected_frame for robot in ROBOTS}
    else:
        raise ValueError('Specify --expected-frame or --accept-namespaced-world-odom')
    observed = {robot: odoms[robot]['frame'] for robot in ROBOTS}
    mismatches = {robot: (observed[robot], expected[robot]) for robot in ROBOTS
                  if observed[robot] != expected[robot]}
    if mismatches:
        raise ValueError(f'Frame mismatch (received, expected): {mismatches}')
    return observed


def snapshot(timeout,expected_frame,max_speed,accept_namespaced_world_odom=False):
    rclpy.init()
    node=Snapshot(); started=time.monotonic()
    try:
        while time.monotonic()-started < timeout:
            rclpy.spin_once(node,timeout_sec=.20)
            if node.clock_ns <= 0 or any(a not in node.odoms for a in ROBOTS): continue
            if any(time.monotonic()-node.odoms[a]['received']>1.5 for a in ROBOTS): continue
            observed_frames=validate_odom_frames(node.odoms,expected_frame,accept_namespaced_world_odom)
            if any(node.odoms[a]['speed']>max_speed for a in ROBOTS):
                raise ValueError('Robots must be stationary before snapshot')
            return tuple(node.odoms[a]['xy'] for a in ROBOTS), node.clock_ns/1e9, observed_frames
        raise TimeoutError('Odometry or /clock missing/stale')
    finally:
        node.destroy_node(); rclpy.shutdown()

STAGES=('pickup1','delivery1','pickup2','delivery2')

def verify_state(state,stage,scenario_path):
    if state.get('version')!='swarmbots.d6.v1':
        raise ValueError('Estado D6 de version inesperada')
    if str(Path(state['scenario']).resolve())!=str(scenario_path.resolve()):
        raise ValueError('Escenario del estado D6 no coincide')
    if hashlib.sha256(scenario_path.read_bytes()).hexdigest()!=state.get('scenario_sha256'):
        raise ValueError('Contenido del escenario ha cambiado durante la misión')
    completed=state.get('completed',[])
    if not isinstance(completed,list) or completed!=list(STAGES[:len(completed)]):
        raise ValueError('Orden de etapas D6 inconsistente')
    if len(completed)>=len(STAGES) or stage!=STAGES[len(completed)]:
        raise ValueError('La siguiente etapa debe ser '+str(STAGES[len(completed)] if len(completed)<4 else 'ninguna'))
    if stage=='delivery1' and len(state['round1_names'])!=3:
        raise ValueError('No hay 3 basuras recogidas confirmadas')
    if stage=='pickup2' and len(state.get('delivered',[]))!=3:
        raise ValueError('No consta entrega real ronda 1')
    if stage=='delivery2' and len(state['round2_names'])!=3:
        raise ValueError('No hay 3 basuras recogidas ronda 2')

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scenario',required=True,help='JSON EXACTO del mundo Gazebo activo')
    p.add_argument('--stage',required=True,choices=('pickup1','delivery1','pickup2','delivery2'))
    p.add_argument('--state',required=True,type=Path,help='Estado D6 verificado por el operador')
    p.add_argument('--expected-frame',help='Un único frame compartido comprobado (modo compatible)')
    p.add_argument('--accept-namespaced-world-odom',action='store_true',
                   help='Aceptar summitN/odom como coordenadas de mundo SOLO después de verificarlo en Gazebo; NO aplica TF')
    p.add_argument('--output-dir',required=True,type=Path)
    p.add_argument('--radius',type=float,default=.55)
    p.add_argument('--margin',type=float,default=.05)
    p.add_argument('--iterations',type=int,default=2)
    p.add_argument('--max-speed',type=float,default=1.2)
    p.add_argument('--max-accel',type=float,default=1.)
    p.add_argument('--max-snap',type=float,default=.75)
    p.add_argument('--timeout',type=float,default=20.)
    p.add_argument('--max-odom-speed',type=float,default=.08)
    p.add_argument('--max-ct',type=int,default=20000)
    p.add_argument('--max-time',type=int,default=65)
    a=p.parse_args()
    if a.accept_namespaced_world_odom == bool(a.expected_frame):
        p.error('Use exactamente una opción: --expected-frame O --accept-namespaced-world-odom')
    if not (.3<=a.radius<=.8 and 0<=a.margin<=.2 and .1<=a.max_snap<=.85 and a.iterations>=0):
        p.error('Invalid configuration')
    if (a.output_dir/f'{a.stage}_live_report.json').exists():
        p.error('Salida de etapa ya existe; usar otro --output-dir para evitar pisar evidencia')
    a.output_dir.mkdir(parents=True,exist_ok=True)
    positions,clock,odom_frames=snapshot(a.timeout,a.expected_frame,a.max_odom_speed,
                                        a.accept_namespaced_world_odom)
    bridge=PlanningBridge.from_json(a.scenario)
    state=json.loads(a.state.read_text(encoding='utf-8'))
    verify_state(state,a.stage,Path(a.scenario))
    for name in state['collected']:
        bridge.confirm_collected(name)
    starts,offsets=nearest_free_starts(bridge.graph,positions,max_snap_m=a.max_snap)
    print('Odometry',positions,'CBS nodes',starts,'offsets',offsets,flush=True)
    def plan_stage():
        if a.stage.startswith('pickup'):
            return bridge.plan_pickup(starts,max_ct=a.max_ct,max_time=a.max_time)
        names=state['round1_names'] if a.stage=='delivery1' else state['round2_names']
        return bridge.plan_delivery(starts,names,max_ct=a.max_ct,max_time=a.max_time)

    def optimize_stage(candidate):
        # Route endpoints always match the actual odometry; no silent snap of pose.
        adapted,metric_points,_=exact_start_inputs(
            candidate,bridge.graph,positions,max_snap_m=a.max_snap,stage=a.stage)
        old_r=geom.CLEARANCE_WALL_M;old_c=corridors.R
        try:
            geom.CLEARANCE_WALL_M=a.radius;corridors.R=a.radius
            candidate_report,candidate_controls=run(
                metric_points,adapted.paths,adapted.obstacles,
                margin=a.margin,iterations=a.iterations,
                max_speed=a.max_speed,max_accel=a.max_accel)
        finally:
            geom.CLEARANCE_WALL_M=old_r;corridors.R=old_c
        return adapted,candidate_report,candidate_controls

    plan=plan_stage()
    data,report,controls=optimize_stage(plan)
    # D.8.8: CBS's discrete dynamic graph uses the object's base radius.
    # Hybrid V2 requires the same radius plus operational margin. If a CBS
    # segment is too close, only replan on a temporarily filtered graph;
    # NEVER relax margin or bypass the original Hybrid V2 certificate.
    if (not report.get('certified') and
            'No hay separacion espacial respecto al obstaculo' in report.get('reason','')):
        original_reason=report['reason']
        print('[D8.8] CBS incompatible con reserva Hybrid V2:', original_reason,flush=True)
        try:
            with margin_compatible_graph(bridge.graph,data.obstacles,a.margin) as summary:
                if any(start not in bridge.graph.vertices for start in starts):
                    raise ValueError('Una posicion inicial viola la reserva operativa; no es seguro replantear')
                alt_plan=plan_stage()
                alt_data,alt_report,alt_controls=optimize_stage(alt_plan)
            recovery={'trigger':original_reason,'graph_filter':summary,
                      'alternative_status':alt_report.get('status'),
                      'alternative_reason':alt_report.get('reason')}
            if alt_report.get('certified') and alt_controls is not None:
                plan,data,report,controls=alt_plan,alt_data,alt_report,alt_controls
                report['replanning_d88']=recovery
                print('[D8.8] ALTERNATIVA CERTIFIED; aristas excluidas:',
                      summary['removed_edges'],flush=True)
            else:
                report['replanning_d88']=recovery
                print('[D8.8] Alternativa rechazada; movimiento prohibido:',
                      alt_report.get('reason','sin certificado'),flush=True)
        except (ValueError,RuntimeError) as ex:
            report['replanning_d88']={'trigger':original_reason,
                'alternative_status':'REJECTED','alternative_reason':str(ex)}
            print('[D8.8] Sin alternativa segura:',ex,flush=True)
    report.update({'stage':'D.4.1','mission_stage':a.stage,'status':report.get('status','REJECTED'),
                   'scenario_json':str(Path(a.scenario).resolve()),'odom_frames':odom_frames,
                   'odom_frame_policy':('verified_namespaced_world_coordinates'
                                        if a.accept_namespaced_world_odom else 'shared_frame'),
                   'odom_snapshot_time_s':clock,'odom_xy':positions,'cbs_starts':starts,
                   'snap_offsets_m':offsets,'experimental_robot_radius_m':a.radius,
                   'warning':'El modo namespaced es una equivalencia de coordenadas verificada manualmente, NO una transformación TF; identidad del escenario aún no demostrada automáticamente. NO se autoriza ejecución'})
    result={'stages':{a.stage:{'certified':bool(report.get('certified')),'hybrid_v2':report,
                                    'cbs_paths':plan.paths}}}
    if a.stage.startswith('pickup'):
        result['stages'][a.stage]['assigned_names']=[name for name,_ in plan.assigned]
    else:
        result['stages'][a.stage]['delivered_names']=list(plan.trash_names)
    report_file=a.output_dir/f'{a.stage}_live_report.json'
    if report.get('certified') and controls is not None:
        controls_file=a.output_dir/f'{a.stage}_live_controls.npz'
        np.savez_compressed(controls_file,**{f'R{i}':np.asarray(c) for i,c in enumerate(controls)},dt_s=report['dt_s'])
        result['stages'][a.stage]['controls_file']=str(controls_file.resolve())
    report_file.write_text(json.dumps(result,ensure_ascii=False,indent=2,default=str),encoding='utf-8')
    print(report['status'],report.get('reason',''), '\nJSON:',report_file)
    if report.get('certified'):
        print('Inicio exacto Bézier coincide con odometría capturada; volver a validar odometría fresca antes de cualquier ejecución.')
    return 0 if report.get('certified') else 2
if __name__=='__main__':
    raise SystemExit(main())
