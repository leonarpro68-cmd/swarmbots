#!/usr/bin/env python3
"""D.6 adapter to SwarmBots' EXISTING turn -> RGB-D -> joint -> return routine.

Navigation Hybrid V2 must already be finished. Only one robot is handled per
invocation. This script never writes cmd_vel, never manipulates Gazebo joints,
and never infers a grasp from an arrival: the original validated
`scripts/prueba_alinear_agarrar_cbs.py` performs the physical maneuver.
"""
from __future__ import annotations

import argparse
import json
import math
import re
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

ROBOTS = ('summit0', 'summit1', 'summit2')
ACKS = ('ALINEAR CONFIRMADO', 'JOINT CONFIRMADO', 'REGRESO CONFIRMADO')


def scenario_targets(scenario: dict) -> dict[str, tuple[float, float]]:
    return {e['name']: tuple(map(float, e['xy_m'])) for e in scenario['elements']
            if e.get('type') == 'trash'}


def assigned_target(report: dict, stage: str, robot: str, scenario_path: Path) -> str:
    state = report['stages'][stage]
    if not state.get('certified') or not state.get('hybrid_v2', {}).get('certified'):
        raise ValueError(f'{stage}: reporte CBS/Hybrid V2 no certificado')
    meta_scenario = state['hybrid_v2'].get('scenario_json')
    if not meta_scenario or Path(meta_scenario).resolve() != scenario_path.resolve():
        raise ValueError('Escenario usado por la planificación no coincide con --scenario')
    names = state.get('assigned_names')
    if not isinstance(names, list) or len(names) != 3 or len(set(names)) != 3:
        raise ValueError('El reporte no contiene exactamente tres assigned_names únicos')
    return names[ROBOTS.index(robot)]


def gazebo_trash_positions(world: str) -> dict[str, tuple[float, float]]:
    """Same pose/info lookup convention as grasp_manager.py, but read only models."""
    res = subprocess.run(['gz', 'topic', '-e', '-t', f'/world/{world}/pose/info', '-n', '1'],
                         capture_output=True, text=True, timeout=10)
    if res.returncode:
        raise RuntimeError(f'No se pudo consultar pose/info: {res.stderr[-300:]}')
    lines, positions, current = res.stdout.splitlines(), {}, None
    for idx, raw in enumerate(lines):
        line = raw.strip()
        match = re.match(r'name:\s*"([^"]+)"', line)
        if match:
            current = match.group(1)
            continue
        if line.startswith('position') and current and current.startswith('trash_'):
            x = y = None
            for part in lines[idx + 1:idx + 5]:
                xm = re.match(r'\s*x:\s*([-+\d.eE]+)', part)
                ym = re.match(r'\s*y:\s*([-+\d.eE]+)', part)
                if xm: x = float(xm.group(1))
                if ym: y = float(ym.group(1))
                if x is not None and y is not None: break
            if x is not None and y is not None:
                positions[current] = (x, y)
            current = None
    return positions


def point_segment_distance(p, a, b):
    vx, vy = b[0] - a[0], b[1] - a[1]
    length_sq = vx * vx + vy * vy
    if length_sq < 1e-12:
        return math.dist(p, a)
    t = max(0., min(1., ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / length_sq))
    return math.dist(p, (a[0] + t * vx, a[1] + t * vy))


def check_preapproach(robot: str, assigned: str, odom: dict, targets: dict,
                      positions: dict, max_distance=2.2,
                      target_tolerance=.20, separation=1.15) -> dict:
    if assigned not in targets or assigned not in positions:
        raise ValueError(f'Basura {assigned!r} ausente del escenario o de Gazebo')
    xy = positions[assigned]
    discrepancy = math.dist(xy, targets[assigned])
    if discrepancy > target_tolerance:
        raise ValueError(f'{assigned}: desplazada {discrepancy:.3f} m respecto al JSON; '
                         'el controlador RGB-D original apunta al JSON. Revisar antes de maniobrar')
    rxy = odom[robot]['xy']
    dist = math.dist(rxy, targets[assigned])
    if dist < .55 or dist > max_distance:
        raise ValueError(f'{robot}: distancia centro-basura {dist:.3f} m fuera de '
                         f'[0.55, {max_distance:.2f}] m del procedimiento original; '
                         'requiere nueva aproximación global antes de usarlo')
    nearest = min(positions, key=lambda n: math.dist(rxy, positions[n]))
    if nearest != assigned:
        raise ValueError(f'{robot}: la basura más cercana al centro es {nearest}, '
                         f'pero CBS asignó {assigned}. Revisar identidad/escenario')
    # Conservative check for the intended local approach corridor among stationary robots.
    for other in ROBOTS:
        if other == robot:
            continue
        gap = point_segment_distance(odom[other]['xy'], rxy, xy)
        if gap < separation:
            raise ValueError(f'{robot}: aproximación local prevista pasa a {gap:.3f} m '
                             f'de {other} (< {separation:.2f} m). No autorizar movimiento')
    yaw = odom[robot]['yaw']
    bearing = math.atan2(targets[assigned][1] - rxy[1], targets[assigned][0] - rxy[0])
    angle = math.atan2(math.sin(bearing-yaw), math.cos(bearing-yaw))
    return {'robot': robot, 'trash': assigned, 'center_distance_m': dist,
            'turn_deg': math.degrees(angle), 'gazebo_json_offset_m': discrepancy,
            'initial_pose_xy_m': list(rxy), 'target_xy_m': list(targets[assigned]),
            'min_center_separation_m': separation}


def snapshot_odom(seconds=10.0):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node

    class Observer(Node):
        def __init__(self):
            super().__init__('d6_grasp_preapproach_check')
            self.data = {}
            for name in ROBOTS:
                self.create_subscription(Odometry, f'/{name}/odom',
                                         lambda m, name=name:self.update(name, m), 10)
        def update(self, name, msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            v = msg.twist.twist.linear
            self.data[name] = {'xy':(float(p.x),float(p.y)), 'yaw':yaw,
                               'frame':str(msg.header.frame_id),
                               'speed':math.hypot(v.x,v.y),'received':time.monotonic()}
    rclpy.init()
    node = Observer()
    try:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.10)
            if len(node.data) != 3:
                continue
            if any(time.monotonic() - d['received'] > 1.2 for d in node.data.values()):
                continue
            result = dict(node.data)
            for name in ROBOTS:
                if result[name]['frame'] != f'{name}/odom':
                    raise RuntimeError(f'{name}: odometría con frame inesperado: {result[name]["frame"]}')
                if result[name]['speed'] > .08:
                    raise RuntimeError(f'{name}: aún se mueve a {result[name]["speed"]:.3f} m/s')
            return result
        raise TimeoutError('No hay odometrías recientes de los tres Summit')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run_original(script: Path, scenario: Path, robot: str, assigned: str,
                 max_distance: float) -> None:
    cmd = [sys.executable, '-u', str(script), '--scenario', str(scenario),
           '--robot', robot, '--trash', assigned, '--max-distance', str(max_distance)]
    print('REUTILIZANDO RUTINA ORIGINAL:', ' '.join(cmd), flush=True)
    res = subprocess.run(cmd)
    if res.returncode:
        raise RuntimeError(f'{robot}: prueba_alinear_agarrar_cbs.py terminó con '
                           f'código {res.returncode}. Revisar Gazebo y ACK; NO reintentar automáticamente')
    # The original exits 0 only after 3 ACKs, including successful return to vertex.


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--report', type=Path, required=True)
    ap.add_argument('--stage', choices=['pickup1', 'pickup2'], default='pickup1')
    ap.add_argument('--scenario', type=Path, required=True)
    ap.add_argument('--robot', choices=ROBOTS, required=True,
                    help='Un solo robot por ejecución; no se mueven los otros dos')
    ap.add_argument('--world', default='world_demo')
    ap.add_argument('--max-distance', type=float, default=2.2)
    ap.add_argument('--max-object-displacement', type=float, default=.20)
    ap.add_argument('--min-separation', type=float, default=1.15)
    ap.add_argument('--execute', action='store_true')
    ap.add_argument('--world-verified', action='store_true',
                    help='Operador confirmó que summitN/odom coincide con el mundo Gazebo')
    ap.add_argument('--navigation-done', action='store_true',
                    help='Operador confirmó los tres estados done de Hybrid V2')
    ap.add_argument('--output', type=Path,
                    help='Guardar ACK de esta maniobra (solo si --execute terminó bien)')
    args = ap.parse_args()
    if args.max_distance != 2.2:
        ap.error('Conservar --max-distance 2.2 m del procedimiento original en D.6')
    if args.min_separation < 1.15:
        ap.error('No reducir el umbral interrobot de esta prueba')
    if args.execute and (not args.world_verified or not args.navigation_done):
        ap.error('Para mover el robot se requieren --world-verified y --navigation-done')
    if args.execute and args.output and args.output.exists():
        ap.error(f'Archivo de evidencia ya existe: {args.output}; no mover hasta revisar estado anterior')
    if not args.scenario.is_file() or not args.report.is_file():
        ap.error('No existe --scenario o --report')
    original = Path(__file__).resolve().with_name('prueba_alinear_agarrar_cbs.py')
    if not original.is_file():
        ap.error(f'Falta la rutina original: {original}')
    scenario = json.loads(args.scenario.read_text(encoding='utf-8'))
    report = json.loads(args.report.read_text(encoding='utf-8'))
    assigned = assigned_target(report, args.stage, args.robot, args.scenario)
    targets = scenario_targets(scenario)
    print(f'D6 {args.stage}: {args.robot} debe recoger {assigned}')
    odom = snapshot_odom()
    positions = gazebo_trash_positions(args.world)
    if not positions:
        raise RuntimeError('Gazebo no devuelve modelos trash_ en pose/info')
    check = check_preapproach(args.robot, assigned, odom, targets, positions,
                              max_distance=args.max_distance,
                              target_tolerance=args.max_object_displacement,
                              separation=args.min_separation)
    print(f'PREORIENTACION NECESARIA: giro {check["turn_deg"]:+.1f}°, '
          f'distancia robot-basura {check["center_distance_m"]:.3f} m')
    print('PREFLIGHT LOCAL OK: puede iniciarse la preorientación original; '
          'todavía NO está en radio de agarre')
    if not args.execute:
        print('DRY RUN: sin giro, sin aproximación y sin cerrar pinza')
        return
    # Check again immediately before hand-off; no other motion command is sent here.
    print('EJECUCIÓN SUPERVISADA: original preorientar -> RGB-D -> joint -> volver al vértice')
    run_original(original, args.scenario, args.robot, assigned, args.max_distance)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        payload = {'stage':args.stage, 'robot':args.robot, 'trash':assigned,
                   'original_script':str(original), 'sequence_ack':list(ACKS),
                   'preflight':check, 'verified_world_odom_by_operator':True,
                   'recorded_utc':datetime.now(timezone.utc).isoformat(),
                   'warning':'ACK de procedimiento original; verificar objeto sujeto visualmente en Gazebo'}
        if args.output.exists():
            raise RuntimeError(f'Archivo evidencia ya existe: {args.output}; no sobrescribir')
        args.output.write_text(json.dumps(payload, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
        print('EVIDENCIA:', args.output)
    print(f'AGARRE Y REGRESO CONFIRMADOS POR LA RUTINA ORIGINAL: {args.robot} / {assigned}')


if __name__ == '__main__':
    main()
