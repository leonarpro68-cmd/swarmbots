"""D.3 offline bridge: certified Hybrid V2 NPZ -> existing timed_bezier.v1 messages.

No ROS imports, publication, coordinate translations or control side effects.
Do not send output without checking fresh Gazebo odometry and shared /clock.
"""
from __future__ import annotations
import json
from pathlib import Path
import numpy as np
from ..hybrid_refinement.timed_protocol import TimedCommand, decode_command, derivative_bounds

STAGES=('pickup1','delivery1','pickup2','delivery2')


def build_commands(report_path, stage, *, session_id, start_time_s, frame_ids,
                   max_speed=1.2, max_accel=1., tracking_error=.16,
                   goal_tolerance=.12, controls_path=None):
    """Return validated JSON commands. Reject non-certified/missing data, never arm robots."""
    if stage not in STAGES: raise ValueError(f'Etapa inválida: {stage}')
    data=json.loads(Path(report_path).read_text(encoding='utf-8'))
    item=data['stages'][stage]
    if not item.get('certified') or not item.get('hybrid_v2',{}).get('certified'):
        raise ValueError('Etapa sin certificación V2')
    report=item['hybrid_v2']
    raw_path=controls_path or item.get('controls_file')
    if not raw_path: raise ValueError('No existe controls_file; exportar con --controls-dir')
    path=Path(raw_path)
    if not path.exists() and not path.is_absolute():
        path=Path(report_path).resolve().parent/path
    if not path.exists(): raise FileNotFoundError(f'Controles no encontrados: {path}')
    if len(frame_ids)!=3: raise ValueError('Se necesitan tres frame_ids verificados con odometría')
    if not isinstance(start_time_s,(float,int)) or start_time_s<=0:
        raise ValueError('start_time_s debe ser tiempo futuro de /clock, no wall-clock')
    results={}
    with np.load(path,allow_pickle=False) as f:
        dt=float(f['dt_s'])
        if not np.isfinite(dt) or abs(dt-float(report['dt_s']))>1e-6:
            raise ValueError('dt_s no coincide con reporte certificado')
        pieces=[]
        for i in range(3):
            c=np.asarray(f[f'R{i}'],dtype=float)
            if c.ndim!=3 or c.shape[1:]!=(6,2) or not np.isfinite(c).all():
                raise ValueError(f'Controles inválidos R{i}')
            pieces.append(c)
        if len({len(x) for x in pieces})!=1 or len(pieces[0])!=int(report['pieces']):
            raise ValueError('Los robots no comparten horizonte común')
        for i,c in enumerate(pieces):
            vmax,amax=derivative_bounds(c.tolist(),dt)
            if vmax>max_speed+1e-6 or amax>max_accel+1e-6:
                raise ValueError(f'R{i} supera límites del protocolo')
            robot=f'summit{i}'
            cmd=TimedCommand('start',session_id,robot,frame_id=str(frame_ids[i]),
                  start_time_s=float(start_time_s),dt_s=dt,control_points=c.tolist(),
                  speed_limit_m_s=max_speed,acceleration_limit_m_s2=max_accel,
                  max_tracking_error_m=tracking_error,goal_tolerance_m=goal_tolerance,
                  goal_settle_s=.6,finish_grace_s=4.)
            encoded=cmd.to_json()
            parsed=decode_command(encoded)
            if parsed.robot!=robot or len(parsed.control_points)!=len(c):
                raise AssertionError('Fallo de roundtrip del protocolo')
            results[robot]=encoded
    return results


def write_commands(commands, output_dir):
    target=Path(output_dir); target.mkdir(parents=True,exist_ok=True)
    for robot,payload in commands.items():
        (target/f'{robot}.json').write_text(payload+'\n',encoding='utf-8')
    return target
